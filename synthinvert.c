#define _XOPEN_SOURCE 600

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <assert.h>

#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sem.h>
#include <sys/ipc.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>

#include <jack/jack.h>
#include <jack/midiport.h>
#include <jack/ringbuffer.h>
#include <sndfile.h>

#define MYNAME "synthinvert"

#define SQR( x ) ((x) * (x))

typedef int midi_note_t;  /* 0-127, 21=A0, 60=C4 */
#define MIDI_NOTE_NONE ((midi_note_t) -1)
typedef jack_default_audio_sample_t sample_t;
typedef union {
  int val; struct semid_ds *buf; unsigned short  *array;
} semctl_arg_t;
const char *name = "synthinvert";
const char *srcport = NULL;
const char *dbfname = NULL;
midi_note_t lomidi = 0, himidi = 0;  /* range of our synth */
midi_note_t stdmidi = 60;  /* note to calibrate against */
midi_note_t transpose_semitones = 0;
int note_scale [12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
int stdmidi_npeaks = 50;  /* # of peaks to read for calibration */
int velo0 = 100;
int initial_sil_ms = 450;
double wavebrk_sil_ms = 1.5;
double note_recog_ms = 12;
double note_recog_max_ms = 250;  // how much to analyze from a waveform

typedef struct {
  char *buf1, *buf2, *ptr, *end, *other;
  size_t sz;
  pthread_mutex_t lock, switchlock;
} memfile;
void memfile_init (memfile *f, size_t sz) {
  f->sz = sz;
  f->buf1 = calloc (sz, 1); f->buf2 = calloc (sz, 1);
  assert (f->buf1 != NULL && f->buf2 != NULL);
  f->ptr = f->buf1; f->end = f->ptr + f->sz; f->other = f->buf2;
  pthread_mutexattr_t lattr;
  pthread_mutexattr_init (&lattr);
  pthread_mutexattr_settype (&lattr, PTHREAD_MUTEX_RECURSIVE);
  pthread_mutex_init (&f->lock, &lattr);
  pthread_mutex_init (&f->switchlock, &lattr);
  pthread_mutexattr_destroy (&lattr);
}
memfile *memfile_alloc (size_t sz) {
  memfile *f = malloc (sizeof (memfile));
  assert (f != NULL);
  memfile_init (f, sz);
  return f;
}
void memfile_switchbuf (memfile *f) {
  pthread_mutex_lock (&f->switchlock);
  pthread_mutex_lock (&f->lock);
  f->other [0] = '\0';
  f->ptr = f->other; f->end = f->other + f->sz;
  f->other = f->other == f->buf1 ? f->buf2 : f->buf1;
  pthread_mutex_unlock (&f->lock);
  pthread_mutex_unlock (&f->switchlock);
}
int memfile_vprintf (memfile *f, const char *fmt, va_list ap) {
  int avail = f->end - f->ptr;
  int written = vsnprintf (f->ptr, avail, fmt, ap);
  f->ptr += written; if (f->ptr > f->end) f->ptr = f->end;
  return written;
}
int memfile_printf (memfile *f, const char *fmt, ...) {
  va_list ap; va_start (ap, fmt);
  int written = memfile_vprintf (f, fmt, ap);
  va_end (ap);
  return written;
}

typedef enum {
  TRACE_NONE, TRACE_FATAL, TRACE_ERR, TRACE_WARN, TRACE_IMPT, TRACE_INFO, TRACE_DIAG, TRACE_INT
} trace_pri_t;
const char *trace_level_symb = "-FEW!ID          ";
trace_pri_t trace_level = TRACE_INT;
int trace_print_tid = 0, trace_print_fn = 0;
memfile *trace_buf = NULL;
FILE *stdtrace = NULL;
void trace_msg (trace_pri_t pri, const char *fn, const char *fmt, ...) {
  if (pri > trace_level) return;
  struct timeval tv;
  char timestr [80];
  size_t timestrsz = sizeof (timestr) - 3;
  if (gettimeofday (&tv, NULL) == 0) {
    struct tm t;
    if (localtime_r (& tv.tv_sec, &t) == NULL)
      strcpy (timestr, "<unknown error>");
    char *timestrp = timestr +
      strftime (timestr, timestrsz, "%Y-%m-%d %H:%M:%S.", &t);
    snprintf (timestrp, 4, "%03d", (int) (tv.tv_usec / 1000));
  } else {
    if (strerror_r (errno, timestr, timestrsz) != 0)
      strcpy (timestr, "<unknown error>");
  }
  pthread_mutex_lock (&trace_buf->lock);
  memfile_printf (trace_buf, "%c %s ", trace_level_symb [pri], timestr);
  va_list ap; va_start (ap, fmt);
  memfile_vprintf (trace_buf, fmt, ap);
  if (trace_print_fn) memfile_printf (trace_buf, " [%s]", fn);
  if (trace_print_tid)
    memfile_printf (trace_buf, " [tid=0x%X]", (unsigned) pthread_self ());
  memfile_printf (trace_buf, "\n");
  pthread_mutex_unlock (&trace_buf->lock);
  va_end (ap);
}
void trace_flush () {
  pthread_mutex_lock (&trace_buf->switchlock);
  memfile_switchbuf (trace_buf);
  fprintf (stdtrace, "%s", trace_buf->other);
  pthread_mutex_unlock (&trace_buf->switchlock);
  fflush (stdtrace);
}
#define TRACE( pri, ... ) trace_msg (pri, __func__, __VA_ARGS__)
#define TRACE_PERROR( pri, called) do {                     \
    char buf [1000]; strerror_r (errno, buf, sizeof (buf)); \
    TRACE (pri, "%s: %s", called, buf);                     \
  } while (0)
#define TRACE_ASSERT( cond_, fail ) do {                                \
    if (! (cond_)) {                                                    \
      TRACE (TRACE_FATAL, "Assertion %s failed (%s:%d)",                \
             #cond_, __FILE__, __LINE__);                               \
      fail;                                                             \
    }                                                                   \
  } while (0)

#if 0
#include <execinfo.h>
void print_backtrace () {
  void *array[30];
  size_t size = backtrace (array, 30);
  char **strings = backtrace_symbols (array, size);
  
  TRACE (TRACE_DIAG, "Obtained %zd stack frames", size);
  for (size_t i = 0; i < size; ++i)
    TRACE (TRACE_DIAG, "%s", strings [i]);  
  free (strings);
}
#endif

sf_count_t my_sf_tell (SNDFILE *sndfile)
{ return sf_seek (sndfile, 0, SEEK_CUR); }

typedef struct {
  int npeaks, max_peak_idx;
  sample_t x, peak, old_peak, max_x, max_peak, noise_peak;
  jack_nframes_t count, voiced_ago, peak_at;
  int evt;
} sample_anl_state;
enum { ANL_EVT_PEAK = 0, ANL_EVT_ZERO, ANL_EVT_SIL };

typedef struct {
  sample_t x;
  jack_nframes_t length;
} gridpt;
typedef struct {
  gridpt *seq;
  size_t length, dur;
  int type;
  midi_note_t note;
} gridpt_seq;
int gridpt_seq_type (gridpt *seq) {
  if (seq [0].x > 0) return 0;
  else if (seq [0].x < 0) return 1;
  else if (seq [1].x < 0) return 2;
  else return 3;
}
int gridpt_seq_cmp (const void *s1_, const void *s2_) {
  const gridpt_seq *s1 = s1_, *s2 = s2_;
  if      (s1->length < s2->length) return -1;
  else if (s1->length > s2->length) return 1;
  else if (s1->type < s2->type) return -1;
  else if (s1->type > s2->type) return 1;
  else return 0;
}

static const midi_note_t note_semitones [] = { 0, 2, 4, 5, 7, 9, 11, 12 };
double note_midi2freq (midi_note_t midi) {
  return 27.5 * exp (.05776226504666210911 * (midi - 21));  // A0 = 27.5
}
void note_midi2sci (midi_note_t midi, char *sci) {
  midi_note_t o = midi / 12 - 1, s = midi % 12;
  int sharp = 0;
  char l;
  for (l = 'C'; l <= 'I' && note_semitones [l - 'C'] < s; l++);
  if (note_semitones [l - 'C'] > s) { sharp = 1; l--; }
  if (l >= 'H') l += 'A' - 'H';
  sprintf (sci, "%c%d%s", l, o, sharp ? "#" : "");
}
midi_note_t note_sci2midi (const char *scinote) {
  if (! isalpha (scinote [0])) goto bad;
  char ul = toupper (scinote [0]);

  if (ul >= 'H') goto bad;
  if (ul < 'C') ul += 'H' - 'A';
  if (! isdigit (scinote [1])) goto bad;
  midi_note_t l =  ul - 'C',
    o = scinote [1] - '0';
  int sharp = 0;
  if (scinote [2] == '#') sharp = 1;
  else if (scinote [2] == 'b') sharp = -1;
  // else if (scinote [2] != '\0') goto bad;

  return 12 * (o + 1) + note_semitones [l] + sharp;

bad:
  return MIDI_NOTE_NONE;
}
int scale2bits_aux (const char *name, int *scale, midi_note_t root) {
  for (int i = 0; i < 12; ++i) {
    if (name [i] == '0' || name [i] == '1')
      scale [(root + i) % 12] = name [i] == '1';
    else return 0;
  }
  return 1;
}
int scale2bits (const char *name, int *scale) {
  if (isdigit (name [0])) {
    return scale2bits_aux (name, scale, 0);
  } else if (isupper (name [0])) {
    char sciroot [4] = { name [0], '0', '\0', '\0' };
    if (name [1] == '#' || name [1] == 'b') sciroot [2] = name++ [1];
    midi_note_t root = note_sci2midi (sciroot);
    TRACE (TRACE_INT, "scale root %d", (int) root);
    if (root == MIDI_NOTE_NONE) return 0;
    if (0 == strcmp (&name [1], "maj"))
      return scale2bits_aux ("101011010101", scale, root % 12);
    else if (0 == strcmp (&name [1], "min"))
      return scale2bits_aux ("101101011010", scale, root % 12);
  }
  return 0;
}

sigset_t sigmask;
pthread_t poll_thread_tid = 0;
jack_nframes_t initial_sil_frames = 0,
  wavebrk_sil_frames = 0,
  note_recog_frames = 0,
  note_recog_max_frames = 0;
long srate = 0;
jack_client_t *jclient = NULL;
volatile int zombified = 0;
jack_nframes_t jmaxbufsz = 0;
jack_port_t *jport = NULL, *jmidiport = NULL;
jack_nframes_t jbuf_len = 16384;
jack_ringbuffer_t *jbuf = NULL, *jmidibuf = NULL;
sample_t *jdataptr = NULL, *jdataend = NULL;
jack_nframes_t jdatalen = 0;
int jbufavail_semid; int jbufavail_valid = 0;
sem_t jcon_sem, jmidibuf_sem;
sample_t norm_noise_peak = 0;
sample_anl_state stdmidi_anls;
sample_t jnorm_factor = 0.0;
sample_anl_state janls;
gridpt *gridpt_buf = NULL;
size_t gridpt_buf_max = 0, gridpt_buf_len = 0;
gridpt_seq *gridpt_seqdb = NULL;
size_t gridpt_seqdb_max = 0, gridpt_seqdb_len = 0;

void sample_anl_state_init (sample_anl_state *s) {
  memset (s, 0, sizeof (*s));
  s->voiced_ago = 15 * srate;
}

static void myshutdown (int failure);
#define ASSERT( cond ) TRACE_ASSERT (cond, myshutdown (1));

void analyze_sample (sample_t x, sample_anl_state * const s) {
  s->evt = 0;
  sample_t absx = fabsf (x);
  if (absx > s->max_x)
    s->max_x = absx;
  if (s->noise_peak > 0.0 && absx > s->noise_peak) {
    if (s->voiced_ago >= wavebrk_sil_frames)
      s->evt |= (1 << ANL_EVT_ZERO);
    s->voiced_ago = 0;
    if (absx > 5 * s->noise_peak && s->peak * (s->peak - x) <= 0) {
      s->peak = x;
      s->peak_at = s->count;
    }
  }
  sample_t border = (sample_t) copysign (s->noise_peak, s->peak);
  if (s->peak != 0.0 && (border - x) * (border - s->peak) <= 0) {
    s->evt |= (1 << ANL_EVT_PEAK) | (1 << ANL_EVT_ZERO);
    s->npeaks++;
    sample_t abs_peak = s->peak > 0 ? s->peak : -s->peak;
    if (s->peak > s->max_peak) {
      s->max_peak = abs_peak;
      s->max_peak_idx = s->npeaks - 1;
    }
    s->old_peak = s->peak;
    s->peak = 0.0;
  }
  ++s->voiced_ago;
  if (s->voiced_ago == wavebrk_sil_frames) s->evt |= (1 << ANL_EVT_SIL);
  ++s->count;
  s->x = x;
}

size_t gridpt_buf_dur (int i) {
  return (gridpt_buf [gridpt_buf_len - 1].length - gridpt_buf [i].length);
}
static float gridpt_buf_add (sample_t x, jack_nframes_t count) {
  gridpt_buf [gridpt_buf_len].x = x;
  gridpt_buf [gridpt_buf_len].length = count;
  ++gridpt_buf_len; ASSERT (gridpt_buf_len <= gridpt_buf_max);
  TRACE (TRACE_INT + 2, "gridpt_buf_len=%d x=%f cnt=%d dur=%d/%d",
         (int) gridpt_buf_len, (float) x, (int) count,
         (int) gridpt_buf_dur (0), (int) note_recog_frames);
  return ((float) gridpt_buf_dur (0)) / note_recog_frames;
}
static void gridpt_buf_shift () {
  memmove (gridpt_buf, gridpt_buf + 1,
           --gridpt_buf_len * sizeof (gridpt));
}
static void gridpt_seq_fix_lengths (gridpt *seq, size_t len) {
  size_t i = len;
  for (; i > 0; --i)
    seq [i - 1].length -= seq [0].length;
}
static void gridpt_seq_init (gridpt_seq *entry, gridpt *seq, size_t len) {
  ASSERT (len >= 2);
  size_t bytes = len * sizeof (gridpt);
  entry->seq = malloc (bytes); ASSERT (entry->seq != NULL);
  memcpy (entry->seq, seq, bytes);
  entry->dur = seq [len - 1].length - seq [0].length;
  gridpt_seq_fix_lengths (entry->seq, len);
  entry->length = len;
  entry->type = gridpt_seq_type (seq);
}
static void gridpt_seq_add (midi_note_t note, gridpt *seq, size_t len) {
  if (gridpt_seqdb_len == gridpt_seqdb_max) {
    gridpt_seqdb_max = 1.5 * gridpt_seqdb_max;
    gridpt_seqdb = realloc (gridpt_seqdb, gridpt_seqdb_max * sizeof (gridpt_seq));
    ASSERT (gridpt_seqdb != NULL);
  }
  gridpt_seq *entry = &gridpt_seqdb [gridpt_seqdb_len];
  gridpt_seq_init (entry, seq, len);
  entry->note = note;
  ++gridpt_seqdb_len;
}
static void gridpt_seq_add_all (midi_note_t note, jack_nframes_t minframes) {
  for (size_t i = 0; i < gridpt_buf_len && gridpt_buf_dur (i) >= minframes; ++i)
    gridpt_seq_add (note, &gridpt_buf [i], gridpt_buf_len - i);
}

typedef struct {
  midi_note_t note;
  float dist;
} gpb_anl_state;
void gpb_anl_state_init (gpb_anl_state *s) {
  s->note = MIDI_NOTE_NONE;
  s->dist = INFINITY;
}
static midi_note_t gridpt_buf_analyze (gpb_anl_state *state) {
  midi_note_t note = MIDI_NOTE_NONE;

  ASSERT (gridpt_buf_len >= 2);
  gridpt_seq gseq;
  gridpt_seq_init (&gseq, gridpt_buf, gridpt_buf_len);
  gridpt *seq = gseq.seq;
  float mindist = INFINITY, mindist_cnote = INFINITY;
  size_t minidx = 0;

  size_t beg = 0, end = gridpt_seqdb_len, mid = 0;
  int rc = 0;
  while (beg < end) {
    mid = ((unsigned) beg + (unsigned) end) >> 1;
    rc = gridpt_seq_cmp (&gseq, &gridpt_seqdb [mid]);
    if (rc > 0) beg = mid + 1;
    else if (rc < 0) end = mid;
    else break;
  }
  if (rc != 0) goto ret;
  for (beg = mid;
       beg > 0 && gridpt_seq_cmp (&gseq, &gridpt_seqdb [beg - 1]) == 0; --beg);
  for (end = mid + 1;
       end < gridpt_buf_len && gridpt_seq_cmp (&gseq, &gridpt_seqdb [end]) == 0; ++end);
  
  for (size_t i = beg; i < end; ++i) {
    if (fabs ((float) gridpt_seqdb [i].dur / gseq.dur - 1) > 0.03) continue;
    midi_note_t newnote = gridpt_seqdb [i].note;
    if (! note_scale [newnote % 12]) continue;
    float dscale = (double) SQR (gridpt_seqdb [i].dur) / (gridpt_buf_len - 1);
    sample_t dist = 0.0;
    gridpt *seq2 = gridpt_seqdb [i].seq;
    size_t j = 0;
    for (; j < gridpt_buf_len; j++) {
      sample_t x = seq [j].x;
      if (x != 0 && fabsf (x / seq2 [j].x - 1) > 0.1) goto next_seq;
      jack_nframes_t l = seq [j].length, l2 = seq2 [j].length;
      if (l != 0 && fabsf ((float) l / l2 - 1) > 0.15) goto next_seq;
      dist += SQR (l - l2) / dscale;
      if (newnote != state->note && dist > mindist) goto next_seq;
    }
    if (dist < mindist) {
      mindist = dist;
      minidx = i;
    }
    if (newnote == state->note && dist < mindist_cnote)
      mindist_cnote = dist;
  next_seq:
    if (dist > 0.0)
      TRACE (TRACE_INT + 1, "recognizing note j=%d/%d dist=%lf",
             (int) j, (int) gridpt_buf_len, dist);
  }

  if (isinf (mindist_cnote) && isfinite (state->dist))
    mindist_cnote = state->dist = state->dist * 5;
  else if (isfinite (mindist_cnote) && mindist_cnote > state->dist)
    mindist_cnote = state->dist = 0.2 * state->dist + 0.8 * mindist_cnote;
  if (isfinite (mindist)) {
    midi_note_t minnote = gridpt_seqdb [minidx].note;
    if (minnote == state->note || mindist * 1.02 <= mindist_cnote) {
      note = state->note = minnote;
      state->dist = mindist;
    }
    TRACE (TRACE_INT, "minnote %d dist %f dist_cnote %f dur=%d",
           (int) minnote, mindist, mindist_cnote, gseq.dur);
  }

  goto ret;
ret: free (seq); return note;
}

static void process_waveform_db () {
  SF_INFO sf_info; memset (&sf_info, sizeof (sf_info), 0);
  SNDFILE *dbf = sf_open (dbfname, SFM_READ, &sf_info);
  if (NULL == dbf) {
    TRACE (TRACE_FATAL, "Could not open file %s", dbfname);
    myshutdown (1);
  }
  if (1 != sf_info.channels) {
    TRACE (TRACE_FATAL, "File %s has %d channels, expecting exactly 1",
               dbfname, (int) sf_info.channels);
    myshutdown (1);
  }
  srate = sf_info.samplerate;
  initial_sil_frames = srate / 100 * initial_sil_ms / 10;
  wavebrk_sil_frames = srate / 100 * wavebrk_sil_ms / 10;
  note_recog_frames = srate / 100 * note_recog_ms / 10;
  note_recog_max_frames = srate / 100 * note_recog_max_ms / 10;
  gridpt_buf_max = note_midi2freq (himidi) * note_recog_ms / 1000 * 64;
  gridpt_buf = malloc (sizeof (gridpt) * gridpt_buf_max);
  ASSERT (gridpt_buf != NULL);
  gridpt_seqdb_max = 20000;
  gridpt_seqdb = malloc (gridpt_seqdb_max * sizeof (gridpt_seq));
  ASSERT (gridpt_seqdb != NULL);

  double x;
  sample_anl_state anls; sample_anl_state_init (&anls);

  // find noise level in waveform db
  for (sf_count_t i = 0; i < initial_sil_frames; ++i) {
    if (sf_read_double (dbf, &x, 1) < 1) goto premature;
    analyze_sample (x, &anls);
  }
  norm_noise_peak = anls.noise_peak = 4 * anls.max_x;
  TRACE (TRACE_DIAG, "Noise peak in db file: %.5f",
             (float) norm_noise_peak);

  for (midi_note_t note = lomidi; note <= himidi; ++note) {
    double note_period = 1 / note_midi2freq (note);
    
    TRACE (TRACE_INT + 1, "Reading MIDI note %d waveform, frame=%ld",
               (int) note, (long) my_sf_tell (dbf));
    // skip silence
    while (anls.voiced_ago > 1) {
      if (sf_read_double (dbf, &x, 1) < 1) goto premature;
      analyze_sample (x, &anls);
    }
    TRACE (TRACE_INT + 1, "Found note at frame=%ld",
               (long) my_sf_tell (dbf));

    sf_count_t note_pos = my_sf_tell (dbf);
    anls.count = 1;
    sample_anl_state stdanls = anls;
    gridpt_buf_len = 0;
    do {
      if ((anls.evt & ((1 << ANL_EVT_PEAK) | (1 << ANL_EVT_ZERO))) != 0 &&
          anls.count < note_recog_max_frames)
      {
        for (int part = 0; part < 2; ++part) {
#define TEST( part, cond ) case part: if (! (cond)) continue; break
          switch (part) {
            TEST (0, (anls.evt & (1 << ANL_EVT_PEAK)) != 0 &&
                  gridpt_buf_add (anls.old_peak, anls.peak_at) > 0.95);
            TEST (1, (anls.evt & (1 << ANL_EVT_ZERO)) != 0 &&
                  gridpt_buf_add (0, anls.count) > 0.95);
          }
#undef TEST
          while (gridpt_buf_dur (0) > (note_recog_frames + note_period * srate / 4) * 1.05)
            gridpt_buf_shift ();
          gridpt_seq_add_all (note, 0.95 * note_recog_frames);
        }
      }
      if (sf_read_double (dbf, &x, 1) < 1) goto premature;
      analyze_sample (x, &anls);
    } while ((anls.evt & (1 << ANL_EVT_SIL)) == 0);

    TRACE (TRACE_INT + 1, "MIDI note %d Waveform ends at frame=%ld, seqs=%ld",
           (int) note, (long) my_sf_tell (dbf), (long) gridpt_seqdb_len);
    if (note == stdmidi) {
      sf_count_t saved_pos = my_sf_tell (dbf);
      sf_seek (dbf, note_pos, SEEK_SET);
      stdanls.npeaks = 0; stdanls.max_peak = 0; stdanls.max_peak_idx = 0;
      TRACE (TRACE_INT + 1, "Back to frame=%ld for stdmidi",
                 (long) my_sf_tell (dbf));
      // gather calibration info
      while ((stdanls.evt & (1 << ANL_EVT_SIL)) == 0 &&
             stdanls.npeaks < stdmidi_npeaks)
      {
        if (sf_read_double (dbf, &x, 1) < 1) goto premature;
        analyze_sample (x, &stdanls);
        if ((stdanls.evt & (1 << ANL_EVT_PEAK)) != 0)
          TRACE (TRACE_INT + 2, "Peak at frame %ld %f",
                 (long) my_sf_tell (dbf), (float) stdanls.old_peak);
      }
      stdmidi_anls = stdanls;
      sf_seek (dbf, saved_pos, SEEK_SET);
      TRACE (TRACE_INT + 1, "Returned to frame=%ld",
                 (long) my_sf_tell (dbf));
    }
  }

  qsort (gridpt_seqdb, gridpt_seqdb_len, sizeof (gridpt_seqdb [0]), gridpt_seq_cmp);
  
  TRACE (TRACE_INFO, "%d sequences loaded from waveform db", gridpt_seqdb_len);
  return;

premature:
  TRACE (TRACE_FATAL, "Premature end of waveform db file, frame=%ld",
             (long) my_sf_tell (dbf));
  myshutdown (1);
}

sample_t get_jsample () {
  int rc;

  if (jdataptr == jdataend) {
    jack_ringbuffer_read_advance (jbuf, jdatalen * sizeof (sample_t));
    jdataptr = jdataend = NULL;
    struct sembuf sops = { .sem_num = 0, .sem_op = -1, .sem_flg = 0 };
    while ((rc = semop (jbufavail_semid, &sops, 1)) == -1)
      if (rc == -1 && errno != EINTR) {
        TRACE_PERROR (TRACE_FATAL, "semop");
        myshutdown (1);
      }
    if (zombified) {
      myshutdown (1);
    }
    jack_ringbuffer_data_t jdatainfo [2];
    jack_ringbuffer_get_read_vector (jbuf, jdatainfo);
    jdatalen = jdatainfo [0].len / sizeof (sample_t);
    ASSERT (jdatalen > 0);
    if (-1 == (rc = semctl (jbufavail_semid, 0, GETVAL))) {
      TRACE_PERROR (TRACE_FATAL, "semctl");
      myshutdown (1);
    }
    if (jdatalen > (jack_nframes_t) ++rc) jdatalen = rc;

    sops.sem_op = -(jdatalen - 1);
    semop (jbufavail_semid, &sops, 1);
    jdataptr = (sample_t *) jdatainfo [0].buf;
    jdataend = jdataptr + jdatalen;
  }
  return *jdataptr++;
}

void pack_midi_evt (jack_midi_data_t *buf, midi_note_t note, int velo, int on) {
  buf [0] = on ? 0x90 : 0x80;
  buf [1] = note;
  buf [2] = on ? velo : 0;
  TRACE (TRACE_INT, "note=%d on=%d", (int) note, on);
}

static void *process_thread (void *arg) {
  (void) arg;

  int connected = 0;
  int rc;
  jack_nframes_t lost_frames = 0;

  for (;;) {

    if (! connected && 0 == sem_trywait (&jcon_sem))
      connected = 1;

    jack_nframes_t nframes_orig = jack_cycle_wait (jclient),
      nframes = nframes_orig;
    if (zombified) break;

    jack_midi_data_t evt [24];
    size_t evtcnt = 0;
    for (; evtcnt < sizeof (evt) / sizeof (evt [0]) / 3 
           && 0 == sem_trywait (&jmidibuf_sem);
         ++evtcnt)
    {
      char ch;
      jack_ringbuffer_read (jmidibuf, &ch, 1);
      pack_midi_evt (&evt [evtcnt * 3], 
                     ch & ((1 << 7) - 1), velo0, (ch & (1 << 7)) != 0);
    }
    void *out = jack_port_get_buffer (jmidiport, nframes);
    jack_midi_clear_buffer (out);
    if (evtcnt > 0) {
      for (size_t i = 0; i < evtcnt; ++i)
        jack_midi_event_write (out, 0, evt + 3 * i, sizeof (evt [0]) * 3);
    }

    sample_t *in = jack_port_get_buffer (jport, nframes);    
    if (-1 == (rc = semctl (jbufavail_semid, 0, GETVAL))) {
      TRACE_PERROR (TRACE_FATAL, "semctl");
      break;
    }
    jack_nframes_t avail = jbuf_len - rc - 2;
    if (avail < 2 * nframes) nframes = (avail < 2) ? 0 : 2;
    if (connected)
      jack_ringbuffer_write (jbuf, (const char *) in, nframes * sizeof (sample_t));
    jack_cycle_signal (jclient, 0);

    if (connected && nframes > 0) {
      struct sembuf sops = { .sem_num = 0, .sem_op = nframes, .sem_flg = 0 };
      semop (jbufavail_semid, &sops, 1);
    }
    if (! lost_frames && nframes != nframes_orig) {
      lost_frames = 1;
      TRACE (TRACE_WARN, "Lost frames %d", (int) (nframes_orig - nframes));
    }
  }

  return NULL;
}

static void calibrate () {
  sample_t noise_peak;
  sample_anl_state_init (&janls);

  // find noise level
  TRACE (TRACE_DIAG, "Analysing noise...");
  for (jack_nframes_t i = 0; i < initial_sil_frames; ++i)
    analyze_sample (get_jsample (), &janls);
  noise_peak = janls.noise_peak = 4 * janls.max_x;
  TRACE (TRACE_DIAG, "Noise peak in input: %.5f", (float) noise_peak);

  // skip silence
  char stdscinote [4]; note_midi2sci (stdmidi, stdscinote);
  TRACE (TRACE_IMPT, "Calibrating against note %s waveform: press note now", stdscinote);
  while (analyze_sample (get_jsample (), &janls), janls.voiced_ago > 1);

  TRACE (TRACE_INT, "Found note");
  ASSERT (janls.npeaks == 0);
  while (analyze_sample (get_jsample (), &janls),
         ((janls.evt & (1 << ANL_EVT_SIL)) == 0 &&
          janls.npeaks < stdmidi_npeaks))
  {
    if ((janls.evt & (1 << ANL_EVT_PEAK)) != 0)
      TRACE (TRACE_INT + 1, "Peak at frame %d %f",
             (int) janls.count, (float) janls.old_peak);
  }
  if (janls.max_peak_idx != stdmidi_anls.max_peak_idx) {
    TRACE (TRACE_FATAL, "calibration mismatch npeaks=%d %d %d %f %f",
               janls.npeaks,
               janls.max_peak_idx, stdmidi_anls.max_peak_idx,
               janls.max_peak, stdmidi_anls.max_peak);
    myshutdown (1);
  }
  jnorm_factor = stdmidi_anls.max_peak / janls.max_peak;
  TRACE (TRACE_INFO, "Scaling factor against waveform db: %f", (float) jnorm_factor);

  // skip rest of note
  while (analyze_sample (get_jsample (), &janls),
         (janls.evt & (1 << ANL_EVT_SIL)) == 0);
}

void send_note (midi_note_t note, int on) {
  char ch = note + transpose_semitones; if (on) ch |= 1 << 7;
  jack_ringbuffer_write (jmidibuf, &ch, 1);
}

static void midi_server () {
  midi_note_t cnote = 0;
  gpb_anl_state gpbs;
  gpb_anl_state_init (&gpbs);
  TRACE (TRACE_IMPT, "MIDI server started, delay %lf", note_recog_ms);
  gridpt_buf_len = 0;
  for (;;) {
    analyze_sample (get_jsample () * jnorm_factor, &janls);
    if ((janls.evt & (1 << ANL_EVT_SIL)) != 0 && cnote != 0) {
      send_note (cnote, 0); sem_post (&jmidibuf_sem);
      cnote = 0;
      gpb_anl_state_init (&gpbs);
      TRACE (TRACE_INFO, "Note off");
      gridpt_buf_len = 0;
    }
    if ((janls.evt & ((1 << ANL_EVT_PEAK) | (1 << ANL_EVT_ZERO))) != 0)
    {
      for (int part = 0; part < 2; part++) {
#define TEST( part, cond ) case part: if (! (cond)) continue; break
        switch (part) {
          TEST (0, (janls.evt & (1 << ANL_EVT_PEAK)) != 0 &&
                gridpt_buf_add (janls.old_peak, janls.peak_at) > 1);
          TEST (1, (janls.evt & (1 << ANL_EVT_ZERO)) != 0 &&
                gridpt_buf_add (0, janls.count) > 1);
        }
#undef TEST
        // when a point is added AND completes a sequence
        while (gridpt_buf_dur (1) > note_recog_frames)
          gridpt_buf_shift ();
        midi_note_t note = gridpt_buf_analyze (&gpbs);
        if (note != MIDI_NOTE_NONE && note != cnote) {
          int cut_last = cnote != 0;
          send_note (note, 1);
          if (cut_last)
            send_note (cnote, 0);
          sem_post (&jmidibuf_sem); if (cut_last) sem_post (&jmidibuf_sem); 

          cnote = note;          
          char scinote [4]; note_midi2sci (note, scinote);
          TRACE (TRACE_INFO, "Note %s", scinote);
        }
      }
    }
  }
}

static void on_jack_shutdown (void *arg) {
  (void) arg;

  zombified = 1;
  TRACE (TRACE_FATAL, "Jack shut us down");
}

static void setup_audio () {
  TRACE (TRACE_INT, "jack setup");
  trace_flush (); fflush(NULL);

  if (-1 == (jbufavail_semid = semget (IPC_PRIVATE, 1, IPC_CREAT | S_IRUSR | S_IWUSR))) {
    TRACE_PERROR (TRACE_FATAL, "semget");
    myshutdown (1);
  }
  jbufavail_valid = 1;
  semctl_arg_t semarg = { .val = 0 };
  if (-1 == semctl (jbufavail_semid, 0, SETVAL, semarg)) {
    TRACE_PERROR (TRACE_FATAL, "semctl");
    myshutdown (1);
  }
  sem_init (&jmidibuf_sem, 0, 0);
  sem_init (&jcon_sem, 0, 0);
  jack_options_t jopt = 0;  // JackNoStartServer;
  jack_status_t jstat;
  if (NULL == (jclient = jack_client_open (name, jopt, &jstat)))
    goto jack_setup_fail;
  jack_on_shutdown (jclient, on_jack_shutdown, NULL);
  int jsrate = jack_get_sample_rate (jclient);
  if (jsrate != srate) {
    TRACE (TRACE_FATAL, "Sample rate %d does not match waveform db's (%d)",
               jsrate, srate);
    goto jack_setup_fail;
  }

  jmaxbufsz = jack_get_buffer_size (jclient);
  TRACE (TRACE_INFO, "Connected to jack, bufsize=%d, srate=%d",
             (int) jmaxbufsz, (int) srate);
  jbuf = jack_ringbuffer_create (jbuf_len); ASSERT (jbuf != NULL);
  jmidibuf = jack_ringbuffer_create (128); ASSERT (jmidibuf != NULL);

  if (0 != jack_set_process_thread (jclient, process_thread, NULL))
    goto jack_setup_fail;

  if (NULL == (jport = jack_port_register (jclient, "in", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0)))
    goto jack_setup_fail;
  
  if (NULL == (jmidiport = jack_port_register (jclient, "midiout", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0)))
    goto jack_setup_fail;
  if (0 != jack_activate (jclient))
    goto jack_setup_fail;

  if (0 != jack_connect (jclient, srcport, jack_port_name (jport)))
    goto jack_setup_fail;
  sem_post (&jcon_sem);
  TRACE (TRACE_DIAG, "Activated");

  return;

jack_setup_fail:
  TRACE (TRACE_FATAL, "Jack setup failed");
  myshutdown (1);
}

void usage (const char *fmt, ...) {
#define NL "\n"
  if (fmt != NULL) {
    printf ("Error: ");
    va_list ap; va_start (ap, fmt);
    vprintf (fmt, ap);
    va_end (ap);
    printf ("\n\n");
  }
  printf ("%s\n",
   "Usage: " MYNAME " -r RANGE -f FILE -p JACKPORT -d DELAY [OPTIONS]"
NL "  or:  " MYNAME " { -h | --help | -? | --version }"
NL
NL "Options:"
NL "  -p, --port JP      Jack audio port to listen on (e.g. system:capture_1)"
NL "  -f, --db FILE      Training recording from synth"
NL "  -r, --range RANGE  Range of notes in training file (e.g. F2-C5)"
NL "  --transpose SEMIT  Semitones to transpose (e.g. -12)"
NL "  --scale S          Scale restriction (e.g. C#min, EbMaj)"
NL "  --log-level N      Log level (higher = more details, defaults to 4)"
          );
#undef NL
  
  trace_level = TRACE_NONE;
  exit (fmt == NULL ? EXIT_SUCCESS : EXIT_FAILURE);
}

static void parse_args (char **argv) {
  if (argv [0] == NULL) usage (NULL);
  for (++argv; *argv != NULL; ++argv) {
    if (*argv [0] == '-') {
      if (0 == strcmp (*argv, "-h") || 0 == strcmp (*argv, "--help") ||
          0 == strcmp (*argv, "-?"))
      {
        usage (NULL);
      } else if (0 == strcmp (*argv, "--version")) {
        printf ("%s version %s\n%s\n", MYNAME, "1.0", "Copyright (C) 2010-2011 Dan Muresan");
        exit (EXIT_SUCCESS);
      } else if (0 == strcmp (*argv, "-f") || 0 == strcmp (*argv, "--db")) {
        dbfname = *++argv;
      } else if (0 == strcmp (*argv, "-r") || 0 == strcmp (*argv, "--range")) {
        char *ptrhi = strchr (argv [1], '-');
        if (ptrhi == NULL) usage ("Bad note range %s", argv [1]);
        lomidi = note_sci2midi (argv [1]);
        if (lomidi == MIDI_NOTE_NONE) usage ("Bad note %s", argv [1]);
        *ptrhi++ = '\0'; himidi = note_sci2midi (ptrhi);
        if (himidi == MIDI_NOTE_NONE) usage ("Bad note %s", ptrhi);
        ++argv;
      } else if (0 == strcmp (*argv, "-p") || 0 == strcmp (*argv, "--port")) {
        srcport = *++argv;
      } else if (0 == strcmp (*argv, "-d") || 0 == strcmp (*argv, "--delay")) {
        if (sscanf (*++argv, "%lf", &note_recog_ms) != 1)
          usage ("Bad delay %s", *argv);
      } else if (0 == strcmp (*argv, "--anl-max")) {
        sscanf (*++argv, "%lf", &note_recog_max_ms);
      } else if (0 == strcmp (*argv, "--transpose")) {
        int sems;
        if (sscanf (*++argv, "%d", &sems) != 1)
          usage ("Bad semitone count %s", *argv);
        transpose_semitones = sems;
      } else if (0 == strcmp (*argv, "--scale")) {
        if (0 == scale2bits (*++argv, note_scale))
          usage ("unknown scale %s", *argv);
      } else if (0 == strcmp (*argv, "--log-level")) {
        int l;
        if (sscanf (*++argv, "%d", &l) != 1)
          usage ("Bad log level %s", *argv);
        TRACE (TRACE_IMPT, "Log level %d", l);
        trace_level = l;
      } else if (0 == strcmp (*argv, "--log-tid")) {
        trace_print_tid = 1;
      } else if (0 == strcmp (*argv, "--log-fun")) {
        trace_print_fn = 1;
      }
    }
    else break;
  }

  if (NULL == dbfname) {
    TRACE (TRACE_FATAL, "waveform database file (--db) not specified");
    myshutdown (1);
  }
  if (0 == himidi) {
    TRACE (TRACE_FATAL, "note range (e.g. --range F2-C5) not specified");
    myshutdown (1);
  }
  if (lomidi > stdmidi || stdmidi > himidi) {
    TRACE (TRACE_FATAL, "calibration MIDI note %d out of range", (int) stdmidi);
    myshutdown (1);
  }
  if (NULL == srcport) {
    TRACE (TRACE_FATAL, "no source port (--port) specified");
    myshutdown (1);
  }
}

static void *poll_thread (void *arg) {
  (void) arg;

  for (;;) {
    struct timespec sleepreq = { tv_sec: 0, tv_nsec: 200000000L };
    trace_flush ();
    nanosleep (&sleepreq, NULL);
  }

  return NULL;
}

static void init_trace () {
  trace_buf = memfile_alloc (1 << 19);
  stdtrace = fdopen (dup (fileno (stdout)), "w");
  setvbuf (stdtrace, NULL, _IONBF, 0);
  pthread_create (&poll_thread_tid, NULL, poll_thread, NULL);
  TRACE (TRACE_INFO, "Logging enabled, level=%d", (int) trace_level);
}

static void cleanup () {
  pthread_cancel (poll_thread_tid);
  pthread_join (poll_thread_tid, NULL);
  if (jclient != NULL) {
    jack_on_shutdown (jclient, NULL, NULL);
    jack_client_close (jclient);
    jclient = NULL;
  }
  if (jbufavail_valid) {
    jbufavail_valid = 0;
    semctl (jbufavail_semid, 0, IPC_RMID);
  }
  TRACE (TRACE_INFO, "Cleanup finished");
  trace_flush ();
}

static void myshutdown (int failure) {
  TRACE (TRACE_INFO, "shutdown requested");
  // print_backtrace ();
  cleanup ();
  exit (failure ? EXIT_FAILURE : EXIT_SUCCESS);
}

static void sig_handler (int sig) {
  TRACE (TRACE_INFO, "Caught signal %d", sig);
  cleanup ();
  struct sigaction act =
    { .sa_mask = sigmask, .sa_flags = 0, .sa_handler = SIG_DFL };
  sigaction (sig, &act, NULL);
  pthread_sigmask (SIG_UNBLOCK, &sigmask, NULL);
  pthread_kill (pthread_self (), sig);
}

void setup_sigs () {
  int sigarr [] = { SIGTERM, SIGQUIT, SIGABRT, SIGPIPE, SIGILL, SIGBUS, SIGFPE, SIGINT };
  sigemptyset (&sigmask);
  for (unsigned i = 0; i < sizeof (sigarr) / sizeof (sigarr [0]); ++i)
    sigaddset (&sigmask, sigarr [i]);
  pthread_sigmask (SIG_BLOCK, &sigmask, NULL);
  struct sigaction act =
    { .sa_mask = sigmask, .sa_flags = 0, .sa_handler = sig_handler };
  for (unsigned i = 0; i < sizeof (sigarr) / sizeof (sigarr [0]); ++i)
    sigaction (sigarr [i], &act, NULL);

}

int main (int argc, char **argv) {
  (void) argc;

  init_trace ();

  setup_sigs ();

  parse_args (argv);

  process_waveform_db ();

  setup_audio ();

  pthread_sigmask (SIG_UNBLOCK, &sigmask, NULL);

  calibrate ();

  midi_server ();

  myshutdown (0); return EXIT_SUCCESS;
}

/*
  
  XSI semaphores + threads aren't portable, and POSIX semaphores only increment
  by 1.

  Mutexes are subject to (possibly unbounded) priority inversion.

  Pipes have an unpredictable buffer size and require syscalls.

  pthread_mutex_trylock + keep an unannounced_data_count (per thread): mutex
  may happen to be taken when queried until reader exhausts data. We could
  use a thread-local ringbuffer to store unannounced data.

  Unsynchronized "single-writer" paradigm issues: atomicity of pointers, cache
  coherency, compiler / pipeline write reordering.

*/

/*
  Compile with:
  
  cc -std=c99 -D_REENTRANT -Wall -Wextra $CFLAGS -ljack -lsndfile -lm synthinvert.c -o synthinvert
*/
