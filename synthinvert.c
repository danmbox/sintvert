#define _XOPEN_SOURCE 600

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
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


// type aliases & constants
#define MYNAME "sintvert"
typedef int midi_note_t;  ///< A MIDI note (0-127, 21=A0, 60=C4)
#define MIDI_NOTE_NONE ((midi_note_t) -1)
double SEMITONE_RATIO = 1.06, INV_SEMITONE_RATIO = 0.94,
  QTTONE_RATIO = 1.03, QTTONE_RATIO_INV = 0.97;
typedef jack_default_audio_sample_t sample_t;
typedef union {  // POSIX requires *us* to declare it instead of <sys/sem.h>
  int val; struct semid_ds *buf; unsigned short *array;
} semctl_arg_t;

// configurable params
/// Allowed notes
int note_scale [12] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
/// Semitones to shift all output MIDI notes.
midi_note_t transpose_semitones = 0;
midi_note_t lomidi = 0, himidi = 0;  ///< MIDI note range of our synth
midi_note_t stdmidi = 60;  ///< MIDI note to calibrate against
int stdmidi_npeaks = 50;  ///< # of peaks to read for calibration
int velo_on = 100,  ///< MIDI velocity of note-on
  velo_off = 0;     ///< MIDI velocity of note-off
int midi_chan = 0;  ///< MIDI channel to send notes on
int initial_sil_ms = 450;     ///< duration of segment for noise estimation
double wavebrk_sil_ms = 1.5;  ///< msecs triggering silence detection
double recog_ms = 12;         ///< min duration of waveform to be recognized
double train_max_ms = 250;    ///< how much to analyze from training waveforms
/// File containing synth note sequence recording
const char *dbfname = NULL;
const char *srcport = NULL, *dstport = NULL;

// --- UTILS ---

#define SQR( x ) ((x) * (x))

// Since sndfile doesn't have it...
static sf_count_t my_sf_tell (SNDFILE *sndfile)
{ return sf_seek (sndfile, 0, SEEK_CUR); }

static void mutex_cleanup_routine (void *lock_) {
  pthread_mutex_unlock ((pthread_mutex_t *) lock_);
}
/// Locks a mutex and pushes a @c pthread_cleanup routine.
/// Must be matched with a <code>pthread_cleanup_pop (1)</code>
#define MUTEX_LOCK_WITH_CLEANUP( lock )                 \
  pthread_mutex_lock (lock);                            \
  pthread_cleanup_push (mutex_cleanup_routine, (lock))

static void pack_midi_evt (jack_midi_data_t *buf, midi_note_t note, int on) {
  buf [0] = (on ? 0x90 : 0x80) | midi_chan;
  buf [1] = note; buf [2] = on ? velo_on : velo_off;
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
static int scale2bits_aux (const char *name, int *scale, midi_note_t root) {
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
    if (root == MIDI_NOTE_NONE) return 0;
    if (0 == strcmp (&name [1], "maj"))
      return scale2bits_aux ("101011010101", scale, root % 12);
    else if (0 == strcmp (&name [1], "min"))
      return scale2bits_aux ("101101011010", scale, root % 12);
  }
  return 0;
}

// --- END UTILS ---


// --- CLASSES ---

/// An in-memory file.
/// Its methods contain no cancellation points (except asserts).
typedef struct {
  char *buf1, *buf2, *ptr, *end, *other;
  size_t sz;
  pthread_mutex_t lock, switchlock;
} memfile;
void memfile_init (memfile *f, size_t sz, char *buf1) {
  f->sz = sz;
  if (buf1 == NULL) {
    f->buf1 = calloc (sz, 1); f->buf2 = calloc (sz, 1);
    assert (f->buf1 != NULL && f->buf2 != NULL);
  } else {
    f->buf1 = buf1; f->buf2 = NULL;
  }
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
  memfile_init (f, sz, NULL);
  return f;
}
/// Switches the active and shadow buffers of @p f.
/// Locks @p f's switchlock, then its lock.
void memfile_switchbuf (memfile *f) {
  MUTEX_LOCK_WITH_CLEANUP (&f->switchlock);
  MUTEX_LOCK_WITH_CLEANUP (&f->lock);
  assert (f->other != NULL);
  f->other [0] = '\0';
  f->ptr = f->other; f->end = f->other + f->sz;
  f->other = f->other == f->buf1 ? f->buf2 : f->buf1;
  pthread_cleanup_pop (1);
  pthread_cleanup_pop (1);
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
const char *trace_level_symb = "-FEW!ID                                   ";
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
  MUTEX_LOCK_WITH_CLEANUP (&trace_buf->lock); {
    memfile_printf (trace_buf, "%c %s ", trace_level_symb [pri], timestr);
    va_list ap; va_start (ap, fmt);
    memfile_vprintf (trace_buf, fmt, ap);
    if (trace_print_fn) memfile_printf (trace_buf, " [%s]", fn);
    if (trace_print_tid)
      memfile_printf (trace_buf, " [tid=0x%X]", (unsigned) pthread_self ());
    memfile_printf (trace_buf, "\n");
    va_end (ap);
  } pthread_cleanup_pop (1);
}
void trace_flush () {
  MUTEX_LOCK_WITH_CLEANUP (&trace_buf->switchlock); {
    memfile_switchbuf (trace_buf);
    fprintf (stdtrace, "%s", trace_buf->other);  // cancellation point
  } pthread_cleanup_pop (1);
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

#if 0  // enable as needed when debugging
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

typedef struct {
  int npeaks,  ///< Total number of peaks
    max_peak_idx;  ///< Index of @c max_peak (among all peaks)
  sample_t x,  ///< Latest sample analyzed
    peak,  ///< Current peak candidate
    old_peak,  ///< Latest confirmed peak
    max_x,  ///< Largest absolute value ever
    max_peak,  ///< Largest peak
    noise_peak;  ///< Estimated noise level
  jack_nframes_t count,  ///< Frame counter
    voiced_ago,  ///< Frames since last sample above @c noise_peak
    peak_at;  ///< Frame count of @c old_peak
  int evt;  ///< Event flags OR-ed together, see @c sample_anl_event_bit
} sample_anl_state;
typedef enum {
  ANL_EVT_PEAK = 0, ANL_EVT_ZERO, ANL_EVT_SIL
} sample_anl_event_bit;
void sample_anl_state_init (sample_anl_state *s) {
  memset (s, 0, sizeof (*s));
  s->voiced_ago = JACK_MAX_FRAMES;
}

/// A feature point in the waveform (peak or zero crossing)
typedef struct {
  sample_t x;  ///< Amplitude
  jack_nframes_t framecnt;  ///< Frame count
} gridpt;
/// Sequence type from @c gpt_seq_type
typedef enum {
  GPT_SEQ_POSITIVE, GPT_SEQ_NEGATIVE, GPT_SEQ_FALLING0, GPT_SEQ_RISING0
} gpt_seq_type_t;
typedef struct {
  gridpt *seq;
  size_t length;  ///< Number of points
  jack_nframes_t dur;  ///< Frame duration of this sequence
  int type;
  midi_note_t note;
} gpt_seq;
/// Determines sequence type based on initial 2 points
gpt_seq_type_t gpt_seq_type (gridpt *seq) {
  if (seq [0].x > 0) return GPT_SEQ_POSITIVE;
  else if (seq [0].x < 0) return GPT_SEQ_NEGATIVE;
  else if (seq [1].x < 0) return GPT_SEQ_FALLING0;
  else return GPT_SEQ_RISING0;
}
static char *gpt_seq_dump (const gpt_seq *gseq, char *buf, size_t sz) {
  static char buf_ [256]; if (buf == NULL) { buf = buf_; sz = sizeof (buf_); }
  memfile mf; memfile_init (&mf, sz, buf);
  memfile_printf (&mf, "l=%d t=%d dur=%d note=%d",
                  gseq->length, gseq->type, (int) gseq->dur, (int) gseq->note);
  for (size_t i = 0; i < gseq->length; i++)
    memfile_printf (&mf, " (%f %d)", gseq->seq [i].x, gseq->seq [i].framecnt);
  return buf;
}
/// Rough comparison function based on @c gpt_seq's @c length and @c type
int gpt_seq_cmp1 (const gpt_seq *s1, const gpt_seq *s2) {
  if      (s1->length < s2->length) return -1;
  else if (s1->length > s2->length) return 1;
  else if (s1->type < s2->type) return -1;
  else if (s1->type > s2->type) return 1;
  else return 0;
}
/// Refined comparison of @c gpt_seq's including duration
int gpt_seq_cmp (const gpt_seq *s1, const gpt_seq *s2) {
  int rc1 = gpt_seq_cmp1 (s1, s2);
  if      (rc1 != 0) return rc1;
  else if (s1->dur < s2->dur) return -1;
  else if (s1->dur > s2->dur) return 1;
  else return 0;
}

// --- END CLASSES ---


// --- PROGRAM ---

// ASSERT to be used from main thread and not for general-purpose routines.
// C assert also works (though not as nicely).
#define ASSERT( cond ) TRACE_ASSERT (cond, myshutdown (1));

static void myshutdown (int failure);

// internal vars
sigset_t sigmask;
pthread_t poll_thread_tid = 0;
jack_nframes_t initial_sil_frames = 0,
  wavebrk_sil_frames = 0,
  recog_frames = 0,
  train_max_frames = 0;
long srate = 0;  ///< Sampling rate
jack_client_t *jclient = NULL;
volatile int zombified = 0;
jack_port_t *jport = NULL, *jmidiport = NULL;
jack_ringbuffer_t *jbuf = NULL,  ///< Incoming audio buffer
  *jmidibuf = NULL;  ///< Outgoing MIDI buffer
/// Size of @c jbuf.
/// Limited by jbufavail semaphore max.
jack_nframes_t jbuf_len = 16384;
int jbufavail_semid;  ///< Counting semaphore for @c jbuf
int jbufavail_valid = 0;  ///< Is @c jbufavail_semid active?
sem_t jmidibuf_sem;  ///< Counting semaphore for @c jmidibuf
sample_t norm_noise_peak = 0;  ///< Estimated noise amplitude in training file
sample_anl_state stdmidi_anls;  ///< State for calibration note analyzer
sample_t jnorm_factor = 0.0;  ///< Multiplier against training file loudness
sample_anl_state janls;  ///< Analyzer state for jack samples
gridpt *gpt_buf = NULL;  ///< Buffer of current feature points
size_t gpt_buf_max = 0,  ///< Maximum for @c gpt_buf_len
  gpt_buf_len = 0;  ///< Current size of @c gpt_buf
gpt_seq *gpt_seqdb = NULL;  ///< Database of gpt sequences from training
size_t gpt_seqdb_max = 20000 /* grows */,
  gpt_seqdb_len = 0;  ///< Current size of @c gpt_seqdb

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

static jack_nframes_t gpt_buf_dur (int i) {
  return (gpt_buf [gpt_buf_len - 1].framecnt - gpt_buf [i].framecnt);
}
static float gpt_buf_add (sample_t x, jack_nframes_t count) {
  ASSERT (gpt_buf_len < gpt_buf_max);
  gpt_buf [gpt_buf_len].x = x;
  gpt_buf [gpt_buf_len].framecnt = count;
  ++gpt_buf_len;
  return ((float) gpt_buf_dur (0)) / recog_frames;
}
static void gpt_buf_shift () {
  memmove (gpt_buf, gpt_buf + 1,
           --gpt_buf_len * sizeof (gridpt));
}
static void gpt_seq_fix_lengths (gridpt *seq, size_t len) {
  size_t i = len;
  for (; i > 0; --i)
    seq [i - 1].framecnt -= seq [0].framecnt;
}
static void gpt_seq_init (gpt_seq *entry, gridpt *seq, size_t len) {
  ASSERT (len >= 2);
  size_t bytes = len * sizeof (gridpt);
  entry->seq = malloc (bytes); ASSERT (entry->seq != NULL);
  memcpy (entry->seq, seq, bytes);
  entry->dur = seq [len - 1].framecnt - seq [0].framecnt;
  gpt_seq_fix_lengths (entry->seq, len);
  entry->length = len;
  entry->type = gpt_seq_type (seq);
  entry->note = MIDI_NOTE_NONE;
}
static void gpt_seq_add (midi_note_t note, gridpt *seq, size_t len) {
  if (gpt_seqdb_len == gpt_seqdb_max) {
    gpt_seqdb_max = 1.5 * gpt_seqdb_max;
    gpt_seqdb = realloc (gpt_seqdb, gpt_seqdb_max * sizeof (gpt_seq));
    ASSERT (gpt_seqdb != NULL);
  }
  gpt_seq *entry = &gpt_seqdb [gpt_seqdb_len];
  gpt_seq_init (entry, seq, len);
  entry->note = note;

  ++gpt_seqdb_len;
}
static void gpt_seq_add_all (midi_note_t note, jack_nframes_t minframes) {
  for (size_t i = 0; i < gpt_buf_len && gpt_buf_dur (i) >= minframes; ++i)
    gpt_seq_add (note, &gpt_buf [i], gpt_buf_len - i);
}

typedef struct {
  midi_note_t note;
  float dist;
} gpt_seq_anl_state;
static void gpt_seq_anl_state_init (gpt_seq_anl_state *s) {
  s->note = MIDI_NOTE_NONE;
  s->dist = INFINITY;
}
static int gpt_seq_search (const gpt_seq *gseq,
                           size_t beg, size_t end, size_t *mid)
{
  int rc = INT_MAX;
  assert (beg < end);
  while (beg < end) {
    *mid = (((unsigned) beg) + (unsigned) (end - 1)) >> 1;
    rc = gpt_seq_cmp (gseq, &gpt_seqdb [*mid]);
    if (rc > 0) beg = *mid + 1;
    else if (rc < 0) end = *mid;
    else break;
  }
  return rc;
}
static midi_note_t gpt_seq_analyze (gpt_seq_anl_state *state) {
  ASSERT (gpt_buf_len >= 2);

  midi_note_t note = MIDI_NOTE_NONE, minnote = MIDI_NOTE_NONE;;
  gpt_seq gseq; gpt_seq_init (&gseq, gpt_buf, gpt_buf_len);
  gridpt *seq = gseq.seq;
  float mindist = INFINITY, mindist_cnote = INFINITY;
  size_t minidx = 0, minidx_cnote = 0;

  gpt_seq gseq_lo = gseq; gseq_lo.dur *= INV_SEMITONE_RATIO;
  gpt_seq gseq_hi = gseq; gseq_hi.dur *= SEMITONE_RATIO;
  if (gpt_seq_cmp (&gseq_hi, &gpt_seqdb [gpt_seqdb_len - 1]) > 0)
    gseq_hi = gpt_seqdb [gpt_seqdb_len - 1];
  size_t lo = 0, hi = gpt_seqdb_len - 1, i = 0;
  int rc = gpt_seq_search (&gseq_lo, lo, hi + 1, &lo);
  if (rc != 0) {
    int rc1 = gpt_seq_cmp1 (&gpt_seqdb [lo], &gseq_lo);
    if (rc1 < 0) {
      if (gpt_seq_cmp1 (&gpt_seqdb [++lo], &gseq_lo) != 0) goto ret;
    }
    else if (rc1 > 0) goto ret;
  }

  for (i = lo; gpt_seq_cmp (&gpt_seqdb [i], &gseq_hi) <= 0; ++i) {
    midi_note_t newnote = gpt_seqdb [i].note;
    if (! note_scale [newnote % 12]) continue;
    gridpt *seq2 = gpt_seqdb [i].seq;
    float dscale = (double) SQR (gpt_seqdb [i].dur) / (gpt_buf_len - 1);
    sample_t dist = 0.0;
    size_t j = 0;
    for (; j < gpt_buf_len; j++) {
      sample_t x = seq [j].x;
      if (x != 0 && fabsf (x / seq2 [j].x - 1) > 0.1) goto next_seq;
      jack_nframes_t l = seq [j].framecnt, l2 = seq2 [j].framecnt;
      if (l != 0 && fabsf ((float) l / l2 - 1) > 0.15) goto next_seq;
      dist += SQR (l - l2) / dscale;
      if (newnote != state->note && dist > mindist) goto next_seq;
    }
    if (dist < mindist) {
      mindist = dist;
      minidx = i;
    }
    if (newnote == state->note && dist < mindist_cnote) {
      mindist_cnote = dist;
      minidx_cnote = i;
    }
  next_seq:
    if (dist > 0.0)
      TRACE (TRACE_INT + 2, "tried note %d j=%d/%d d=%lf",
             (int) gpt_seqdb [i].note, j, gpt_buf_len, dist);
  }

  if (isinf (mindist_cnote) && isfinite (state->dist))
    mindist_cnote = state->dist = state->dist * 5;
  else if (isfinite (mindist_cnote) && mindist_cnote > state->dist)
    mindist_cnote = state->dist = 0.2 * state->dist + 0.8 * mindist_cnote;
  if (isfinite (mindist)) {
    minnote = gpt_seqdb [minidx].note;
    if (minnote == state->note ||
        (mindist < SQR (SEMITONE_RATIO - 1) && mindist * QTTONE_RATIO <= mindist_cnote))
    {
      note = state->note = minnote;
    }
  }
  state->dist = mindist_cnote;

  goto ret;

ret:
  gseq.note = note;
  char buf [2048]; gpt_seq_dump (&gseq, buf, sizeof (buf));
  TRACE (TRACE_INT + 1, "min %d-%d: d=%f d_cn=%f i=%d i_cn=%d seq %s",
         lo, i, mindist, mindist_cnote, minidx, minidx_cnote, buf);
  free (seq); return note;
}

static void load_waveform_db () {
  SF_INFO sf_info; memset (&sf_info, 0, sizeof (sf_info));
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
  recog_frames = srate / 100 * recog_ms / 10;
  train_max_frames = srate / 100 * train_max_ms / 10;
  gpt_buf_max = note_midi2freq (himidi) * recog_ms / 1000 * 64;

  gpt_buf = malloc (sizeof (gridpt) * gpt_buf_max);
  ASSERT (gpt_buf != NULL);
  gpt_seqdb = malloc (gpt_seqdb_max * sizeof (gpt_seq));
  ASSERT (gpt_seqdb != NULL);

  double x;
  sample_anl_state anls; sample_anl_state_init (&anls);

  // find noise level in training file
  for (sf_count_t i = 0; i < initial_sil_frames; ++i) {
    if (sf_read_double (dbf, &x, 1) < 1) goto premature;
    analyze_sample (x, &anls);
  }
  norm_noise_peak = anls.noise_peak = 4 * anls.max_x;
  TRACE (TRACE_DIAG, "Noise peak in db file: %.5f", (float) norm_noise_peak);

  for (midi_note_t note = lomidi; note <= himidi; ++note) {
    double note_period = 1 / note_midi2freq (note);

    TRACE (TRACE_INT, "Reading MIDI note %d waveform, frame=%ld",
               (int) note, (long) my_sf_tell (dbf));
    while (anls.voiced_ago > 1) {  // skip silence
      if (sf_read_double (dbf, &x, 1) < 1) goto premature;
      analyze_sample (x, &anls);
    }
    TRACE (TRACE_INT, "Found note at frame=%ld", (long) my_sf_tell (dbf));

    sf_count_t note_pos = my_sf_tell (dbf);
    anls.count = 1;
    sample_anl_state stdanls = anls;
    gpt_buf_len = 0;
    do {
      if ((anls.evt & ((1 << ANL_EVT_PEAK) | (1 << ANL_EVT_ZERO))) != 0 &&
          anls.count < train_max_frames)
      {
        for (int part = 0; part < 2; ++part) {
#define TEST( part, cond ) case part: if (! (cond)) continue; break
          switch (part) {
            TEST (0, (anls.evt & (1 << ANL_EVT_PEAK)) != 0 &&
                  gpt_buf_add (anls.old_peak, anls.peak_at) > INV_SEMITONE_RATIO);
            TEST (1, (anls.evt & (1 << ANL_EVT_ZERO)) != 0 &&
                  gpt_buf_add (0, anls.count) > INV_SEMITONE_RATIO);
          }
#undef TEST
          // when TEST succeeds a point was added AND completed a sequence:
          while (gpt_buf_dur (0) > (recog_frames + note_period * srate / 4) * SEMITONE_RATIO)
            gpt_buf_shift ();
          gpt_seq_add_all (note, INV_SEMITONE_RATIO * recog_frames);
        }
      }
      if (sf_read_double (dbf, &x, 1) < 1) goto premature;
      analyze_sample (x, &anls);
    } while ((anls.evt & (1 << ANL_EVT_SIL)) == 0);
    TRACE (TRACE_INT, "MIDI note %d Waveform ends at frame=%ld, seqs=%d",
           (int) note, (long) my_sf_tell (dbf), gpt_seqdb_len);

    if (note == stdmidi) {  // go back and analyze it
      sf_count_t saved_pos = my_sf_tell (dbf);
      sf_seek (dbf, note_pos, SEEK_SET);
      stdanls.npeaks = 0; stdanls.max_peak = 0; stdanls.max_peak_idx = 0;
      TRACE (TRACE_INT, "Back to frame=%ld for stdmidi",
                 (long) my_sf_tell (dbf));
      // gather calibration info
      while ((stdanls.evt & (1 << ANL_EVT_SIL)) == 0 &&
             stdanls.npeaks < stdmidi_npeaks)
      {
        if (sf_read_double (dbf, &x, 1) < 1) goto premature;
        analyze_sample (x, &stdanls);
        if ((stdanls.evt & (1 << ANL_EVT_PEAK)) != 0)
          TRACE (TRACE_INT, "Peak at frame %ld %f",
                 (long) my_sf_tell (dbf), (float) stdanls.old_peak);
      }
      // resume loading of notes
      stdmidi_anls = stdanls;
      sf_seek (dbf, saved_pos, SEEK_SET);
      TRACE (TRACE_INT, "Returned to frame=%ld", (long) my_sf_tell (dbf));
    }
  }

  qsort (gpt_seqdb, gpt_seqdb_len, sizeof (gpt_seqdb [0]),
         (int (*) (const void *, const void *)) gpt_seq_cmp);

  for (size_t i = 0; i < gpt_seqdb_len; ++i) {
    char buf [2048]; gpt_seq_dump (&gpt_seqdb [i], buf, sizeof (buf));
    TRACE (TRACE_INT, "seq i=%d %s", i, buf);
    if (i % 10 == 0) trace_flush ();
  }
  TRACE (TRACE_INFO, "%d sequences loaded from training file", gpt_seqdb_len);

  return;

premature:
  TRACE (TRACE_FATAL, "Premature end of training file file, frame=%ld",
             (long) my_sf_tell (dbf));
  myshutdown (1);
}

static sample_t get_jsample () {
  static jack_nframes_t jdatalen = 0;
  static sample_t *jdataptr = NULL,  ///< Beginning of current data in @c jbuf
    *jdataend = NULL;  ///< End of current data in @c jbuf
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
    if (zombified)
      myshutdown (1);
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

static void *process_thread (void *arg) {
  (void) arg;

  int rc;
  jack_nframes_t lost_frames = 0;

  for (;;) {

    jack_nframes_t nframes_orig = jack_cycle_wait (jclient),
      nframes = nframes_orig;
    if (0 == nframes || 0 == jack_port_connected (jport)) {
      jack_cycle_signal (jclient, 0);
      continue;
    }

    jack_midi_data_t evt [24];
    size_t evtcnt = 0;
    for (; evtcnt < sizeof (evt) / sizeof (evt [0]) / 3
           && 0 == sem_trywait (&jmidibuf_sem);
         ++evtcnt)
    {
      char ch;
      jack_ringbuffer_read (jmidibuf, &ch, 1);
      pack_midi_evt (&evt [evtcnt * 3],
                     ch & ((1 << 7) - 1), (ch & (1 << 7)) != 0);
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
    jack_ringbuffer_write (jbuf, (const char *) in, nframes * sizeof (sample_t));

    jack_cycle_signal (jclient, 0);

    if (nframes > 0) {
      struct sembuf sops = { .sem_num = 0, .sem_op = nframes, .sem_flg = 0 };
      semop (jbufavail_semid, &sops, 1);
    }
    lost_frames += nframes_orig - nframes;
    if (lost_frames > 0) {
      if (0 == pthread_mutex_trylock (&trace_buf->lock)) {
        //TRACE (TRACE_WARN, "Lost frames %d", (int) lost_frames);
        lost_frames = 0;
        pthread_mutex_unlock (&trace_buf->lock);
      }
    }
  }

  return NULL;
}

/// Calibrates the live signal against the waveform database.
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
  TRACE (TRACE_INFO, "Scaling factor against training file: %f", (float) jnorm_factor);

  // skip rest of note
  while (analyze_sample (get_jsample (), &janls),
         (janls.evt & (1 << ANL_EVT_SIL)) == 0);
}

static void queue_note (midi_note_t note, int on) {
  char ch = note + transpose_semitones; if (on) ch |= 1 << 7;
  jack_ringbuffer_write (jmidibuf, &ch, 1);
}

static void midi_server () {
  midi_note_t cnote = 0;
  gpt_seq_anl_state gpbs;
  gpt_seq_anl_state_init (&gpbs);
  TRACE (TRACE_IMPT, "MIDI server started, delay %lf", recog_ms);
  gpt_buf_len = 0;
  for (;;) {
    analyze_sample (get_jsample () * jnorm_factor, &janls);
    if ((janls.evt & (1 << ANL_EVT_SIL)) != 0 && cnote != 0) {
      queue_note (cnote, 0); sem_post (&jmidibuf_sem);
      cnote = 0;
      gpt_seq_anl_state_init (&gpbs);
      TRACE (TRACE_INFO, "Note off");
      gpt_buf_len = 0;
    }
    if ((janls.evt & ((1 << ANL_EVT_PEAK) | (1 << ANL_EVT_ZERO))) != 0)
    {
      for (int part = 0; part < 2; part++) {
#define TEST( part, cond ) case part: if (! (cond)) continue; break
        switch (part) {
          TEST (0, (janls.evt & (1 << ANL_EVT_PEAK)) != 0 &&
                gpt_buf_add (janls.old_peak, janls.peak_at) > 1);
          TEST (1, (janls.evt & (1 << ANL_EVT_ZERO)) != 0 &&
                gpt_buf_add (0, janls.count) > 1);
        }
#undef TEST
        // when TEST succeeds a point was added AND completed a sequence:
        while (gpt_buf_dur (1) > recog_frames)
          gpt_buf_shift ();
        midi_note_t note = gpt_seq_analyze (&gpbs);
        if (note != MIDI_NOTE_NONE && note != cnote) {
          int cut_last = cnote != 0;
          queue_note (note, 1);
          if (cut_last)
            queue_note (cnote, 0);
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
  // Wake up get_jsample(). It will test zombified and realize there's no
  // extra sample.
  struct sembuf sops = { .sem_num = 0, .sem_op = 1, .sem_flg = 0 };
  semop (jbufavail_semid, &sops, 1);
  TRACE (TRACE_FATAL, "Jack shut us down");
}

static void setup_audio () {
  TRACE (TRACE_DIAG, "jack setup");
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
  jack_options_t jopt = 0;  // JackNoStartServer;
  jack_status_t jstat;
  if (NULL == (jclient = jack_client_open (MYNAME, jopt, &jstat)))
    goto jack_setup_fail;
  jack_on_shutdown (jclient, on_jack_shutdown, NULL);
  int jsrate = jack_get_sample_rate (jclient);
  if (jsrate != srate) {
    TRACE (TRACE_FATAL, "Sample rate %d does not match training file's (%d)",
           jsrate, srate);
    goto jack_setup_fail;
  }

  jack_nframes_t jmaxbufsz = jack_get_buffer_size (jclient);
  TRACE (TRACE_INFO, "Connected to jack, bufsize=%d, srate=%d",
             (int) jmaxbufsz, (int) srate);
  jbuf = jack_ringbuffer_create (jbuf_len * sizeof (sample_t)); ASSERT (jbuf != NULL);
  jmidibuf = jack_ringbuffer_create (128); ASSERT (jmidibuf != NULL);

  if (0 != jack_set_process_thread (jclient, process_thread, NULL))
    goto jack_setup_fail;

  if (NULL == (jport = jack_port_register (jclient, "in", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0)))
    goto jack_setup_fail;
  if (NULL == (jmidiport = jack_port_register (jclient, "midi_out", JACK_DEFAULT_MIDI_TYPE, JackPortIsOutput, 0)))
    goto jack_setup_fail;

  if (0 != jack_activate (jclient))
    goto jack_setup_fail;

  if (NULL != dstport && 0 != jack_connect (jclient, jack_port_name (jmidiport), dstport))
    goto jack_setup_fail;

  if (NULL != srcport && 0 != jack_connect (jclient, srcport, jack_port_name (jport)))
    goto jack_setup_fail;

  TRACE (TRACE_DIAG, "Activated");

  return;

jack_setup_fail:
  TRACE (TRACE_FATAL, "Jack setup failed");
  myshutdown (1);
}

static void usage (const char *fmt, ...) {
#define NL "\n"
  if (fmt != NULL) {
    printf ("Error: ");
    va_list ap; va_start (ap, fmt);
    vprintf (fmt, ap);
    va_end (ap);
    printf ("\n\n");
  }
  printf ("%s\n",
   MYNAME " is a Jack wave-to-MIDI server for known waveforms"
NL ""
NL "Usage: " MYNAME " -r RANGE -f FILE -d DELAY [OPTIONS]"
NL "  or:  " MYNAME " { -h | --help | -? | --version }"
NL
NL "Options:"
NL "  -f, --db FILE      Notes recorded from synth for training"
NL "  -r, --range RANGE  Range of notes in training file (e.g. F2-C5)"
NL "  -p, --port JP      Jack audio port to listen on (e.g. system:capture_1)"
NL "  -t, --midi-to JP   Jack MIDI port to output to"
NL "  --train-max MSEC   Truncate recorded notes to MSEC during training"
NL "  --log-level N      Log level (defaults to 4, increase for details)"
NL "  --velo V           MIDI note-on velocity (default 100)"
NL "  --velo-off Voff    MIDI note-off velocity (default 0)"
NL "  -c, --chan C       MIDI channel number"
NL "  --transpose SEMIT  Semitones to transpose (e.g. -12)"
NL "  --scale S          Scale restriction (e.g. C#min, EbMaj)"
NL ""
NL MYNAME " needs a mono recording containing all the notes of the chromatic"
NL "scale in the given --range, with short pauses in between. After loading this"
NL "training file, the program expects you to press the middle C key on your"
NL "keyboard or synthesizer once for calibration (prior to this step you must"
NL "connect " MYNAME " to Jack manually if -p was not given). The program"
NL "estimates the loudness of the input signal compared to the training file, "
NL "then starts recognizing notes and sending MIDI messages until killed."
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
      } else if (0 == strcmp (*argv, "-t") || 0 == strcmp (*argv, "--midi-to")) {
        dstport = *++argv;
      } else if (0 == strcmp (*argv, "-d") || 0 == strcmp (*argv, "--delay")) {
        if (sscanf (*++argv, "%lf", &recog_ms) != 1)
          usage ("Bad delay %s", *argv);
      } else if (0 == strcmp (*argv, "--train-max")) {
        sscanf (*++argv, "%lf", &train_max_ms);
      } else if (0 == strcmp (*argv, "-c") || 0 == strcmp (*argv, "--chan")) {
        sscanf (*++argv, "%d", &midi_chan);
        assert (midi_chan >= 0 && midi_chan < 128);
      } else if (0 == strcmp (*argv, "--velo")) {
        sscanf (*++argv, "%d", &velo_on);
      } else if (0 == strcmp (*argv, "--velo-off")) {
        sscanf (*++argv, "%d", &velo_off);
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
      } else {
        usage ("unknown option %s", *argv);
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
}

/// Performs periodic tasks.
/// Stop it with @c pthread_cancel().
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

// pitfall: name clash with shutdown(2)
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

static void setup_sigs () {
  int sigarr [] = { SIGTERM, SIGQUIT, SIGABRT, SIGPIPE, SIGILL, SIGBUS, SIGFPE, SIGINT, SIGALRM };
  sigemptyset (&sigmask);
  for (unsigned i = 0; i < sizeof (sigarr) / sizeof (sigarr [0]); ++i)
    sigaddset (&sigmask, sigarr [i]);
  if (0 != pthread_sigmask (SIG_BLOCK, &sigmask, NULL)) goto err;
  struct sigaction act =
    { .sa_mask = sigmask, .sa_flags = 0, .sa_handler = sig_handler };
  for (unsigned i = 0; i < sizeof (sigarr) / sizeof (sigarr [0]); ++i)
    if (0 != sigaction (sigarr [i], &act, NULL))
      goto err;

  return;

err: perror (__func__); exit (EXIT_FAILURE);
}

int main (int argc, char **argv) {
  (void) argc;

  setup_sigs ();

  init_trace ();

  parse_args (argv);

  load_waveform_db ();

  setup_audio ();

  pthread_sigmask (SIG_UNBLOCK, &sigmask, NULL);

  calibrate ();

  midi_server ();

  myshutdown (0); assert (0);
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

// Local Variables:
// write-file-functions: (lambda () (delete-trailing-whitespace) nil)
// compile-command: "cc -Os -g -std=c99 -pthread -Wall -Wextra -march=native -pipe -ljack -lsndfile -lm synthinvert.c -o synthinvert"
// End:

/*
  Compile with:

  cc -Os -g -std=c99 -D_REENTRANT -Wall -Wextra $CFLAGS -ljack -lsndfile -lm synthinvert.c -o synthinvert
  # -fmudflapth -lmudflapth
*/
