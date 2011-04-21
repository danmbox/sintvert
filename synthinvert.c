#define _XOPEN_SOURCE 600

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <assert.h>

#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include <jack/jack.h>
#include <jack/ringbuffer.h>
#include <sndfile.h>

typedef int midi_note_t;  /* 0-127, 21=A0, 60=C4 */
typedef jack_default_audio_sample_t sample_t;

const char *name = "synthinvert";
const char *srcport = NULL;
const char *dbfname = NULL;
midi_note_t lomidi = 0, himidi = 0;  /* range of our synth */
midi_note_t stdmidi = 60;  /* note to calibrate against */
int stdmidi_npeaks = 50;  /* # of peaks to read for calibration */
int velo0 = 100;
int initial_silence_ms = 450;
int wavebrk_silence_ms = 2;

typedef struct {
  char *buf1, *buf2, *ptr, *end, *other;
  size_t sz;
  pthread_mutex_t lock, switchlock;
} memfile;
void memfile_init (memfile *f, size_t sz) {
  f->sz = sz;
  f->buf1 = calloc (sz, 1); f->buf2 = calloc (sz, 1);
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
  TRACE_FATAL, TRACE_ERR, TRACE_WARN, TRACE_IMPT, TRACE_INFO, TRACE_DIAG, TRACE_INT
} trace_pri_t;
const char *trace_level_symb = "FEW!ID          ";
trace_pri_t trace_level = TRACE_INT;
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
  memfile_vprintf (trace_buf, fmt, ap); memfile_printf (trace_buf, " [%s]\n", fn);
  pthread_mutex_unlock (&trace_buf->lock);
  va_end (ap);
}
#define TRACE(pri, ...) trace_msg (pri, __func__, __VA_ARGS__)

sf_count_t my_sf_tell (SNDFILE *sndfile)
{ return sf_seek (sndfile, 0, SEEK_CUR); }

typedef struct {
  int npeaks;
  sample_t peak, old_peak;
  sample_t max_x, max_peak, noise_peak;
  jack_nframes_t count, max_peak_idx, voiced_ago;
  int event;
} analyzer_state;
enum { ANL_EVT_PEAK = 0 };

pthread_t poll_thread_tid = 0;
jack_nframes_t initial_silence_frames = 0, wavebrk_silence_frames = 0;
long srate = 0;
jack_client_t *jclient = NULL;
volatile int zombified = 0;
jack_nframes_t jmaxbufsz = 0;
jack_port_t *jport = NULL;
jack_ringbuffer_t *jbuf = NULL;
sample_t *jdataptr = NULL, *jdataend = NULL;
jack_nframes_t jbufavail = 0, jdatalen = 0;
pthread_mutex_t jbufavail_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t jbufavail_cond = PTHREAD_COND_INITIALIZER;
sample_t norm_noise_peak = 0;
analyzer_state stdmidi_anls;
double jnorm_factor = 0.0;

void analyzer_state_init (analyzer_state *s) {
  memset (s, 0, sizeof (*s));
  s->voiced_ago = 15 * srate;
}

void shutdown (int failure);

void process_sample (sample_t x, analyzer_state * const s) {
  s->event = 0;
  sample_t absx = x > 0 ? x : -x;
  if (absx > s->max_x)
    s->max_x = absx;
  if (s->noise_peak > 0.0 && absx > s->noise_peak) {
    s->voiced_ago = 0;
    if (absx > 5 * s->noise_peak && s->peak * (s->peak - x) <= 0)
      s->peak = x;
  }
  sample_t border = (sample_t) copysign (s->noise_peak, s->peak);
  if (s->peak != 0.0 && (border - x) * (border - s->peak) <= 0) {
    s->event |= (1 << ANL_EVT_PEAK);
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
  ++s->count;
}

int process_waveform_db () {
  SF_INFO sf_info;
  memset (&sf_info, sizeof (sf_info), 0);
  SNDFILE *dbf = sf_open (dbfname, SFM_READ, &sf_info);
  if (NULL == dbf) {
    TRACE (TRACE_FATAL, "Could not open file %s", dbfname);
    return 0;
  }
  if (1 != sf_info.channels) {
    TRACE (TRACE_FATAL, "File %s has %d channels, expecting exactly 1",
               dbfname, (int) sf_info.channels);
    return 0;
  }
  srate = sf_info.samplerate;
  initial_silence_frames = srate / 100 * initial_silence_ms / 10;
  wavebrk_silence_frames = srate / 100 * wavebrk_silence_ms / 10;

  double x;
  analyzer_state anls;
  analyzer_state_init (&anls);

  // find noise level in waveform db
  for (sf_count_t i = 0; i < initial_silence_frames; i++) {
    if (sf_read_double (dbf, &x, 1) < 1) goto premature;
    process_sample (x, &anls);
  }
  norm_noise_peak = anls.noise_peak = 4 * anls.max_x;
  TRACE (TRACE_DIAG, "Noise peak in db file: %.5f",
             (float) norm_noise_peak);

  for (midi_note_t n = lomidi; n <= himidi; n++) {
    TRACE (TRACE_INT, "Reading MIDI %d waveform, frame=%ld",
               (int) n, (long) my_sf_tell (dbf));
    // skip silence
    while (anls.voiced_ago > 1) {
      if (sf_read_double (dbf, &x, 1) < 1) goto premature;
      process_sample (x, &anls);
    }
    TRACE (TRACE_INT, "Found note at frame=%ld",
               (long) my_sf_tell (dbf));

    sf_count_t note_pos = my_sf_tell (dbf);
    analyzer_state stdanls = anls;
    while (anls.voiced_ago < wavebrk_silence_frames) {
      if (sf_read_double (dbf, &x, 1) < 1) goto premature;
      process_sample (x, &anls);
      // TODO: analyze waveform
    }
    TRACE (TRACE_INT, "MIDI %d Waveform ends at frame=%ld",
               (int) n, (long) my_sf_tell (dbf));
    if (n == stdmidi) {
      sf_count_t saved_pos = my_sf_tell (dbf);
      sf_seek (dbf, note_pos, SEEK_SET);
      stdanls.npeaks = 0; stdanls.max_peak = 0; stdanls.max_peak_idx = 0;
      TRACE (TRACE_INT, "Back to frame=%ld for stdmidi",
                 (long) my_sf_tell (dbf));
      // gather calibration info
      while (stdanls.voiced_ago < wavebrk_silence_frames &&
             stdanls.npeaks < stdmidi_npeaks)
      {
        if (sf_read_double (dbf, &x, 1) < 1) goto premature;
        process_sample (x, &stdanls);
        if ((stdanls.event & (1 << ANL_EVT_PEAK)) != 0)
          TRACE (TRACE_INT, "Peak at frame %ld %f",
                 (long) my_sf_tell (dbf), (float) stdanls.old_peak);
      }
      stdmidi_anls = stdanls;
      sf_seek (dbf, saved_pos, SEEK_SET);
      TRACE (TRACE_INT, "Returned to frame=%ld",
                 (long) my_sf_tell (dbf));
    }
  }

  return 1;

premature:
  TRACE (TRACE_FATAL, "Premature end of waveform db file, frame=%ld",
             (long) my_sf_tell (dbf));
  return 0;
}

sample_t get_jsample () {
  if (zombified) {
    TRACE (TRACE_FATAL, "Jack shut us down");
    shutdown (1);
  }
  if (jdataptr == jdataend) {
    jack_ringbuffer_read_advance (jbuf, jdatalen * sizeof (sample_t));
    jdataptr = jdataend = NULL;
    for (pthread_mutex_lock (&jbufavail_lock);
         jbufavail <= 0;
         pthread_cond_wait (&jbufavail_cond, &jbufavail_lock));
    jack_ringbuffer_data_t jdatainfo [2];
    jack_ringbuffer_get_read_vector (jbuf, jdatainfo);
    jdatalen = jdatainfo [0].len / sizeof (sample_t);
    if (jdatalen > jbufavail) jdatalen = jbufavail;
    jbufavail -= jdatalen;
    pthread_mutex_unlock (&jbufavail_lock);
    if (jdatalen == 0) {
      TRACE (TRACE_FATAL, "jdatalen=0, jbufavail=%d, jdatainfo=%d", (int) jbufavail, (int) jdatainfo [0].len);
      shutdown (1);
    }
    jdataptr = (sample_t *) jdatainfo [0].buf;
    jdataend = jdataptr + jdatalen;
  }
  return *jdataptr++;
}

void *process_thread (void *arg) {
  (void) arg;

  for (;;) {
    jack_nframes_t nframes_orig = jack_cycle_wait (jclient),
      nframes = nframes_orig;
    if (nframes <= 0 || zombified) break;
    sample_t *in = jack_port_get_buffer (jport, nframes);
    size_t avail = jack_ringbuffer_write_space (jbuf) / sizeof (sample_t);
    if (avail < nframes) nframes = avail;
    jack_nframes_t saved_bytes = 
      jack_ringbuffer_write (jbuf, (const char *) in, nframes * sizeof (sample_t));
    jack_cycle_signal (jclient, 0);

    assert (saved_bytes == nframes * sizeof (sample_t));
    pthread_mutex_lock (&jbufavail_lock);
    jbufavail += nframes;
    pthread_cond_signal (&jbufavail_cond);
    pthread_mutex_unlock (&jbufavail_lock);
    if (nframes != nframes_orig)
      TRACE (TRACE_WARN, "Lost frames %d", (int) (nframes_orig - nframes));
  }

  return NULL;
}

void calibrate () {
  sample_t noise_peak;
  analyzer_state anls;
  analyzer_state_init (&anls);

  // find noise level
  TRACE (TRACE_DIAG, "Analysing noise...");
  for (jack_nframes_t i = 0; i < initial_silence_frames; i++)
    process_sample (get_jsample (), &anls);
  noise_peak = anls.noise_peak = 4 * anls.max_x;
  TRACE (TRACE_DIAG, "Noise peak in input: %.5f", (float) noise_peak);

  // skip silence
  TRACE (TRACE_DIAG, "Calibrating against MIDI %d waveform", (int) stdmidi);
  while (process_sample (get_jsample (), &anls), anls.voiced_ago > 1);

  TRACE (TRACE_INT, "Found note");
  assert (anls.npeaks == 0);
  while (process_sample (get_jsample (), &anls),
         (anls.voiced_ago < wavebrk_silence_frames && anls.npeaks < stdmidi_npeaks)) {
    if ((anls.event & (1 << ANL_EVT_PEAK)) != 0)
      TRACE (TRACE_INT, "Peak at frame %d %f", (int) anls.count, (float) anls.old_peak);
  }
  if (anls.max_peak_idx != stdmidi_anls.max_peak_idx) {
    TRACE (TRACE_FATAL, "calibration mismatch npeaks=%d %d %d %f %f",
               anls.npeaks,
               anls.max_peak_idx, stdmidi_anls.max_peak_idx,
               anls.max_peak, stdmidi_anls.max_peak);
    shutdown (1);
  }
  jnorm_factor = anls.max_peak / stdmidi_anls.max_peak;
  TRACE (TRACE_INFO, "Scaling factor against waveform db: %f", (float) jnorm_factor);
}

void midi_server () {
}

void on_jack_shutdown (void *arg) {
  (void) arg;

  zombified = 1;
  TRACE (TRACE_FATAL, "Jack shut us down");
  shutdown (1);
}

int setup_audio () {
  TRACE (TRACE_INT, "jack setup");
  fflush(stdout); fflush (stderr);

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
  TRACE (TRACE_INFO, "Connected client, bufsize=%d, srate=%d",
             (int) jmaxbufsz, (int) srate);
  jbuf = jack_ringbuffer_create (64 * jmaxbufsz * sizeof (sample_t));

  if (0 != jack_set_process_thread (jclient, process_thread, NULL))
    goto jack_setup_fail;

  if (0 != jack_activate (jclient))
    goto jack_setup_fail;

  if (NULL == (jport = jack_port_register (jclient, "in", JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0)))
    goto jack_setup_fail;
  TRACE (TRACE_INT, "jport=%p", jport);
  
  if (0 != jack_connect (jclient, srcport, jack_port_name (jport)))
    goto jack_setup_fail;

  return 1;

jack_setup_fail:
  TRACE (TRACE_FATAL, "Jack setup failed");
  if (jclient != NULL)
    jack_client_close (jclient);
  return 0; 
}

midi_note_t scinote2midi (char *scinote) {
  static const int semitones [] = { 0, 2, 3, 5, 7, 8, 10, 12 };
  char lc = toupper (scinote [0]);
  
  midi_note_t l =  lc - 'A',
    o = scinote [1] - '0' - (lc < 'C' ? 0 : 1);
  return 21 + 12 * o + semitones [l];
}

void parse_args (char **argv) {
  for (++argv; *argv != NULL; ++argv) {
    if (*argv [0] == '-') {
      if (0 == strcmp (*argv, "--db")) {
        dbfname = *++argv;
      } else if (0 == strcmp (*argv, "--range")) {
        lomidi = scinote2midi (argv [1]);
        himidi = scinote2midi (argv [1] + 2);
        ++argv;
      } else if (0 == strcmp (*argv, "--port")) {
        srcport = *++argv;
      }
    }
    else break;
  }

  if (NULL == dbfname) {
    TRACE (TRACE_FATAL, "waveform database file (--db) not specified");
    shutdown (1);
  }
  if (0 == himidi) {
    TRACE (TRACE_FATAL, "note range (e.g. --range G2C5) not specified");
    shutdown (1);
  }
  if (lomidi > stdmidi || stdmidi > himidi) {
    TRACE (TRACE_FATAL, "calibration note out of range");
    shutdown (1);
  }
  if (NULL == srcport) {
    TRACE (TRACE_FATAL, "no source port (--port) specified");
    shutdown (1);
  }
}

void trace_flush () {
  pthread_mutex_lock (&trace_buf->switchlock);
  memfile_switchbuf (trace_buf);
  fprintf (stdtrace, "%s", trace_buf->other);
  pthread_mutex_unlock (&trace_buf->switchlock);
  fflush (stdtrace);
}

void *poll_thread (void *arg) {
  (void) arg;

  for (;;) {
    struct timespec sleepreq = { tv_sec: 0, tv_nsec: 200000000L };
    nanosleep (&sleepreq, NULL);
    trace_flush ();
  }
}

void init_trace () {
  trace_buf = memfile_alloc (1 << 20);
  stdtrace = fdopen (dup (fileno (stdout)), "w");
  setvbuf (stdtrace, malloc (1 << 16), _IOFBF, 1 << 16);
  pthread_create (&poll_thread_tid, NULL, poll_thread, NULL);
  TRACE (TRACE_INFO, "Starting");
}

void shutdown (int failure) {
  pthread_cancel (poll_thread_tid);
  pthread_join (poll_thread_tid, NULL);
  if (jclient != NULL && ! zombified)
    jack_client_close (jclient);
  TRACE (TRACE_INFO, "Exiting");
  trace_flush ();
  exit (failure ? EXIT_FAILURE : EXIT_SUCCESS);
}

int main (int argc, char **argv) {
  (void) argc;

  init_trace ();

  parse_args (argv);

  if (! process_waveform_db ())
    shutdown (1);

  if (! setup_audio ())
    shutdown (1);
  calibrate ();
  midi_server ();

  shutdown (0); return EXIT_SUCCESS;
}

/*
cc -std=c99 -Wall -Wextra $CFLAGS -ljack -lsndfile -lm synthinvert.c -o synthinvert
*/
