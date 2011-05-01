// Copyright (C) 2010-2011 Dan Muresan
// Part of sintvert (http://danmbox.github.com/sintvert/)

#ifndef __SINTVERT__MIDIUTIL_H
#define __SINTVERT__MIDIUTIL_H

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

typedef int midi_note_t;  ///< A MIDI note (0-127, 21=A0, 60=C4)
#define MIDI_NOTE_NONE ((midi_note_t) -1)
double SEMITONE_RATIO = 1.06, INV_SEMITONE_RATIO = 0.94,
  QTTONE_RATIO = 1.03, QTTONE_RATIO_INV = 0.97;

static const midi_note_t note_semitones [] = { 0, 2, 4, 5, 7, 9, 11, 12 };
double note_midi2freq (midi_note_t midi) {
  return 27.5 * exp (.05776226504666210911 * (midi - 21));  // A0 = 27.5
}
void note_midi2sci (midi_note_t midi, char *sci) {
  midi_note_t o = midi / 12 - 1, s = midi % 12;
  int sharp = 0;
  char l;
  for (l = 'C'; l <= 'I' && note_semitones [l - 'C'] < s; ++l);
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
static int scale2bits (const char *name, int *scale) {
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


#endif  // __SINTVERT__MIDIUTIL_H
