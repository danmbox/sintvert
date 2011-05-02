SINTVERT
A real-time wave-to-MIDI server for known waveforms

Home page: http://danmbox.github.com/sintvert/
Author: Dan A. Muresan (danmbox at gmail dot com)


1. DESCRIPTION:

sintvert needs a mono recording containing all the notes of the
chromatic scale to be recognized, with short pauses in between. Create
this file by recording output from your keyboard or synthesizer. Try
to pick a "clean" patch, i.e. one which has no echo (there is no sound
after you release a key) and which looks as close as possible to a
sine wave.

After loading this training file, the program expects you to press the
middle C key on your synth once for amplitude calibration. The program
estimates the loudness of the input signal compared to the training
file, then starts recognizing notes and sending MIDI messages until
killed.


2. DEPENDENCIES:

* jack, sndfile

* build dependencies: GNU make


3. INSTALLING:

# installs in /usr/local:
make install

# installs elsewhere:
make prefix=/opt/sintvert install
PATH="$PATH":/opt/sintvert

# for packagers:
make DESTDIR=build/ prefix=/usr install
cd build; tar cvzf ../sintvert.tar.gz


4. RUNNING

See sintvert --help (or the manpage).


5. COPYRIGHT:

Copyright 2010-2011 Dan A. Muresan

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
