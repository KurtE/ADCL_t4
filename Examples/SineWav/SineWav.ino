#include <Audio.h>

// GUItool: begin automatically generated code
AudioSynthWaveform       waveform1;      //xy=110,75
AudioOutputAnalogStereo  audioOutput;
AudioConnection          patchCord1(waveform1, 0, audioOutput, 0);
AudioConnection          patchCord2(waveform1, 0, audioOutput, 1);
// GUItool: end automatically generated code

void setup() {
  AudioMemory(15);
  audioOutput.analogReference(DEFAULT);
  waveform1.begin(WAVEFORM_SINE);
  waveform1.frequency(60);
  waveform1.amplitude(0.99);
}

void loop() {
}
