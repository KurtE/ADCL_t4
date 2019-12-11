#include <Audio.h>
#define POT_PIN 14
#define POT_PIN2 15
int last_pot_val = 0xffff;
int last_pot_val2 = 0xffff;
int led_on_time = 256;
int led_off_time = 256;
elapsedMillis led_timer;

// GUItool: begin automatically generated code
AudioSynthWaveform       waveform1;      //xy=110,75
AudioSynthWaveform       waveform2;      //xy=110,75
AudioOutputAnalogStereo  audioOutput;
AudioConnection          patchCord1(waveform1, 0, audioOutput, 0);
AudioConnection          patchCord2(waveform2, 0, audioOutput, 1);
// GUItool: end automatically generated code

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  AudioMemory(15);
  audioOutput.analogReference(DEFAULT);
  waveform1.begin(WAVEFORM_SINE);
  waveform1.frequency(60);
  waveform1.amplitude(0.99);

  waveform2.begin(WAVEFORM_SINE);
  waveform2.frequency(60);
  waveform2.amplitude(0.99);

  analogReadResolution(12);  // 12 bit resolution
  led_timer = 0;
}

void loop() {
#ifdef POT_PIN
  int pot_val = analogRead(POT_PIN); // lets reduce noise
  if (abs(pot_val - last_pot_val) > 8) {
    last_pot_val = pot_val;
    float new_amp = (float)last_pot_val / 4096.0;
    led_on_time = new_amp * 512;
    led_off_time = 512 - led_on_time;
    waveform1.amplitude(new_amp);
    #ifndef POT_PIN2
    waveform2.amplitude(new_amp);
    #endif
    if (Serial) {
      Serial.print("New Amp 1: ");
      Serial.print(new_amp, 4);
      Serial.print("(");
      Serial.print(pot_val, DEC);
      Serial.println(")");
    }
  }
#ifdef POT_PIN2
  pot_val = analogRead(POT_PIN2); // lets reduce noise
  if (abs(pot_val - last_pot_val2) > 8) {
    last_pot_val2 = pot_val;
    float new_amp = (float)last_pot_val2 / 4096.0;
    led_on_time = new_amp * 512;
    led_off_time = 512 - led_on_time;
    waveform2.amplitude(new_amp);
    if (Serial) {
      Serial.print("New Amp 2: ");
      Serial.print(new_amp, 4);
      Serial.print("(");
      Serial.print(pot_val, DEC);
      Serial.println(")");
    }
  }
#endif  
  if (digitalReadFast(13)) {
    if (led_timer > led_on_time) {
      digitalWriteFast(13, LOW);
      led_timer = 0;
    }
  } else if (led_timer > led_off_time) {
    digitalWriteFast(13, HIGH);
    led_timer = 0;
  }
#endif  
}