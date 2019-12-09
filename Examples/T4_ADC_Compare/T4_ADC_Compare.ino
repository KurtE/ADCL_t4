#if !defined(__IMXRT1062__)  // Teensy 4.x
#error "Only runs on T4"
#endif
#include <ADCL_t4.h>

ADCL *adc;
uint8_t inp_pin = 15;
int8_t resolution = 10;

void setup() {
  // put your setup code here, to run once:
  while (!Serial && millis() < 5000) ;
  Serial.begin(115200);
  Serial.println("Quick and dirty T4 Analog compare stuff");

  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);

  analogWriteFrequency(2,60000);
  analogWrite(2,127);
  adc = new ADCL;
  
  adc->setResolution(12, 0);
  //since cmp pins are on adc4 we have to change the resoultion
  adc->enableCompare(0, 4000, false);
}

void loop() {
  // put your main code here, to run repeatedly:
  int value;
  //value = adc->analogRead(15);
  value = adc->analogReadCmp(15,0);
  Serial.println(value);
  delay(100);
}
