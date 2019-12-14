#if !defined(__IMXRT1062__)  // Teensy 4.x
#error "Only runs on T4"
#endif
#include <ADCL_t4.h>

ADCL *adc;
uint8_t pin_cmp = A1;
int8_t resolution = 10;

void setup() {
  // put your setup code here, to run once:
  while (!Serial && millis() < 5000) ;
  Serial.begin(115200);
  Serial.println("Quick and dirty T4 Analog compare stuff");

  adc = new ADCL;
  
  adc->setResolution(resolution, 0);
  //since cmp pins are on adc4 we have to change the resoultion
  // measurement will be ready if value < 1.0V
  //adc->enableCompare(1.0/3.3*adc->getMaxValue(ADC_0), 0, ADC_0); 

  // ready if value lies out of [1.0,2.0] V
  adc->enableCompareRange(1.0*adc->getMaxValue(ADC_1)/3.3, 2.0*adc->getMaxValue(ADC_1)/3.3, 0, 0, ADC_0); 
}

void loop() {
  // put your main code here, to run repeatedly:
  int value;
  //value = adc->analogRead(15);
  value = adc->analogRead(pin_cmp, 0);
  if(adc->adc0->fail_flag == ADC_ERROR::COMPARISON) {
    //Serial.println("Some other error happened when comparison should have succeeded.");
    //adc->adc0->printError();
  } else {
    Serial.println(value);
  }
  adc->adc0->resetError();

  delay(100);
}