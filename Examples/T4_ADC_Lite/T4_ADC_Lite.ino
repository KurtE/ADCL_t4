#if !defined(__IMXRT1062__)  // Teensy 4.x
#error "Only runs on T4"
#endif
#include <ADCL_t4.h>

ADCL *adc;

void setup() {
  // put your setup code here, to run once:
  while (!Serial && millis() < 5000) ;
  Serial.begin(115200);
  Serial.println("Quick and dirty T4 Analog Read stuff");

  adc = new ADCL;

}
  

typedef struct {
  uint8_t resolution;
  uint8_t averaging;
  ADC_CONVERSION_SPEED conversion_speed;
  ADC_SAMPLING_SPEED sampling_speed;
} analog_configs_t;

const analog_configs_t test_configs[] = {
  {8, 2, ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, {8, 4,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, 
  {8, 8,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, {8, 16,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, 
  {8, 32,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED},
  {10, 2,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, {10, 4,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, 
  {10, 8,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, {10, 16,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, 
  {10, 32,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED},
  {12, 2,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, {12, 4,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, 
  {12, 8,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, {12, 16,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}, 
  {12, 32,ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED},
  {8, 2, ADC_CONVERSION_SPEED::MED_SPEED, ADC_SAMPLING_SPEED::MED_SPEED}
};
uint8_t test_config_index = 0xff;



void loop() {
  test_config_index++;
  if (test_config_index == (sizeof(test_configs) / sizeof(test_configs[0]))) {
    test_config_index = 0;
  }

  adc->setResolution(test_configs[test_config_index].resolution, 0);
  adc->setResolution(test_configs[test_config_index].resolution, 1);

  adc->setAveraging(test_configs[test_config_index].averaging, 0);
  adc->setAveraging(test_configs[test_config_index].averaging, 1);

  adc->setConversionSpeed(test_configs[test_config_index].conversion_speed, 0);
  adc->setConversionSpeed(test_configs[test_config_index].conversion_speed, 1);

  adc->setSamplingSpeed(test_configs[test_config_index].sampling_speed, 0);
  adc->setSamplingSpeed(test_configs[test_config_index].sampling_speed, 1);

  // put your main code here, to run repeatedly:
  uint32_t t1 = micros();
  int a0 = adc->analogRead(14);
  int a1 = adc->analogRead(15);
  uint32_t t2 = micros();
  int a0_4 = adc->analogRead(14, 0);
  int a1_4 = adc->analogRead(15, 1);

  uint32_t t3 = micros();
  adc->startSingleRead(14, 0);
  adc->startSingleRead(15, 1);
  while (adc->isConverting(0) || adc->isConverting(1) ) ;
  int a0_4C = adc->readSingle(0);
  int a1_4C = adc->readSingle(1);

  uint32_t t4 = micros();
  Sync_result sr = adc->analogSynchronizedRead(14, 15);
  uint32_t t5 = micros();

  Serial.printf("%x:%u:%u:%u(%08x %08x)> %d %d %u : %d %d %u : %d %d %u : %u %u %u\n",
                test_configs[test_config_index].resolution, test_configs[test_config_index].averaging,
                (uint8_t)test_configs[test_config_index].conversion_speed, (uint8_t)test_configs[test_config_index].sampling_speed,
                ADC1_CFG, ADC1_GC,
                a0, a1, t2 - t1,
                a0_4, a1_4, t3 - t2,
                a0_4C, a1_4C, t4 - t3,
                sr.result_adc0, sr.result_adc1, t5 - t4);
  delay(500);
}
