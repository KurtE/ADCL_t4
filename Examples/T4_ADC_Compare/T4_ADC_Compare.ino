#if !defined(__IMXRT1062__)  // Teensy 4.x
#error "Only runs on T4"
#endif
#include <ADCL_t4.h>

ADCL *adc;

void setup() {
  // put your setup code here, to run once:
  while (!Serial && millis() < 5000) ;
  Serial.begin(115200);
  Serial.println("Quick and dirty T4 Analog compare stuff");

  adc = new ADCL;
  adc->setResolution(12, 0);
  adc->enableCompare(ADCL::ACMP3, 17);  //A3
  
  /*  Call it after changing the resolution
  *  Use with interrupts or poll conversion completion with isComplete()
  * 1.  compare true if the result is less than the value1. (greaterThan=0. mode0)
  * 2.  compare true if the result is greater than or equal to value1.
  * greaterThan=1. mode1)
  */
  adc->enableCompareValue(0, 4090, 1);

  /*  the range given by (lowerLimit, upperLimit),including (inclusive=1) the limits or not (inclusive=0).
  *
  *  1.  Value1 <= Value2, compare true if the result is less than value1 
  *  OR the result is Greater than value2. (inclusive = 0, insideRange = 0, mode2)
  *  2.  Value1 >  Value2, compare true if the result is less than value1 
  *  AND the result is Greater than value2. (inclusive = 0, insideRange = 1, mode3)
  *  3.  Value1 <= Value2, compare true if the result is greater than 
  *  OR equal to value1 AND the result is less than or equal to value2.   (inclusive = 1, insideRange = 1, mode4)
  *  4.  Value1 >  Value2, compare true if the result is greater than 
  *  OR equal to value1 OR the result is less than or equal to value2.   (inclusive = 1, insideRange = 0, mode5)
  */
  //adc->enableCompareRange(adc_num, lowerLimit, upperLimit, insideRange, inclusive);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(adc->getAdcCompareRes(ADCL::ACMP3));   // COUT low bit
  delay(500);


}
