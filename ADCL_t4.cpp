#include <Arduino.h>
#include "ADCL_t4.h"
// modified version from T4 analog.c
// 0x80 bit - Only on ADC2
// 0x40 bit - only on ADC1
const uint8_t t4_pin_to_channel[] = {
  7,  // 0/A0  AD_B1_02
  8,  // 1/A1  AD_B1_03
  12, // 2/A2  AD_B1_07
  11, // 3/A3  AD_B1_06
  6,  // 4/A4  AD_B1_01
  5,  // 5/A5  AD_B1_00
  15, // 6/A6  AD_B1_10
  0,  // 7/A7  AD_B1_11
  13, // 8/A8  AD_B1_08
  14, // 9/A9  AD_B1_09
  0x40 + 1, // 24/A10 AD_B0_12
  0x40 + 2, // 25/A11 AD_B0_13
  0x80 + 3, // 26/A12 AD_B1_14 - only on ADC2, 3
  0x80 + 4, // 27/A13 AD_B1_15 - only on ADC2, 4
  7,  // 14/A0  AD_B1_02
  8,  // 15/A1  AD_B1_03
  12, // 16/A2  AD_B1_07
  11, // 17/A3  AD_B1_06
  6,  // 18/A4  AD_B1_01
  5,  // 19/A5  AD_B1_00
  15, // 20/A6  AD_B1_10
  0,  // 21/A7  AD_B1_11
  13, // 22/A8  AD_B1_08
  14, // 23/A9  AD_B1_09
  0x40 + 1, // 24/A10 AD_B0_12
  0x40 + 2, // 25/A11 AD_B0_13
  0x80 + 3, // 26/A12 AD_B1_14 - only on ADC2, 3
  0x80 + 4 // 27/A13 AD_B1_15 - only on ADC2, 4
};

#if 0
extern int T4AnalogRead(uint8_t pin, uint8_t adc_num) {
  if (pin > sizeof(t4_pin_to_channel)) return 0;
  // I believe all the calibration should be done by now
  uint8_t ch = t4_pin_to_channel[pin];
  switch (adc_num) {
    case 1:
      if (ch & 0x80) return 0;  // does not handle this pin
      break;
    case 2:
      if (ch & 0x40) return 0;  // does not handle this pin
      break;
    default:
      adc_num =  (ch & 0x80) ? 2 : 1;
  }
  if (adc_num == 1) {
    ADC1_HC0 = ch & 0x3f;
    while (!(ADC1_HS & ADC_HS_COCO0)) ; // wait
    return ADC1_R0;
  } else {
    ADC2_HC0 = ch & 0x3f;
    while (!(ADC2_HS & ADC_HS_COCO0)) ; // wait
    return ADC2_R0;
  }
}

bool T4AnalogReadBoth(uint8_t adc1_pin, uint16_t &adc1_val, uint8_t adc2_pin, uint16_t &adc2_val)
{
  if (adc1_pin > sizeof(t4_pin_to_channel)) return false;
  uint8_t ch_adc1 = t4_pin_to_channel[adc1_pin];
  if (ch_adc1 & 0x80) return false; // only valid on ADC2

  if (adc2_pin > sizeof(t4_pin_to_channel)) return false;
  uint8_t ch_adc2 = t4_pin_to_channel[adc2_pin];
  if (ch_adc2 & 0x40) return false; // only valid on ADC1

  // Now lets start the conversions...
  ADC1_HC0 = ch_adc1 & 0x3f;
  ADC2_HC0 = ch_adc2 & 0x3f;

  // now wait for both to complete
  while (!(ADC1_HS & ADC_HS_COCO0) || !(ADC2_HS & ADC_HS_COCO0)) ; // wait
  adc1_val = ADC1_R0;
  adc2_val = ADC2_R0;
  return true;
}

bool T4AnalogReadStart(uint8_t pin, uint8_t adc_num)
{
  // warning not checking to see if already busy or ...
  if (pin > sizeof(t4_pin_to_channel)) return false;
  // I believe all the calibration should be done by now
  uint8_t ch = t4_pin_to_channel[pin];
  switch (adc_num) {
    case 1:
      if (ch & 0x80) return false;  // does not handle this pin
      break;
    case 2:
      if (ch & 0x40) return false;  // does not handle this pin
      break;
    default:
      return false;
  }
  if (adc_num == 1) ADC1_HC0 = ch & 0x3f;
  else ADC2_HC0 = ch & 0x3f;
  return true;
}

int T4AnalogReadComplete(uint8_t adc_num, bool wait_complete) {
  if (adc_num == 1) {
    if (!wait_complete && !(ADC1_HS & ADC_HS_COCO0)) return -1;
    while (!(ADC1_HS & ADC_HS_COCO0)) ; // wait
    return ADC1_R0;
  } else {
    if (!wait_complete && !(ADC2_HS & ADC_HS_COCO0)) return -1;
    while (!(ADC2_HS & ADC_HS_COCO0)) ; // wait
    return ADC2_R0;
  }
}


void T4AnalogReadRes(unsigned int bits, uint8_t adc_num)
{
  uint32_t tmp32, mode;

  if (bits == 8) {
    // 8 bit conversion (17 clocks) plus 8 clocks for input settling
    mode = ADC_CFG_MODE(0) | ADC_CFG_ADSTS(3);
  } else if (bits == 0x88)  {
    // 8 bit fast conversion
    mode = ADC_CFG_MODE(0) | ADC_CFG_ADSTS(0);
  } else if (bits == 10) {
    // 10 bit conversion (17 clocks) plus 20 clocks for input settling
    mode = ADC_CFG_MODE(1) | ADC_CFG_ADSTS(2) | ADC_CFG_ADLSMP;
  } else {
    // 12 bit conversion (25 clocks) plus 24 clocks for input settling
    mode = ADC_CFG_MODE(2) | ADC_CFG_ADSTS(3) | ADC_CFG_ADLSMP;
  }

  if (adc_num == 1) {
    tmp32  = (ADC1_CFG & (0xFFFFFC00));
    tmp32 |= (ADC1_CFG & (0x03));  // ADICLK
    tmp32 |= (ADC1_CFG & (0xE0));  // ADIV & ADLPC

    tmp32 |= mode;
    ADC1_CFG = tmp32;
  } else {
    tmp32  = (ADC2_CFG & (0xFFFFFC00));
    tmp32 |= (ADC2_CFG & (0x03));  // ADICLK
    tmp32 |= (ADC2_CFG & (0xE0));  // ADIV & ADLPC
    tmp32 |= mode;
    ADC2_CFG = tmp32;
  }
}

void T4AnalogReadAveraging(unsigned int num, uint8_t adc_num)
{
  uint32_t mode, mode1;

  //disable averaging, ADC1 and ADC2
  if (adc_num == 1 ) {
    ADC1_GC &= ~0x20;
    mode = ADC1_CFG & ~0xC000;
    if (num >= 32) {
      mode |= ADC_CFG_AVGS(3);
    } else if (num >= 16) {
      mode |= ADC_CFG_AVGS(2);
    } else if (num >= 8) {
      mode |= ADC_CFG_AVGS(1);
    } else if (num >= 4) {
      mode |= ADC_CFG_AVGS(0);
    } else {
      mode |= 0;
    }
    ADC1_CFG = mode;
    if (num >= 4) {
      ADC1_GC |= ADC_GC_AVGE;// turns on averaging
    }
  } else {
    ADC2_GC &= ~0x20;
    mode1 = ADC2_CFG & ~0xC000;

    if (num >= 32) {
      mode1 |= ADC_CFG_AVGS(3);
    } else if (num >= 16) {
      mode1 |= ADC_CFG_AVGS(2);
    } else if (num >= 8) {
      mode1 |= ADC_CFG_AVGS(1);
    } else if (num >= 4) {
      mode1 |= ADC_CFG_AVGS(0);
    } else {
      mode1 |= 0;
    }

    ADC2_CFG = mode1;
    if (num >= 4) {
      ADC2_GC |= ADC_GC_AVGE;// turns on averaging
    }
  }
}
#endif
//==========================================================================================
//class ADCL
//==========================================================================================
  /////////////// METHODS TO SET/GET SETTINGS OF THE ADC ////////////////////

  //! Set the voltage reference you prefer, default is vcc
  /*! It recalibrates at the end.
      \param type can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 (not for Teensy LC) or ADC_REFERENCE::REF_EXT
      \param adc_num ADC number to change.
  */
//  void ADCL::setReference(ADC_REFERENCE type, int8_t adc_num)
//{
//}




  //! Change the resolution of the measurement.
  /*!
     \param bits is the number of bits of resolution.
     For single-ended measurements: 8, 10, 12 or 16 bits.
     For differential measurements: 9, 11, 13 or 16 bits.
     If you want something in between (11 bits single-ended for example) select the immediate higher
     and shift the result one to the right.
     Whenever you change the resolution, change also the comparison values (if you use them).
      \param adc_num ADC number to change.
  */
void ADCL::setResolution(uint8_t bits, int8_t adc_num)
{
  uint32_t tmp32, mode;

  if (bits == 8) {
    // 8 bit conversion (17 clocks) plus 8 clocks for input settling
    mode = ADC_CFG_MODE(0) | ADC_CFG_ADSTS(3);
  } else if (bits == 0x88)  {
    // 8 bit fast conversion
    mode = ADC_CFG_MODE(0) | ADC_CFG_ADSTS(0);
  } else if (bits == 10) {
    // 10 bit conversion (17 clocks) plus 20 clocks for input settling
    mode = ADC_CFG_MODE(1) | ADC_CFG_ADSTS(2) | ADC_CFG_ADLSMP;
  } else {
    // 12 bit conversion (25 clocks) plus 24 clocks for input settling
    mode = ADC_CFG_MODE(2) | ADC_CFG_ADSTS(3) | ADC_CFG_ADLSMP;
  }

  if (adc_num == 0) {
    tmp32  = (ADC1_CFG & (0xFFFFFC00));
    tmp32 |= (ADC1_CFG & (0x03));  // ADICLK
    tmp32 |= (ADC1_CFG & (0xE0));  // ADIV & ADLPC

    tmp32 |= mode;
    ADC1_CFG = tmp32;
  } else {
    tmp32  = (ADC2_CFG & (0xFFFFFC00));
    tmp32 |= (ADC2_CFG & (0x03));  // ADICLK
    tmp32 |= (ADC2_CFG & (0xE0));  // ADIV & ADLPC
    tmp32 |= mode;
    ADC2_CFG = tmp32;
  }
}



  //! Returns the resolution of the ADC_Module.
  /**
      \param adc_num ADC number to query.
      \return the resolution of adc_num ADC.
  */
uint8_t ADCL::getResolution(int8_t adc_num)
{
  // bits 2-3:  00->8 01->10  10->12 
  return  8 + (((adc_num? ADC2_CFG : ADC1_CFG) >> 1) & 0x6); 
}



  //! Returns the maximum value for a measurement: 2^res-1.
  /**
      \param adc_num ADC number to query.
      \return the maximum value of adc_num ADC.
  */
  uint32_t ADCL::getMaxValue(int8_t adc_num)
{
  switch (getResolution(adc_num)) {
    case 8: return 255;
    case 10: return 1023;
    default: return 4095;
  }
}




  //! Sets the conversion speed (changes the ADC clock, ADCK)
  /**
    \param speed can be any from the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED, VERY_HIGH_SPEED,
          ADACK_2_4, ADACK_4_0, ADACK_5_2 or ADACK_6_2.

    VERY_LOW_SPEED is guaranteed to be the lowest possible speed within specs for resolutions less than 16 bits (higher than 1 MHz),
    it's different from LOW_SPEED only for 24, 4 or 2 MHz bus frequency.
    LOW_SPEED is guaranteed to be the lowest possible speed within specs for all resolutions (higher than 2 MHz).
    MED_SPEED is always >= LOW_SPEED and <= HIGH_SPEED.
    HIGH_SPEED_16BITS is guaranteed to be the highest possible speed within specs for all resolutions (lower or eq than 12 MHz).
    HIGH_SPEED is guaranteed to be the highest possible speed within specs for resolutions less than 16 bits (lower or eq than 18 MHz).
    VERY_HIGH_SPEED may be out of specs, it's different from HIGH_SPEED only for 48, 40 or 24 MHz bus frequency.

    Additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
    where the numbers are the frequency of the ADC clock (ADCK) in MHz and are independent on the bus speed.
    This is useful if you are using the Teensy at a very low clock frequency but want faster conversions,
    but if F_BUS<F_ADCK, you can't use VERY_HIGH_SPEED for sampling speed.
      \param adc_num ADC number to change.
  */
  void ADCL::setConversionSpeed(ADC_CONVERSION_SPEED speed, int8_t adc_num)
{
}




  //! Sets the sampling speed
  /** Increase the sampling speed for low impedance sources, decrease it for higher impedance ones.
    \param speed can be any of the ADC_SAMPLING_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED.

    VERY_LOW_SPEED is the lowest possible sampling speed (+24 ADCK).
    LOW_SPEED adds +16 ADCK.
    MED_SPEED adds +10 ADCK.
    HIGH_SPEED adds +6 ADCK.
    VERY_HIGH_SPEED is the highest possible sampling speed (0 ADCK added).
      \param adc_num ADC number to change.
  */
  void ADCL::setSamplingSpeed(ADC_SAMPLING_SPEED speed, int8_t adc_num)
{
}




  //! Set the number of averages
  /*!
    \param num can be 0, 4, 8, 16 or 32.
      \param adc_num ADC number to change.
  */
  void ADCL::setAveraging(uint8_t num, int8_t adc_num)
{
  uint32_t mode, mode1;

  //disable averaging, ADC1 and ADC2
  if (adc_num == 0 ) {
    ADC1_GC &= ~0x20;
    mode = ADC1_CFG & ~0xC000;
    if (num >= 32) {
      mode |= ADC_CFG_AVGS(3);
    } else if (num >= 16) {
      mode |= ADC_CFG_AVGS(2);
    } else if (num >= 8) {
      mode |= ADC_CFG_AVGS(1);
    } else if (num >= 4) {
      mode |= ADC_CFG_AVGS(0);
    } else {
      mode |= 0;
    }
    ADC1_CFG = mode;
    if (num >= 4) {
      ADC1_GC |= ADC_GC_AVGE;// turns on averaging
    }
  } else {
    ADC2_GC &= ~0x20;
    mode1 = ADC2_CFG & ~0xC000;

    if (num >= 32) {
      mode1 |= ADC_CFG_AVGS(3);
    } else if (num >= 16) {
      mode1 |= ADC_CFG_AVGS(2);
    } else if (num >= 8) {
      mode1 |= ADC_CFG_AVGS(1);
    } else if (num >= 4) {
      mode1 |= ADC_CFG_AVGS(0);
    } else {
      mode1 |= 0;
    }

    ADC2_CFG = mode1;
    if (num >= 4) {
      ADC2_GC |= ADC_GC_AVGE;// turns on averaging
    }
  }
}





    ////////////// INFORMATION ABOUT THE STATE OF THE ADC /////////////////

    //! Is the ADC converting at the moment?
    /**
        \param adc_num ADC number to query
        \return true if yes, false if not.
    */
    bool ADCL::isConverting(int8_t adc_num)
{
  return ((adc_num? ADC2_HS : ADC1_HS) & ADC_HS_COCO0) == 0;
}



    //! Is an ADC conversion ready?
    /** When a value is read this function returns 0 until a new value exists
        So it only makes sense to call it with continuous or non-blocking methods
        \param adc_num ADC number to query
        \return true if yes, false if not.
    */
bool ADCL::isComplete(int8_t adc_num)
{
  return ((adc_num? ADC2_HS : ADC1_HS) & ADC_HS_COCO0) != 0;
}



    //////////////// BLOCKING CONVERSION METHODS //////////////////

    //! Returns the analog value of the pin.
    /** It waits until the value is read and then returns the result.
      If a comparison has been set up and fails, it will return ADC_ERROR_VALUE.
      This function is interrupt safe, so it will restore the adc to the state it was before being called
      If more than one ADC exists, it will select the module with less workload, you can force a selection using
      adc_num. If you select ADC1 in Teensy 3.0 it will return ADC_ERROR_VALUE.
        \param pin can be any of the analog pins
        \param adc_num ADC_X ADC module
        \return the value of the pin.
    */
int ADCL::analogRead(uint8_t pin, int8_t adc_num)
{
  if (pin > sizeof(t4_pin_to_channel)) return ADC_ERROR_VALUE;
  // I believe all the calibration should be done by now
  uint8_t ch = t4_pin_to_channel[pin];
  switch (adc_num) {
    case 0:
      if (ch & 0x80) return ADC_ERROR_VALUE;  // does not handle this pin
      break;
    case 1:
      if (ch & 0x40) return ADC_ERROR_VALUE;  // does not handle this pin
      break;
    default:
      adc_num =  (ch & 0x80) ? 1 : 0;
  }
  if (adc_num == 0) {
    ADC1_HC0 = ch & 0x3f;
    while (!(ADC1_HS & ADC_HS_COCO0)) ; // wait
    return ADC1_R0;
  } else {
    ADC2_HC0 = ch & 0x3f;
    while (!(ADC2_HS & ADC_HS_COCO0)) ; // wait
    return ADC2_R0;
  }
}



    //! Returns the analog value of the special internal source, such as the temperature sensor.
    /** It calls analogRead(uint8_t pin) internally, with the correct value for the pin for all boards.
        Possible values:
        TEMP_SENSOR,  Temperature sensor.
        VREF_OUT,  1.2 V reference (switch on first using VREF.h).
        BANDGAP, BANDGAP (switch on first using VREF.h).
        VREFH, High VREF.
        VREFL, Low VREF.
        \param pin ADC_INTERNAL_SOURCE to read.
        \param adc_num ADC_X ADC module
        \return the value of the pin.
    */
 
    /////////////// NON-BLOCKING CONVERSION METHODS //////////////

    //! Starts an analog measurement on the pin and enables interrupts.
    /** It returns immediately, get value with readSingle().
        If this function interrupts a measurement, it stores the settings in adc_config
        \param pin can be any of the analog pins
        \param adc_num ADC_X ADC module
        \return true if the pin is valid, false otherwise.
    */
bool ADCL::startSingleRead(uint8_t pin, int8_t adc_num)
{
  // warning not checking to see if already busy or ...
  if (pin > sizeof(t4_pin_to_channel)) return false;
  // I believe all the calibration should be done by now
  uint8_t ch = t4_pin_to_channel[pin];
  switch (adc_num) {
    case 0:
      if (ch & 0x80) return false;  // does not handle this pin
      break;
    case 1:
      if (ch & 0x40) return false;  // does not handle this pin
      break;
    default:
      return false;
  }
  if (adc_num == 0) ADC1_HC0 = ch & 0x3f;
  else ADC2_HC0 = ch & 0x3f;
  return true;
}





    //! Reads the analog value of a single conversion.
    /** Set the conversion with with startSingleRead(pin) or startSingleDifferential(pinP, pinN).
        \param adc_num ADC_X ADC module
        \return the converted value.
    */
int ADCL::readSingle(int8_t adc_num)
{
  return adc_num? ADC2_R0 : ADC1_R0;
}





/////////// SYNCHRONIZED METHODS ///////////////
///// ONLY FOR BOARDS WITH MORE THAN ONE ADC /////
#if ADC_NUM_ADCS>1

//////////////// SYNCHRONIZED BLOCKING METHODS //////////////////

//! Returns the analog values of both pins, measured at the same time by the two ADC modules.
/** It waits until the values are read and then returns the result as a struct Sync_result,
    use Sync_result.result_adc0 and Sync_result.result_adc1.
    If a comparison has been set up and fails, it will return ADC_ERROR_VALUE in both fields of the struct.
    This function is interrupt safe, so it will restore the adc to the state it was before being called
    \param pin0 pin in ADC0
    \param pin1 pin in ADC1
    \return a Sync_result struct with the result of each ADC value.
*/
Sync_result ADCL::analogSynchronizedRead(uint8_t pin0, uint8_t pin1) {
    Sync_result sr = {ADC_ERROR_VALUE, ADC_ERROR_VALUE};
    if (pin0 > sizeof(t4_pin_to_channel)) return sr;
    uint8_t ch_adc1 = t4_pin_to_channel[pin0];
    if (ch_adc1 & 0x80) return sr; // only valid on ADC2

    if (pin1 > sizeof(t4_pin_to_channel)) return sr;
    uint8_t ch_adc2 = t4_pin_to_channel[pin1];
    if (ch_adc2 & 0x40) return sr; // only valid on ADC1

    // Now lets start the conversions...
    ADC1_HC0 = ch_adc1 & 0x3f;
    ADC2_HC0 = ch_adc2 & 0x3f;

    // now wait for both to complete
    while (!(ADC1_HS & ADC_HS_COCO0) || !(ADC2_HS & ADC_HS_COCO0)) ; // wait

    sr.result_adc0 = ADC1_R0;
    sr.result_adc1 = ADC2_R0;
    return sr;

}

    /////////////// SYNCHRONIZED NON-BLOCKING METHODS //////////////

    //! Starts an analog measurement at the same time on the two ADC modules
    /** It returns immediately, get value with readSynchronizedSingle().
        If this function interrupts a measurement, it stores the settings in adc_config
        \param pin0 pin in ADC0
        \param pin1 pin in ADC1
        \return true if the pins are valid, false otherwise.
    */
bool ADCL::startSynchronizedSingleRead(uint8_t pin0, uint8_t pin1)
{
    if (pin0 > sizeof(t4_pin_to_channel)) return false;
    uint8_t ch_adc1 = t4_pin_to_channel[pin0];
    if (ch_adc1 & 0x80) return false; // only valid on ADC2

    if (pin1 > sizeof(t4_pin_to_channel)) return false;
    uint8_t ch_adc2 = t4_pin_to_channel[pin1];
    if (ch_adc2 & 0x40) return false; // only valid on ADC1

    // Now lets start the conversions...
    ADC1_HC0 = ch_adc1 & 0x3f;
    ADC2_HC0 = ch_adc2 & 0x3f;
    return true;
}

    //! Start a differential conversion between two pins (pin0P - pin0N) and (pin1P - pin1N)
    /** It returns immediately, get value with readSynchronizedSingle().
        If this function interrupts a measurement, it stores the settings in adc_config
        \param pin0P positive pin in ADC0
        \param pin0N negative pin in ADC0
        \param pin1P positive pin in ADC1
        \param pin1N negative pin in ADC1
        \return true if the pins are valid, false otherwise.
    */
//    bool startSynchronizedSingleDifferential(uint8_t pin0P, uint8_t pin0N, uint8_t pin1P, uint8_t pin1N);

    //! Reads the analog value of a single conversion.
    /**
        \return the converted value.
    */
Sync_result ADCL::readSynchronizedSingle() 
{
    Sync_result sr = {ADC_ERROR_VALUE, ADC_ERROR_VALUE};

    sr.result_adc0 = ADC1_R0;
    sr.result_adc1 = ADC2_R0;
    return sr;
}


    ///////////// SYNCHRONIZED CONTINUOUS CONVERSION METHODS ////////////

    //! Starts a continuous conversion in both ADCs simultaneously
    /** Use readSynchronizedContinuous to get the values
        \param pin0 pin in ADC0
        \param pin1 pin in ADC1
        \return true if the pins are valid, false otherwise.
    */
 //   bool startSynchronizedContinuous(uint8_t pin0, uint8_t pin1);

    //! Starts a continuous differential conversion in both ADCs simultaneously
    /** Use readSynchronizedContinuous to get the values
        \param pin0P positive pin in ADC0
        \param pin0N negative pin in ADC0
        \param pin1P positive pin in ADC1
        \param pin1N negative pin in ADC1
        \return true if the pins are valid, false otherwise.
    */
  //  bool startSynchronizedContinuousDifferential(uint8_t pin0P, uint8_t pin0N, uint8_t pin1P, uint8_t pin1N);

    //! Returns the values of both ADCs.
    /**
        \return the converted value.
    */
  //Sync_result ADCL::readSynchronizedContinuous();

    //! Stops synchronous continuous conversion
 //   void ADCL::stopSynchronizedContinuous();

#endif
