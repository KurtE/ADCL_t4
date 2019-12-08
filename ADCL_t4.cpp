#include <Arduino.h>
#include "ADCL_t4.h"
// modified version from T4 analog.c
// 0x80 bit - Only on ADC2
// 0x40 bit - only on ADC1
const uint8_t  ADCL::t4_pin_to_channel[] = {
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

//analog compare ACMP3 and ACMP4 available on pins 26 and 27
uint8_t acmp3_inp_pins[] = {18, 17, 255, 255, 255, 23, 255};
uint8_t acmp4_inp_pins[] = {18, 17, 255, 255, 255, 20, 26 };

//==========================================================================================
//class ADCL
//==========================================================================================
uint8_t ADCL::mapPinToChannel(uint8_t pin, int8_t adc_num) 
{
  if (pin > sizeof(t4_pin_to_channel)) return 0xff;
  uint8_t ch = t4_pin_to_channel[pin];

  if (!(ch & 0x3f) || (adc_num == -1)) return ch; // simply return raw...

  if (adc_num == 0) {
    if (ch & 0x80) return 0xff;
  } else if (ch & 0x40) return 0xff;
  return ch & 0x3f;
}

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
  if (adc_num == 0) 
    adc0->setResolution(bits);
  else 
    adc1->setResolution(bits);
}



  //! Returns the resolution of the ADC_Module.
  /**
      \param adc_num ADC number to query.
      \return the resolution of adc_num ADC.
  */
uint8_t ADCL::getResolution(int8_t adc_num)
{
  if (adc_num == 0) 
    return adc0->getResolution();
  else 
    return adc1->getResolution();
}



  //! Returns the maximum value for a measurement: 2^res-1.
  /**
      \param adc_num ADC number to query.
      \return the maximum value of adc_num ADC.
  */
  uint32_t ADCL::getMaxValue(int8_t adc_num)
{
  if (adc_num == 0) 
    return adc0->getMaxValue();
  else 
    return adc1->getMaxValue();
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
  if (adc_num == 0) 
    adc0->setConversionSpeed(speed);
  else 
    adc1->setConversionSpeed(speed);
}

void ADCL::setAdcClockSpeed(ADC_CONVERSION_SPEED speed1)
{
	
	uint32_t adc_cfg_div;
	uint8_t div, clk;
	
	uint8_t ADIV, ACLK;
	unsigned  mask;
	//ACLK
	mask = ((1 << 2) - 1) << 0;
	ACLK = ADC1_CFG & mask;
	//DIV
	mask = ((1 << 2) - 1) << 5;
	ADIV = (ADC1_CFG & mask) >> 5;
	
	if(speed1==clock_speed) { // no change
		return;
	}
	//Serial.printf("Speed: %d, %d\n",clock_speed, speed1);
	//if (calibrating) wait_for_cal();

	// internal asynchronous clock settings: fADK = 2.4, 4.0, 5.2 or 6.2 MHz
	if( (speed1 == ADC_CONVERSION_SPEED::ADACK_1_25_N) ||
		(speed1 == ADC_CONVERSION_SPEED::ADACK_2_5_N) ||
		(speed1 == ADC_CONVERSION_SPEED::ADACK_5_0_N) ||
		(speed1 == ADC_CONVERSION_SPEED::ADACK_10_0_N)) {
		adc_cfg_div = ADC_CFG_ADICLK(3);
		clk = 3;
		if(speed1 == ADC_CONVERSION_SPEED::ADACK_1_25_N) {
			adc_cfg_div = ADC_CFG_ADIV(3) | adc_cfg_div;
			div = 3;
		} else if(speed1 == ADC_CONVERSION_SPEED::ADACK_2_5_N) {
			adc_cfg_div = ADC_CFG_ADIV(2) | adc_cfg_div;
			div = 2;
		} else if(speed1 == ADC_CONVERSION_SPEED::ADACK_5_0_N) {
			adc_cfg_div = ADC_CFG_ADIV(1) | adc_cfg_div;
			div = 1;
		} else if(speed1 == ADC_CONVERSION_SPEED::ADACK_10_0_N) {
			adc_cfg_div = ADC_CFG_ADIV(0) | adc_cfg_div;
			div = 0;
		}
		//ADC1
		ADC1_CFG = (ADC1_CFG & ~ADC_CFG_ADIV(div) & ~ADC_CFG_ADICLK(clk) & ~ADC_CFG_ADHSC) | adc_cfg_div;
	//need cal?
		ADC2_CFG = (ADC2_CFG & ~ADC_CFG_ADIV(div) & ~ADC_CFG_ADICLK(clk) & ~ADC_CFG_ADHSC) | adc_cfg_div;
	//need cal?
		clock_speed = speed1;
		return;
	}

	//ADHSC is set by default in analog_init so only have to deal with clock 
	//and divider
	if( (speed1 == ADC_CONVERSION_SPEED::ADACK_2_5_H) ||
		(speed1 == ADC_CONVERSION_SPEED::ADACK_5_0_H) ||
		(speed1 == ADC_CONVERSION_SPEED::ADACK_10_0_H) ||
		(speed1 == ADC_CONVERSION_SPEED::ADACK_20_0_H)) {
		adc_cfg_div = ADC_CFG_ADICLK(3);
		clk = 3;
		if(speed1 == ADC_CONVERSION_SPEED::ADACK_2_5_H) {
			adc_cfg_div = ADC_CFG_ADIV(3) | adc_cfg_div;
			div = 3;
		} else if(speed1 == ADC_CONVERSION_SPEED::ADACK_5_0_H) {
			adc_cfg_div = ADC_CFG_ADIV(2) | adc_cfg_div;
			div = 2;
		} else if(speed1 == ADC_CONVERSION_SPEED::ADACK_10_0_H) {
			adc_cfg_div = ADC_CFG_ADIV(1) | adc_cfg_div;
			div = 1;
		} else if(speed1 == ADC_CONVERSION_SPEED::ADACK_20_0_H) {
			adc_cfg_div = ADC_CFG_ADIV(0) | adc_cfg_div;
			div = 0;
		}
		//ADC1
		ADC1_CFG = (ADC1_CFG & ~ADC_CFG_ADIV(div) & ~ADC_CFG_ADICLK(clk)) | adc_cfg_div;
//need cal?
		ADC2_CFG = (ADC2_CFG & ~ADC_CFG_ADIV(div) & ~ADC_CFG_ADICLK(clk)) | adc_cfg_div;
//need cal?
		clock_speed = speed1;
		return;
	}
	
	switch (speed1) {
	case ADC_CONVERSION_SPEED::VERY_LOW_SPEED:    /*!< is guaranteed to be the lowest possible speed within specs for resolutions less than 16 bits (higher than 1 MHz). */
	  adc_cfg_div = ADC_CFG_ADIV(3) | ADC_CFG_ADICLK(1); // use IPG/16
	  div = 3; clk = 1;
	  break;
	case ADC_CONVERSION_SPEED::LOW_SPEED:     /*!< is guaranteed to be the lowest possible speed within specs for all resolutions (higher than 2 MHz). */
	  adc_cfg_div = ADC_CFG_ADIV(2) | ADC_CFG_ADICLK(1); // use IPG/8
	  div = 2; clk = 1;
	  break;
	case ADC_CONVERSION_SPEED::MED_SPEED:     /*!< is always >= LOW_SPEED and <= HIGH_SPEED. */
	  adc_cfg_div = ADC_CFG_ADIV(1) | ADC_CFG_ADICLK(1); // use IPG/4
	  div = 1; clk = 1;
	  break;
	case ADC_CONVERSION_SPEED::HIGH_SPEED:    /*!< is guaranteed to be the highest possible speed within specs for resolutions less than 16 bits (lower */
	  adc_cfg_div = ADC_CFG_ADIV(0) | ADC_CFG_ADICLK(1); // use IPG/2
	  div = 0; clk = 1;
	  break;
	case ADC_CONVERSION_SPEED::VERY_HIGH_SPEED:    // this speed is most likely out of specs, so accuracy can be bad
	  adc_cfg_div = ADC_CFG_ADIV(0) | ADC_CFG_ADICLK(0); // use IPG
	  div = 0; clk = 0;
	  break;
	default:   // low speed
	  adc_cfg_div = ADC_CFG_ADIV(2) | ADC_CFG_ADICLK(1); // use IPG/8
	  div = 2; clk = 1;
	  break;

	}
  
  	//Serial.printf("DIV: %d, CLK: %d (%08x %08x)\n", div, clk, ADC1_CFG, ADC1_GC);

	//ADC1
	ADC1_CFG = (ADC1_CFG & (~ADC_CFG_ADIV(ADIV)) & (~ADC_CFG_ADICLK(ACLK))) | adc_cfg_div;
	//calibrating = 1;
	while (ADC1_GC & ADC_GC_CAL) ;
	//calibrating = 0;	
	ADC2_CFG = (ADC2_CFG & (~ADC_CFG_ADIV(ADIV)) & (~ADC_CFG_ADICLK(ACLK))) | adc_cfg_div;
	//calibrating = 1;
	while (ADC2_GC & ADC_GC_CAL) ;
	//calibrating = 0;
	
  	//Serial.printf("DIV: %d, CLK: %d (%08x %08x)\n", div, clk, ADC1_CFG, ADC1_GC);
	clock_speed = speed1;

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
  if (adc_num == 0) 
    adc0->setSamplingSpeed(speed);
  else 
    adc1->setSamplingSpeed(speed);
}





  //! Set the number of averages
  /*!
    \param num can be 0, 4, 8, 16 or 32.
      \param adc_num ADC number to change.
  */
void ADCL::setAveraging(uint8_t num, int8_t adc_num)
{
  if (adc_num == 0) 
    adc0->setAveraging(num);
  else 
    adc1->setAveraging(num);
}

// Enable interrupts
/* An IRQ_ADC0 Interrupt will be raised when the conversion is completed
*  (including hardware averages and if the comparison (if any) is true).
*/
void ADCL::enableInterrupts(int8_t adc_num) 
{
  if (adc_num == 0) 
    adc0->enableInterrupts();
  else 
    adc1->enableInterrupts();
}

// Disable interrupts
void ADCL::disableInterrupts(int8_t adc_num) 
{
  if (adc_num == 0) 
    adc0->disableInterrupts();
  else 
    adc1->disableInterrupts();
}



//! Enable DMA request
/** An ADC DMA request will be raised when the conversion is completed
*  (including hardware averages and if the comparison (if any) is true).
*   \param adc_num ADC number to change.
*/
void ADCL::enableDMA(int8_t adc_num) 
{
  if (adc_num == 0) 
    adc0->enableDMA();
  else 
    adc1->enableDMA();
}

//! Disable ADC DMA request
/**
*   \param adc_num ADC number to change.
*/
void ADCL::disableDMA(int8_t adc_num)
{
  if (adc_num == 0) 
    adc0->disableDMA();
  else 
    adc1->disableDMA();
}





    ////////////// INFORMATION ABOUT THE STATE OF THE ADC /////////////////

    //! Is the ADC converting at the moment?
    /**
        \param adc_num ADC number to query
        \return true if yes, false if not.
    */
bool ADCL::isConverting(int8_t adc_num)
{
  if (adc_num == 0) 
    return adc0->isConverting();
  else 
    return adc1->isConverting();
}



    //! Is an ADC conversion ready?
    /** When a value is read this function returns 0 until a new value exists
        So it only makes sense to call it with continuous or non-blocking methods
        \param adc_num ADC number to query
        \return true if yes, false if not.
    */
bool ADCL::isComplete(int8_t adc_num)
{
  if (adc_num == 0) 
    return adc0->isComplete();
  else 
    return adc1->isComplete();
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
  if (adc_num == 0) 
    return adc0->analogRead(pin);
  else 
    return adc1->analogRead(pin);
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
  if (adc_num == 0) 
    return adc0->startSingleRead(pin);
  else 
    return adc1->startSingleRead(pin);
}





    //! Reads the analog value of a single conversion.
    /** Set the conversion with with startSingleRead(pin) or startSingleDifferential(pinP, pinN).
        \param adc_num ADC_X ADC module
        \return the converted value.
    */
int ADCL::readSingle(int8_t adc_num)
{
  if (adc_num == 0) 
    return adc0->readSingle();
  else 
    return adc1->readSingle();
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
 
 
//! enable the compare function
int ADCL::enableCompare(uint8_t acmp_pin, uint8_t input_pin)
{	
	uint8_t INx = 255;
	
	if(acmp_pin == ACMP3){
		for(uint8_t i = 0; i < sizeof(acmp3_inp_pins); i++){
			if(input_pin == acmp3_inp_pins[i]){
				INx = i;
			}
		}
		if (INx == 255)  return ADC_ERROR_VALUE;
		
		CCM_CCGR3 |= CCM_CCGR3_ACMP3(CCM_CCGR_ON);  // ACMP on
		CMP3_MUXCR = CMP_MUXCR_PSEL(INx) | CMP_MUXCR_MSEL(7);
		CMP3_CR1 = CMP_CR1_ENABLE ;   // enable
		CMP3_DACCR = DACCR_ENABLE  | 32 | 0x40;  // 3v3/2 and enable
		// ? do i need to configure DAC pin to see output?  output ACMP result HIGH or LOW
		pinMode(ACMP3, OUTPUT);
		IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14 = 1;  // ALT 1 ACMP3_OUT
	} else if(acmp_pin == ACMP4){
		for(uint8_t i = 0; i < sizeof(acmp4_inp_pins); i++){
			if(input_pin == acmp4_inp_pins[i]){
				INx = i;
			}
		}
		if (INx == 255)  return ADC_ERROR_VALUE;
		
		CCM_CCGR3 |= CCM_CCGR3_ACMP4(CCM_CCGR_ON);  // ACMP on
		CMP4_MUXCR = CMP_MUXCR_PSEL(INx) | CMP_MUXCR_MSEL(7);
		CMP4_CR1 = CMP_CR1_ENABLE ;   // enable
		CMP4_DACCR = DACCR_ENABLE  | 32 | 0x40;  // 3v3/2 and enable
		// ? do i need to configure DAC pin to see output?  output ACMP result HIGH or LOW
		pinMode(ACMP4, OUTPUT);
		IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15 = 1;  // ALT 1 ACMP3_OUT 
	} else {
		return ADC_ERROR_VALUE;
	}
	
	return 0;
}
			
//! Disable the compare function
void ADCL::disableCompare(uint8_t acmp_pin)
{
	if(acmp_pin == ACMP3){
		CCM_CCGR3 |= CCM_CCGR3_ACMP3(CCM_CCGR_OFF);  // ACMP on
		CMP3_CR1 = CMP_CR1_DISABLE ;
	} else if(acmp_pin == ACMP4){
		CCM_CCGR3 |= CCM_CCGR3_ACMP3(CCM_CCGR_OFF);  // ACMP on
		CMP4_CR1 = CMP_CR1_DISABLE ;
	}
}

//! Enable the compare function to a single value
/** A conversion will be completed only when the ADC value
*  is >= compValue (greaterThan=1) or < compValue (greaterThan=0)
*  Call it after changing the resolution
*  Use with interrupts or poll conversion completion with isComplete()
*	1.  compare true if the result is less than the value1. (greaterThan=0. mode0)
*	2.  compare true if the result is greater than or equal to value1.(greaterThan=1. mode1)
*   Setting value1(0-4095) for hardware compare mode. 
*   \param compValue value to compare
*   \param greaterThan or equal to true or false
*/
void ADCL::enableCompareValue(int8_t adc_num, int16_t compValue, bool greaterThan)
{
    uint32_t tmp32;

	if(adc_num == 0){
		tmp32 = ADC1_GC & ~(ADC_GC_ACFE_MASK | ADC_GC_ACFGT_MASK | 		ADC_GC_ACREN_MASK);
		/* Enable the feature. */
		tmp32 |= ADC_GC_ACFE_MASK;
		
		if(greaterThan == true)
				tmp32 |= ADC_GC_ACFGT_MASK;
				
		ADC1_GC = tmp32;

		/* Load the compare values. */
		tmp32 = ADC_CV_CV1(compValue) | ADC_CV_CV2(compValue);
		ADC1_CV = tmp32;	
	} else {
		tmp32 = ADC2_GC & ~(ADC_GC_ACFE_MASK | ADC_GC_ACFGT_MASK | 		ADC_GC_ACREN_MASK);
		/* Enable the feature. */
		tmp32 |= ADC_GC_ACFE_MASK;
		
		if(greaterThan == true)
				tmp32 |= ADC_GC_ACFGT_MASK;
				
		ADC2_GC = tmp32;

		/* Load the compare values. */
		tmp32 = ADC_CV_CV1(compValue) | ADC_CV_CV2(compValue);
		ADC2_CV = tmp32;	
	}

}


//! Enable the compare function to a range
/** A conversion will be completed only when the ADC value is inside (insideRange=1) or outside (=0)
*  the range given by (lowerLimit, upperLimit),including (inclusive=1) the limits or not (inclusive=0).
*
*  1.  Value1 <= Value2, compare true if the result is less than value1 OR the result is Greater than value2. (inclusive = 0, insideRange = 0, mode2)
*  2.  Value1 >  Value2, compare true if the result is less than value1 AND the result is Greater than value2. (inclusive = 0, insideRange = 1, mode3)
*  3.  Value1 <= Value2, compare true if the result is greater than or equal to value1 AND the result is less than or equal to value2.   (inclusive = 1, insideRange = 1, mode4)
*  4.  Value1 >  Value2, compare true if the result is greater than or equal to value1 OR the result is less than or equal to value2.   (inclusive = 1, insideRange = 0, mode5)
*	
*  Call it after changing the resolution
*  Use with interrupts or poll conversion completion with isComplete()
*   \param lowerLimit lower value to compare
*   \param upperLimit upper value to compare
*   \param insideRange true or false
*   \param inclusive true or false
*/
void ADCL::enableCompareRange(int8_t adc_num, int16_t lowerLimit, int16_t upperLimit, bool insideRange, bool inclusive)
{
	uint8_t mode = 0;
	uint32_t tmp32;
	
	if(insideRange == 0 && inclusive == 0) mode = 2;
	if(insideRange == 1 && inclusive == 0) mode = 3;
	if(insideRange == 1 && inclusive == 1) mode = 4;
	if(insideRange == 0 && inclusive == 1) mode = 5;
	
	if(adc_num == 0){
		tmp32 = ADC1_GC & ~(ADC_GC_ACFE_MASK | ADC_GC_ACFGT_MASK | ADC_GC_ACREN_MASK);

		/* Enable the feature. */
		tmp32 |= ADC_GC_ACFE_MASK;

		/* Select the hardware compare working mode. */
		switch (mode)
		{
			case 0:
				break;
			case 1:
				break;
			case 2:
				tmp32 |= ADC_GC_ACREN_MASK;
				break;
			case 3:
				tmp32 |= ADC_GC_ACFGT_MASK | ADC_GC_ACREN_MASK;
				break;
			default:
				break;
		}
		ADC1_GC = tmp32;

		/* Load the compare values. */
		tmp32 = ADC_CV_CV1(lowerLimit) | ADC_CV_CV2(upperLimit);
		ADC1_CV = tmp32;
	} else {
		tmp32 = ADC2_GC & ~(ADC_GC_ACFE_MASK | ADC_GC_ACFGT_MASK | ADC_GC_ACREN_MASK);

		/* Enable the feature. */
		tmp32 |= ADC_GC_ACFE_MASK;

		/* Select the hardware compare working mode. */
		switch (mode)
		{
			case 0:
				break;
			case 1:
				break;
			case 2:
				tmp32 |= ADC_GC_ACREN_MASK;
				break;
			case 3:
				tmp32 |= ADC_GC_ACFGT_MASK | ADC_GC_ACREN_MASK;
				break;
			default:
				break;
		}
		ADC2_GC = tmp32;

		/* Load the compare values. */
		tmp32 = ADC_CV_CV1(lowerLimit) | ADC_CV_CV2(upperLimit);
		ADC2_CV = tmp32;
	}
	
}

int ADCL::getAdcCompareRes(uint8_t acmp_pin)
{
	if(acmp_pin == ACMP3){
		return CMP3_SCR & CMP_SCR_COUT;
	} else if(acmp_pin == ACMP4){
		return CMP4_SCR & CMP_SCR_COUT;
	}
	return -1;
}

#endif
