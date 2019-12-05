#ifndef ADCL_T4_H
#define ADCL_T4_H

#if 0
extern int T4AnalogRead(uint8_t pin, uint8_t adc_num = 0);
extern bool T4AnalogReadBoth(uint8_t adc1_pin, uint16_t &adc1_val, uint8_t adc2_pin, uint16_t &adc2_val);

extern bool T4AnalogReadStart(uint8_t pin, uint8_t adc_num);
extern int T4AnalogReadComplete(uint8_t adc_num, bool wait_complete = false);

extern void T4AnalogReadRes(unsigned int bits, uint8_t adc_num);
extern void T4AnalogReadAveraging(unsigned int num, uint8_t adc_num);
#endif
//-------------------------------------------
// Extract from ADC library
//-------------------------------------------


#define ADC_0 0
#define ADC_1 1

//----------------------------
// From module file
//---------------------------
// Easier names for the boards

#define ADC_NUM_ADCS (2)
#define ADC_USE_DMA (1)
#define ADC_USE_PGA (0)
#define ADC_USE_PDB (0)
#define ADC_USE_INTERNAL_VREF (1)

enum class ADC_REF_SOURCE : uint8_t {REF_DEFAULT = 0, REF_ALT = 1, REF_NONE = 2}; // internal, do not use
enum class ADC_REFERENCE : uint8_t {
    REF_3V3 = ADC_REF_SOURCE::REF_DEFAULT, /*!< 3.3 volts */
    NONE = ADC_REF_SOURCE::REF_NONE // internal, do not use
};

#define ADC_MAX_PIN (27)
#define ADC_DIFF_PAIRS (0)

enum class ADC_INTERNAL_SOURCE : uint8_t{
};

//! Struct for synchronous measurements
/** result_adc0 has the result from ADC0 and result_adc1 from ADC1.
*/
struct Sync_result {
  int32_t result_adc0, result_adc1;
};


// Settings for the power/speed of conversions/sampling
/*! ADC conversion speed.
*   Common set of options to select the ADC clock speed F_ADCK, which depends on F_BUS, except for the ADACK_X_Y options that are independent.
*   This selection affects the sampling speed too.
*   Note: the F_ADCK speed is not equal to the conversion speed; any measurement takes several F_ADCK cycles to complete including the sampling and conversion steps.
*/
enum class ADC_CONVERSION_SPEED : uint8_t {
    VERY_LOW_SPEED, /*!< is guaranteed to be the lowest possible speed within specs for resolutions less than 16 bits (higher than 1 MHz). */
    LOW_SPEED, /*!< is guaranteed to be the lowest possible speed within specs for all resolutions (higher than 2 MHz). */
    MED_SPEED, /*!< is always >= LOW_SPEED and <= HIGH_SPEED. */
    HIGH_SPEED_16BITS, /*!< is guaranteed to be the highest possible speed within specs for all resolutions (lower than or equal to 12 MHz). */
    HIGH_SPEED, /*!< is guaranteed to be the highest possible speed within specs for resolutions less than 16 bits (lower than or equal to 18 MHz),
                            except for Teensy 3.6 (NOT 3.5), for which the maximum is 24 MHz. */
    VERY_HIGH_SPEED, /*!< may be out of specs */

};
/*! ADC sampling speed.
*   It selects how many ADCK clock cycles to add.
*/
enum class ADC_SAMPLING_SPEED : uint8_t {
    VERY_LOW_SPEED, /*!< is the lowest possible sampling speed (+24 ADCK). */
    LOW_SPEED, /*!< adds +16 ADCK. */
    MED_SPEED, /*!< adds +10 ADCK. */
    HIGH_SPEED, /*!< adds +6 ADCK. */
    VERY_HIGH_SPEED, /*!< is the highest possible sampling speed (0 ADCK added). */
};

//! Handle ADC errors
#define ADC_ERROR_DIFF_VALUE (-70000)
#define ADC_ERROR_VALUE ADC_ERROR_DIFF_VALUE
namespace ADC_Error {

    //! ADC errors.
    /*! Use adc->printError() to print the errors (if any) in a human-readable form.
    *   Use adc->resetError() to reset them.
    */
    enum class ADC_ERROR : uint16_t {
        OTHER               = 1<<0, /*!< Other error not considered below. */
        CALIB               = 1<<1, /*!< Calibration error. */
        WRONG_PIN           = 1<<2, /*!< A pin was selected that cannot be read by this ADC module. */
        ANALOG_READ         = 1<<3, /*!< Error inside the analogRead method. */
        ANALOG_DIFF_READ    = 1<<4, /*!< Error inside the analogReadDifferential method. */
        CONT                = 1<<5, /*!< Continuous single-ended measurement error. */
        CONT_DIFF           = 1<<6, /*!< Continuous differential measurement error. */
        COMPARISON          = 1<<7, /*!< Error during the comparison. */
        WRONG_ADC           = 1<<8, /*!< A non-existent ADC module was selected. */
        SYNCH               = 1<<9, /*!< Error during a synchronized measurement. */

        CLEAR               = 0,    /*!< No error. */
    };
    //! OR operator for ADC_ERRORs.
    inline constexpr ADC_ERROR operator|(ADC_ERROR lhs, ADC_ERROR rhs) {
        return static_cast<ADC_ERROR> (static_cast<uint16_t>(lhs) | static_cast<uint16_t>(rhs));
    }
    //! AND operator for ADC_ERRORs.
    inline constexpr ADC_ERROR operator&(ADC_ERROR lhs, ADC_ERROR rhs) {
        return static_cast<ADC_ERROR> (static_cast<uint16_t>(lhs) & static_cast<uint16_t>(rhs));
    }
    //! |= operator for ADC_ERRORs, it changes the left hand side ADC_ERROR.
    inline ADC_ERROR operator|=(volatile ADC_ERROR& lhs, ADC_ERROR rhs) {
        return lhs = static_cast<ADC_ERROR> (static_cast<uint16_t>(lhs) | static_cast<uint16_t>(rhs));
    }
    //! &= operator for ADC_ERRORs, it changes the left hand side ADC_ERROR.
    inline ADC_ERROR operator&=(volatile ADC_ERROR& lhs, ADC_ERROR rhs) {
        return lhs = static_cast<ADC_ERROR> (static_cast<uint16_t>(lhs) & static_cast<uint16_t>(rhs));
    }

    //! Prints the human-readable error, if any.
    inline void printError(ADC_ERROR fail_flag, uint8_t ADC_num = 0) {
        if(fail_flag != ADC_ERROR::CLEAR) {
            Serial.print("ADC"); Serial.print(ADC_num);
            Serial.print(" error: ");
            switch(fail_flag) {
                case ADC_ERROR::CALIB:
                    Serial.print("Calibration");
                    break;
                case ADC_ERROR::WRONG_PIN:
                    Serial.print("Wrong pin");
                    break;
                case ADC_ERROR::ANALOG_READ:
                    Serial.print("Analog read");
                    break;
                case ADC_ERROR::COMPARISON:
                    Serial.print("Comparison");
                    break;
                case ADC_ERROR::ANALOG_DIFF_READ:
                    Serial.print("Analog differential read");
                    break;
                case ADC_ERROR::CONT:
                    Serial.print("Continuous read");
                    break;
                case ADC_ERROR::CONT_DIFF:
                    Serial.print("Continuous differential read");
                    break;
                case ADC_ERROR::WRONG_ADC:
                    Serial.print("Wrong ADC");
                    break;
                case ADC_ERROR::SYNCH:
                    Serial.print("Synchronous");
                    break;
                case ADC_ERROR::OTHER:
                case ADC_ERROR::CLEAR: // silence warnings
                default:
                    Serial.print("Unknown");
                    break;
            }
            Serial.println(" error.");
        }
    }

    //! Resets all errors from the ADC, if any.
    inline void resetError(volatile ADC_ERROR& fail_flag) {
        fail_flag = ADC_ERROR::CLEAR;
    }

}
using ADC_Error::ADC_ERROR;


// debug mode: blink the led light
#define ADC_debug 0




/** Class ADC: Controls the Teensy 3.x ADC

*/
class ADCL
{
  protected:
  private:


  public:

    /** Default constructor */
    ADCL() {};



    /////////////// METHODS TO SET/GET SETTINGS OF THE ADC ////////////////////

    //! Set the voltage reference you prefer, default is vcc
    /*! It recalibrates at the end.
        \param type can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 (not for Teensy LC) or ADC_REFERENCE::REF_EXT
        \param adc_num ADC number to change.
    */
    void setReference(ADC_REFERENCE type, int8_t adc_num = -1);


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
    void setResolution(uint8_t bits, int8_t adc_num = -1);

    //! Returns the resolution of the ADC_Module.
    /**
        \param adc_num ADC number to query.
        \return the resolution of adc_num ADC.
    */
    uint8_t getResolution(int8_t adc_num = -1);

    //! Returns the maximum value for a measurement: 2^res-1.
    /**
        \param adc_num ADC number to query.
        \return the maximum value of adc_num ADC.
    */
    uint32_t getMaxValue(int8_t adc_num = -1);


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
    void setConversionSpeed(ADC_CONVERSION_SPEED speed, int8_t adc_num = -1);


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
    void setSamplingSpeed(ADC_SAMPLING_SPEED speed, int8_t adc_num = -1);


    //! Set the number of averages
    /*!
      \param num can be 0, 4, 8, 16 or 32.
        \param adc_num ADC number to change.
    */
    void setAveraging(uint8_t num, int8_t adc_num = -1);



    ////////////// INFORMATION ABOUT THE STATE OF THE ADC /////////////////

    //! Is the ADC converting at the moment?
    /**
        \param adc_num ADC number to query
        \return true if yes, false if not.
    */
    bool isConverting(int8_t adc_num = -1);

    //! Is an ADC conversion ready?
    /** When a value is read this function returns 0 until a new value exists
        So it only makes sense to call it with continuous or non-blocking methods
        \param adc_num ADC number to query
        \return true if yes, false if not.
    */
    bool isComplete(int8_t adc_num = -1);


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
    int analogRead(uint8_t pin, int8_t adc_num = -1);

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
    int analogRead(ADC_INTERNAL_SOURCE pin, int8_t adc_num = -1) __attribute__((always_inline)) {
      return analogRead(static_cast<uint8_t>(pin), adc_num);
    }

    //! Reads the differential analog value of two pins (pinP - pinN).
    /** It waits until the value is read and then returns the result.
      This function is interrupt safe, so it will restore the adc to the state it was before being called
      If more than one ADC exists, it will select the module with less workload, you can force a selection using
      adc_num
        \param pinP must be A10 or A12.
        \param pinN must be A11 (if pinP=A10) or A13 (if pinP=A12).
        \param adc_num ADC_X ADC module
        \return the differential value of the pins, invalid pins return ADC_ERROR_VALUE.
        If a comparison has been set up and fails, it will return ADC_ERROR_VALUE.
    */
  //  int analogReadDifferential(uint8_t pinP, uint8_t pinN, int8_t adc_num = -1);


    /////////////// NON-BLOCKING CONVERSION METHODS //////////////

    //! Starts an analog measurement on the pin and enables interrupts.
    /** It returns immediately, get value with readSingle().
        If this function interrupts a measurement, it stores the settings in adc_config
        \param pin can be any of the analog pins
        \param adc_num ADC_X ADC module
        \return true if the pin is valid, false otherwise.
    */
    bool startSingleRead(uint8_t pin, int8_t adc_num = -1);

    //! Start a differential conversion between two pins (pinP - pinN) and enables interrupts.
    /** It returns immediately, get value with readSingle().
        If this function interrupts a measurement, it stores the settings in adc_config
        \param pinP must be A10 or A12.
        \param pinN must be A11 (if pinP=A10) or A13 (if pinP=A12).
        \param adc_num ADC_X ADC module
        \return true if the pins are valid, false otherwise.
    */
 //   bool startSingleDifferential(uint8_t pinP, uint8_t pinN, int8_t adc_num = -1);

    //! Reads the analog value of a single conversion.
    /** Set the conversion with with startSingleRead(pin) or startSingleDifferential(pinP, pinN).
        \param adc_num ADC_X ADC module
        \return the converted value.
    */
    int readSingle(int8_t adc_num = -1);



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
    Sync_result analogSynchronizedRead(uint8_t pin0, uint8_t pin1);

    //! Same as analogSynchronizedRead
    /**
        \param pin0 pin in ADC0
        \param pin1 pin in ADC1
        \return a Sync_result struct with the result of each ADC value.
    */
    Sync_result analogSyncRead(uint8_t pin0, uint8_t pin1) __attribute__((always_inline)) {
      return analogSynchronizedRead(pin0, pin1);
    }

    //! Returns the differential analog values of both sets of pins, measured at the same time by the two ADC modules.
    /** It waits until the values are read and then returns the result as a struct Sync_result,
        use Sync_result.result_adc0 and Sync_result.result_adc1.
        If a comparison has been set up and fails, it will return ADC_ERROR_VALUE in both fields of the struct.
        This function is interrupt safe, so it will restore the adc to the state it was before being called
        \param pin0P positive pin in ADC0
        \param pin0N negative pin in ADC0
        \param pin1P positive pin in ADC1
        \param pin1N negative pin in ADC1
        \return a Sync_result struct with the result of each differential ADC value.
    */
//    Sync_result analogSynchronizedReadDifferential(uint8_t pin0P, uint8_t pin0N, uint8_t pin1P, uint8_t pin1N);

    //! Same as analogSynchronizedReadDifferential
    /**
        \param pin0P positive pin in ADC0
        \param pin0N negative pin in ADC0
        \param pin1P positive pin in ADC1
        \param pin1N negative pin in ADC1
        \return a Sync_result struct with the result of each differential ADC value.
    */
//    Sync_result analogSyncReadDifferential(uint8_t pin0P, uint8_t pin0N, uint8_t pin1P, uint8_t pin1N) __attribute__((always_inline)) {
//      return analogSynchronizedReadDifferential(pin0P, pin0N, pin1P, pin1N);
 //   }

    /////////////// SYNCHRONIZED NON-BLOCKING METHODS //////////////

    //! Starts an analog measurement at the same time on the two ADC modules
    /** It returns immediately, get value with readSynchronizedSingle().
        If this function interrupts a measurement, it stores the settings in adc_config
        \param pin0 pin in ADC0
        \param pin1 pin in ADC1
        \return true if the pins are valid, false otherwise.
    */
    bool startSynchronizedSingleRead(uint8_t pin0, uint8_t pin1);

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
    Sync_result readSynchronizedSingle();


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
    Sync_result readSynchronizedContinuous();

    //! Stops synchronous continuous conversion
    void stopSynchronizedContinuous();

#endif


};





#endif // ADC_H
