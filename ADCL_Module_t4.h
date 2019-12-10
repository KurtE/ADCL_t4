#ifndef ADCL_T4_MODULE_H
#define ADCL_T4_MODULE_H
//-------------------------------------------
// Extract from ADC library
//-------------------------------------------


#define ADC_0 0
#define ADC_1 1

//----------------------------
// From module file
//---------------------------
// Easier names for the boards
//----------------------------
// Define a class for IMXRT_ADC object
//---------------------------
#include <stdint.h>
#include <Arduino.h>
#ifndef IMXRT_ADC1S
typedef struct {
    volatile uint32_t HC0;
    volatile uint32_t HC1;
    volatile uint32_t HC2;
    volatile uint32_t HC3;
    volatile uint32_t HC4;
    volatile uint32_t HC5;
    volatile uint32_t HC6;
    volatile uint32_t HC7;
    volatile uint32_t HS; 
    volatile uint32_t R0; 
    volatile uint32_t R1; 
    volatile uint32_t R2; 
    volatile uint32_t R3; 
    volatile uint32_t R4; 
    volatile uint32_t R5; 
    volatile uint32_t R6; 
    volatile uint32_t R7; 
    volatile uint32_t CFG;
    volatile uint32_t GC; 
    volatile uint32_t GS; 
    volatile uint32_t CV; 
    volatile uint32_t OFS;
    volatile uint32_t CAL;
} IMXRT_ADCS_t;

#define IMXRT_ADC1S         (*(IMXRT_ADCS_t *)0x400C4000)
#define IMXRT_ADC2S         (*(IMXRT_ADCS_t *)0x400C8000)
#endif 

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

  ADACK_1_25_N, /*!< 1.25 MHz asynchronous ADC clock (independent of the global clocks F_CPU or F_BUS), Normal Mode*/
  ADACK_2_5_N,  /*!< 2.5 MHz asynchronous ADC clock (independent of the global clocks F_CPU or F_BUS), Normal Mode*/
  ADACK_5_0_N, /*!< 5.0 MHz asynchronous ADC clock (independent of the global clocks F_CPU or F_BUS), Normal Mode*/
  ADACK_10_0_N, /*!< 10.0 MHz asynchronous ADC clock (independent of the global clocks F_CPU or F_BUS), Normal Mode*/
  ADACK_2_5_H,  /*!< 2.5 MHz asynchronous ADC clock (independent of the global clocks F_CPU or F_BUS), High Speed*/
  ADACK_5_0_H, /*!< 5.0 MHz asynchronous ADC clock (independent of the global clocks F_CPU or F_BUS), High Speed*/
  ADACK_10_0_H, /*!< 10.0 MHz asynchronous ADC clock (independent of the global clocks F_CPU or F_BUS), High Speed*/
  ADACK_20_0_H /*!< 20.0 MHz asynchronous ADC clock (independent of the global clocks F_CPU or F_BUS), High Speed*/
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
		TIMEOUT             = 1<<10,/*!Timeout inside the analogRead method */

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
				case ADC_ERROR::TIMEOUT:
                    Serial.print("Analog read timeout ");
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

//analog compare
#define CMP_MUXCR_PSEL(n)       (uint8_t)(((n) & 0x07) << 3) // Plus Input Mux Control //input select 290-291
#define CMP_MUXCR_MSEL(n)       (uint8_t)(((n) & 0x07) << 0) // Minus Input Mux Control  //set to 7 always

#define DACCR_ENABLE (1<<7)   //Enable the DAC
#define DACCR_DISABLE (0<<7)
#define CMP_CR1_ENABLE (1<<0) //Enable Analog Comparator module
#define CMP_CR1_DISABLE (0<<0)
#define CMP_SCR_COUT (1<<0)

#define ADC_GC_ACFE_MASK                         (0x10U)
#define ADC_GC_ACFGT_MASK                        (0x8U)
#define ADC_GC_ACREN_MASK                        (0x4U)


// debug mode: blink the led light


class ADCL_Module
{
  protected:
   IMXRT_ADCS_t &_padc;
   int8_t       _adc_num;
  private:


  public:

    /** Default constructor */
    ADCL_Module(IMXRT_ADCS_t &padc, int8_t adc_num) : _padc(padc), _adc_num(adc_num) {};
    
    //! This flag indicates that some kind of error took place
    /** Use the defines at the beginning of this file to find out what caused the fail.
    */
    volatile ADC_ERROR fail_flag = ADC_ERROR::CLEAR;

    //! Prints the human-readable error, if any.
    void printError() {
        ADC_Error::printError(fail_flag, _adc_num);
    }

    //! Resets all errors from the ADC, if any.
    void resetError() {
        ADC_Error::resetError(fail_flag);
    }

    // Quick and dirty version to handle cases wehre callers made use of these modules...
    void setReference(ADC_REFERENCE type);

    void setResolution(uint8_t bits);
    uint8_t getResolution();
    uint32_t getMaxValue();
    void setConversionSpeed(ADC_CONVERSION_SPEED speed);
    void setSamplingSpeed(ADC_SAMPLING_SPEED speed);
    void setAveraging(uint8_t num);
    void attachInterrupt(void (*adc_isr)(void));
    void enableInterrupts();
    void disableInterrupts();
    void enableDMA();
    void disableDMA();
    bool isConverting();
    bool isComplete();
    int analogRead(uint8_t pin, uint32_t timeout = -1);
    bool startSingleRead(uint8_t pin);
    int readSingle()  __attribute__((always_inline)) {
      return (int16_t)(int32_t)_padc.R0;
    }

    bool startContinuous(uint8_t pin);
    bool startContinuousDifferential(uint8_t pinP, uint8_t pinN);
    int analogReadContinuous() __attribute__((always_inline)) {
        return (int16_t)(int32_t)_padc.R0;
    }

    //! Stops continuous conversion
    void stopContinuous();
	
	int getAdcCompareRes(uint8_t acmp_pin);

};

#endif

