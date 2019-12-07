#ifndef ADCL_T4_MODULE_H
#define ADCL_T4_MODULE_H


class ADCL_Module
{
  protected:
   IMXRT_ADCS_t &_padc;
   int8_t       _adc_num;
  private:


  public:

    /** Default constructor */
    ADCL_Module(IMXRT_ADCS_t &padc, int8_t adc_num) : _padc(padc), _adc_num(adc_num) {};
    
    // Quick and dirty version to handle cases wehre callers made use of these modules...
    void setReference(ADC_REFERENCE type);

    void setResolution(uint8_t bits);
    uint8_t getResolution();
    uint32_t getMaxValue();
    void setConversionSpeed(ADC_CONVERSION_SPEED speed);
    void setSamplingSpeed(ADC_SAMPLING_SPEED speed);
    void setAveraging(uint8_t num);
    void enableInterrupts();
    void disableInterrupts();
    void enableDMA();
    void disableDMA();
    bool isConverting();
    bool isComplete();
    int analogRead(uint8_t pin);
    bool startSingleRead(uint8_t pin);
    int readSingle();
};

#endif

