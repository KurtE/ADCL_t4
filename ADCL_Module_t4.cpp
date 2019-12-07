#include "ADCL_t4.h"
#include "ADCL_Module_t4.h"

    
// Quick and dirty version to handle cases wehre callers made use of these modules...
void ADCL_Module::setReference(ADC_REFERENCE type)
{
}



void ADCL_Module::setResolution(uint8_t bits)
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

    tmp32  = (_padc.CFG & (0xFFFFFC00));
    tmp32 |= (_padc.CFG & (0x03));  // ADICLK
    tmp32 |= (_padc.CFG & (0xE0));  // ADIV & ADLPC

    tmp32 |= mode;
    _padc.CFG = tmp32;
}


uint8_t ADCL_Module::getResolution()
{
  // bits 2-3:  00->8 01->10  10->12 
  return  8 + ((_padc.CFG >> 1) & 0x6); 
}


uint32_t ADCL_Module::getMaxValue()
{
  switch (getResolution()) {
    case 8: return 255;
    case 10: return 1023;
    default: return 4095;
  }
}

void ADCL_Module::setConversionSpeed(ADC_CONVERSION_SPEED speed)
{
  uint32_t adc_cfg_adsts;
  switch (speed) {
   case ADC_CONVERSION_SPEED::VERY_LOW_SPEED:    /*!< is guaranteed to be the lowest possible speed within specs for resolutions less than 16 bits (higher than 1 MHz). */
      adc_cfg_adsts = ADC_CFG_ADSTS(3);
      break;
   case ADC_CONVERSION_SPEED::LOW_SPEED:     /*!< is guaranteed to be the lowest possible speed within specs for all resolutions (higher than 2 MHz). */
      adc_cfg_adsts = ADC_CFG_ADSTS(2);
      break;
   case ADC_CONVERSION_SPEED::MED_SPEED:     /*!< is always >= LOW_SPEED and <= HIGH_SPEED. */
      adc_cfg_adsts = ADC_CFG_ADSTS(1);
      break;
   default:   
   case ADC_CONVERSION_SPEED::HIGH_SPEED:    /*!< is guaranteed to be the highest possible speed within specs for resolutions less than 16 bits (lower */
      adc_cfg_adsts = ADC_CFG_ADSTS(0);
    break;
      break;

  }
  _padc.CFG = (_padc.CFG & ~(ADC_CFG_ADSTS(3))) | adc_cfg_adsts;    
}


void ADCL_Module::setSamplingSpeed(ADC_SAMPLING_SPEED speed)
{
  uint32_t adc_cfg_adlsmp = 0;
  switch (speed) {
    case ADC_SAMPLING_SPEED::VERY_LOW_SPEED: /*!< adds +16 ADCK. */
    case ADC_SAMPLING_SPEED::LOW_SPEED: /*!< adds +16 ADCK. */
      adc_cfg_adlsmp = ADC_CFG_ADLSMP;
      break;
    case ADC_SAMPLING_SPEED::MED_SPEED: /*!< adds +10 ADCK. */
      adc_cfg_adlsmp = ADC_CFG_ADLSMP | ADC_CFG_ADHSC;
      break;
    case ADC_SAMPLING_SPEED::VERY_HIGH_SPEED: /*!< adds +6 ADCK. */
    case ADC_SAMPLING_SPEED::HIGH_SPEED: /*!< adds +6 ADCK. */
      adc_cfg_adlsmp = ADC_CFG_ADHSC;
      break;
  }
  _padc.CFG = (_padc.CFG & ~(ADC_CFG_ADLSMP | ADC_CFG_ADHSC)) | adc_cfg_adlsmp;    
}


void ADCL_Module::setAveraging(uint8_t num)
{
  uint32_t mode;

    _padc.GC &= ~0x20;
    mode = _padc.CFG & ~0xC000;
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
    _padc.CFG = mode;
    if (num >= 4) {
      _padc.GC |= ADC_GC_AVGE;// turns on averaging
    }
}


void ADCL_Module::enableInterrupts()
{
      _padc.HC0 |= ADC_HC_AIEN;  // enable the interrupt      
}


void ADCL_Module::disableInterrupts()
{
      _padc.HC0 &= ~ADC_HC_AIEN;  // enable the interrupt
}


void ADCL_Module::enableDMA()
{
      _padc.GC |= ADC_GC_DMAEN;  // enable the interrupt      
}


void ADCL_Module::disableDMA()
{
      _padc.GC &= ~ADC_GC_DMAEN;  // enable the interrupt      
}


bool ADCL_Module::isConverting()
{
  return (_padc.HS & ADC_HS_COCO0) == 0;
}


bool ADCL_Module::isComplete()
{
  return (_padc.HS & ADC_HS_COCO0) != 0;
}


int ADCL_Module::analogRead(uint8_t pin)
{
  uint8_t ch = ADCL::mapPinToChannel(pin, _adc_num);
  if (ch == 0xff) return ADC_ERROR_VALUE;

  _padc.HC0 = ch;
  while (!(_padc.HS & ADC_HS_COCO0)) ; // wait
  return _padc.R0;
}  



bool ADCL_Module::startSingleRead(uint8_t pin)
{
  uint8_t ch = ADCL::mapPinToChannel(pin, _adc_num);
  if (ch == 0xff) return false;

  _padc.HC0 = ch;
  return true;
}


int ADCL_Module::readSingle()
{
  return _padc.R0;
}


