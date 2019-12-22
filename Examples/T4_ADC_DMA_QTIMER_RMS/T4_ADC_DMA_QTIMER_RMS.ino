/*
    Warning this is T4 specific, although not sure how much...

    This one is really hacked up to try to use qtimer0/XBar/ADC_ETC to
    control the timing of the query!
*/

#include <ADCL_t4.h>
#include <DMAChannel.h>
#include <AnalogBufferDMA.h>
#include "imxrt_additions.h"

#define USE_TIMED_READS
//#define USE_PIT0

const int readPin = 15;
ADCL *adc = new ADCL(); // adc object
extern void dumpDMA_TCD(DMABaseClass *dmabc);

// Going to try two buffers here  using 2 dmaSettings and a DMAChannel

const uint32_t buffer_size = 1600;
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2[buffer_size];
AnalogBufferDMA abdma1(dma_adc_buff1, buffer_size, dma_adc_buff2, buffer_size);

void setup() {
  while (!Serial && millis() < 5000) ;

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(readPin, INPUT); //pin 23 single ended

  Serial.begin(9600);
  Serial.println("Setup ADC_0");
  // reference can be ADC_REFERENCE::REF_3V3, ADC_REFERENCE::REF_1V2 (not for Teensy LC) or ADC_REF_EXT.
  //adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2

  adc->setAveraging(8); // set number of averages
  adc->setResolution(12); // set bits of resolution


  // always call the compare functions after changing the resolution!
  //adc->enableCompare(1.0/3.3*adc->getMaxValue(ADC_0), 0, ADC_0); // measurement will be ready if value < 1.0V
  //adc->enableCompareRange(1.0*adc->getMaxValue(ADC_1)/3.3, 2.0*adc->getMaxValue(ADC_1)/3.3, 0, 1, ADC_1); // ready if value lies out of [1.0,2.0] V

  // enable DMA and interrupts
  Serial.println("before enableDMA"); Serial.flush();


  // setup a DMA Channel.
  // Now lets see the different things that RingbufferDMA setup for us before
  abdma1.init(adc, ADC_0, DMAMUX_SOURCE_ADC_ETC);
  Serial.println("After enableDMA"); Serial.flush();

  // Start the dma operation..
#ifdef USE_TIMED_READS
  setup_adc_hardware_trigger(readPin, &abdma1);
#else
  adc->analogRead(readPin, ADC_0);
#endif
  Serial.println("End Setup");
}

char c = 0;

int average_value = 2048;

void loop() {

  // Maybe only when both have triggered?
  if ( abdma1.interrupted()) {
    ProcessAnalogData(&abdma1, 0);
    Serial.println();
  }
}

void ProcessAnalogData(AnalogBufferDMA *pabdma, int8_t adc_num) {
  uint32_t sum_values = 0;
  uint16_t min_val = 0xffff;
  uint16_t max_val = 0;

  volatile uint16_t *pbuffer = pabdma->bufferLastISRFilled();
  volatile uint16_t *end_pbuffer = pbuffer + pabdma->bufferCountLastISRFilled();

  float sum_delta_sq = 0.0;
  if ((uint32_t)pbuffer >= 0x20200000u)  arm_dcache_delete((void*)pbuffer, sizeof(dma_adc_buff1));
  while (pbuffer < end_pbuffer) {
    if (*pbuffer < min_val) min_val = *pbuffer;
    if (*pbuffer > max_val) max_val = *pbuffer;
    sum_values += *pbuffer;
    int delta_from_center = (int) * pbuffer - average_value;
    sum_delta_sq += delta_from_center * delta_from_center;

    pbuffer++;
  }

  int rms = sqrt(sum_delta_sq / buffer_size);
  Serial.printf(" %d - %u(%u): %u <= %u <= %u %d ", adc_num, pabdma->interruptCount(), pabdma->interruptDeltaTime(), min_val,
                sum_values / buffer_size, max_val, rms);
  pabdma->clearInterrupt();
}

#ifdef USE_TIMED_READS
// try to use some teensy core functions...
// mainly out of pwm.c
extern "C" {
  extern void xbar_connect(unsigned int input, unsigned int output);
  extern void quadtimer_init(IMXRT_TMR_t *p);
  extern void quadtimerWrite(IMXRT_TMR_t *p, unsigned int submodule, uint16_t val);
  extern void quadtimerFrequency(IMXRT_TMR_t *p, unsigned int submodule, float frequency);
}
void xbar_init() {
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);   //turn clock on for xbara1
#ifdef USE_PIT0
  xbar_connect(XBARA1_IN_PIT_TRIGGER0, XBARA1_OUT_ADC_ETC_TRIG00);   // pit to adc_etc
#else
  xbar_connect(XBARA1_IN_QTIMER4_TIMER0, XBARA1_OUT_ADC_ETC_TRIG00);   // pit to adc_etc
#endif
}
void adc_init() {
  ADC1_CFG |= ADC_CFG_ADTRG;   // hardware trigger
  ADC1_HC0 = 16;   // ADC_ETC channel
  ADC1_GC &= ~ADC_GC_ADCO;
}

void adc_etc_init(uint8_t pin) {
  uint8_t adc_pin_channel = ADCL::mapPinToChannel(pin);
  IMXRT_ADC_ETC.CTRL = ADC_ETC_CTRL_SOFTRST; // SOFTRST
  IMXRT_ADC_ETC.CTRL &= ~ADC_ETC_CTRL_SOFTRST; // SOFTRST
  delay(5);
  IMXRT_ADC_ETC.CTRL = (ADC_ETC_CTRL_TSC_BYPASS | ADC_ETC_CTRL_DMA_MODE_SEL | ADC_ETC_CTRL_TRIG_ENABLE(1)); // 0x40000001;  // start with trigger 0
  Serial.printf("ADC_ETC_CTRL: %x %x\n",  IMXRT_ADC_ETC.CTRL, ADC_ETC_CTRL_TRIG_ENABLE(1));
  IMXRT_ADC_ETC.TRIG[0].CTRL = ADC_ETC_TRIG_CTRL_TRIG_CHAIN(0);   // chainlength -1 only us
  IMXRT_ADC_ETC.TRIG[0].CHAIN_1_0 =
    ADC_ETC_TRIG_CHAIN_IE0(1) /*| ADC_ETC_TRIG_CHAIN_B2B0 */
    | ADC_ETC_TRIG_CHAIN_HWTS0(1) | ADC_ETC_TRIG_CHAIN_CSEL0(adc_pin_channel) ;

  IMXRT_ADC_ETC.DMA_CTRL = ADC_ETC_DMA_CTRL_TRIQ_ENABLE(0);

  //ADC_ETC_TRIG0_CHAIN_1_0 = 0x1017;   // ADC1 7  chain channel, HWTS,  BB? TODO
  // From other sample
  //ADC_ETC_TRIG0_CTRL = ADC_ETC_TRIG_CTRL_SYNC_MODE; // 0x100;   // chainlength -1
  //ADC_ETC_TRIG0_CHAIN_1_0 = 0x50283017;   // ADC1 7 8, chain channel, HWTS, IE, B2B
}

#ifdef USE_PIT0
void pit_init(uint32_t cycles)
{
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
  PIT_MCR = 0;

  IMXRT_PIT_CHANNELS[0].LDVAL = cycles;
  IMXRT_PIT_CHANNELS[0].TCTRL = PIT_TCTRL_TEN;
}
#else
void  qtimer_init(float freq)  // try at 20 hz for test...
{
  Serial.println("Try to init QTimer"); Serial.flush();
  quadtimer_init(&IMXRT_TMR4);
  quadtimerFrequency(&IMXRT_TMR4, 0, freq);
  quadtimerWrite(&IMXRT_TMR4, 0, 5);

  Serial.println("After Qtimer init"); Serial.flush();

}
#endif
void update_dma_settings(AnalogBufferDMA *pabdma)
{
  // Lets muck with the dma structure we setup earlier....
  // for now lets assume ADC1 (ADC_0) // I think R0 is fine?
#if 0
  pabdma->_dmasettings_adc[0].source((volatile uint16_t&)(ADC1_R0));
  pabdma->_dmasettings_adc[1].source((volatile uint16_t&)(ADC1_R0));
  pabdma->_dmachannel_adc = pabdma->_dmasettings_adc[0];
  // Lets update the event...
  pabdma->_dmachannel_adc.disable();
  pabdma->_dmachannel_adc.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC_ETC);
  pabdma->_dmachannel_adc.enable();
#endif
}


void setup_adc_hardware_trigger(uint8_t pin, AnalogBufferDMA *pabdma)
{

  xbar_init();
  Serial.println("After XBAR"); Serial.flush();
  adc_init();
  // dma_init();  // hopefully already done.
  adc_etc_init(pin);
  Serial.println("After ADC/ADC_ETC"); Serial.flush();

  update_dma_settings(pabdma);

  Serial.println("After UPDATE DMA"); Serial.flush();
#ifdef USE_PIT0
  pit_init(24 * 1000);   // 1 khz  1000 ADCs/sec
#else
  qtimer_init(3000.0);  // try at 20 hz for test...
#endif

  // Lets again try dumping lots of data.
  dumpDMA_TCD(&(pabdma->_dmachannel_adc));
  dumpDMA_TCD(&(pabdma->_dmasettings_adc[0]));
  dumpDMA_TCD(&(pabdma->_dmasettings_adc[1]));
  Serial.printf("ADC1: HC0:%x HS:%x CFG:%x GC:%x GS:%x\n", ADC1_HC0, ADC1_HS,  ADC1_CFG, ADC1_GC, ADC1_GS);
  Serial.printf("ADC_ETC: CTRL:%x DMA: %x TRIG0: CTRL: %x CHAIN01:%x\n",
                IMXRT_ADC_ETC.CTRL, IMXRT_ADC_ETC.DMA_CTRL, ADC_ETC_TRIG0_CTRL, ADC_ETC_TRIG0_CHAIN_1_0);
}
#endif
