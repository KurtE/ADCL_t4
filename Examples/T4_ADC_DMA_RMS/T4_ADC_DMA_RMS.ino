/*
    Warning this is T4 specific, although not sure how much...
*/

#include <ADCL_t4.h>
#include <DMAChannel.h>

const int readPin = 15;
// optional to define second pin
#define PROCECESS_2_PINS
const int readPin2 = 14;

ADCL *adc = new ADCL(); // adc object
extern void dumpDMA_TCD(DMABaseClass *dmabc);

// Going to try two buffers here  using 2 dmaSettings and a DMAChannel

// lets wrap some of our Dmasettings stuff into helper class
class AnalogBufferDMA {
    // keep our settings and the like:
    DMASetting dmasettings_adc[2];
    DMAChannel  dmachannel_adc;

public: 
    AnalogBufferDMA(volatile uint16_t *buffer1, uint16_t buffer1_count, volatile uint16_t *buffer2, uint16_t buffer2_count) :
            _buffer1(buffer1), _buffer1_count(buffer1_count), _buffer2(buffer2), _buffer2_count(buffer2_count) {};
    void init(ADCL *adc,  void (*dma_func)(), int8_t adc_num = -1);
    inline void clearInterrupt() {dmachannel_adc.clearInterrupt();}
    inline volatile uint16_t *bufferLastISRFilled() {return (interrupt_count & 1)? _buffer1 : _buffer2;}
    inline uint16_t bufferCountLastISRFilled() {return (interrupt_count & 1)? _buffer1_count : _buffer2_count;}

    volatile uint32_t interrupt_count = 0;
    volatile uint32_t interrupt_delta_time;
    volatile uint32_t last_isr_time;

    volatile uint16_t *_buffer1;
    uint16_t _buffer1_count;
    volatile uint16_t *_buffer2;
    uint16_t _buffer2_count;

};

const uint32_t buffer_size = 1600;
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2[buffer_size];
AnalogBufferDMA abdma1(dma_adc_buff1, buffer_size, dma_adc_buff2, buffer_size);

#ifdef PROCECESS_2_PINS
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc2_buff1[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc2_buff2[buffer_size];
AnalogBufferDMA abdma2(dma_adc2_buff1, buffer_size, dma_adc2_buff2, buffer_size);
#endif

void AnalogBufferDMA::init(ADCL *adc, void (*dma_func)(), int8_t adc_num)
{
    // enable DMA and interrupts
    Serial.println("before enableDMA"); Serial.flush();


    // setup a DMA Channel.
    // Now lets see the different things that RingbufferDMA setup for us before

    dmasettings_adc[0].source((volatile uint16_t&)((adc_num==1)? ADC2_R0 : ADC1_R0));
    dmasettings_adc[0].destinationBuffer((uint16_t*)_buffer1, _buffer1_count*2); // 2*b_size is necessary for some reason
    dmasettings_adc[0].replaceSettingsOnCompletion(dmasettings_adc[1]);    // go off and use second one... 
    dmasettings_adc[0].interruptAtCompletion(); //interruptAtHalf or interruptAtCompletion

    dmasettings_adc[1].source((volatile uint16_t&)((adc_num==1)? ADC2_R0 : ADC1_R0));
    dmasettings_adc[1].destinationBuffer((uint16_t*)_buffer2, _buffer2_count*2); // 2*b_size is necessary for some reason
    dmasettings_adc[1].replaceSettingsOnCompletion(dmasettings_adc[0]);    // Cycle back to the first one
    dmasettings_adc[1].interruptAtCompletion(); //interruptAtHalf or interruptAtCompletion

    dmachannel_adc = dmasettings_adc[0];

    dmachannel_adc.attachInterrupt(dma_func);
    dmachannel_adc.triggerAtHardwareEvent((adc_num==1)? DMAMUX_SOURCE_ADC2 : DMAMUX_SOURCE_ADC1); // start DMA channel when ADC finishes a conversion
    //arm_dcache_flush((void*)dmaChannel, sizeof(dmaChannel));
    dmachannel_adc.enable();


    dumpDMA_TCD(&dmachannel_adc);
    dumpDMA_TCD(&dmasettings_adc[0]);
    dumpDMA_TCD(&dmasettings_adc[1]);
    if (adc_num == 1)
        Serial.printf("ADC2: HC0:%x HS:%x CFG:%x GC:%x GS:%x\n", ADC2_HC0, ADC2_HS,  ADC2_CFG, ADC2_GC, ADC2_GS);
    else
        Serial.printf("ADC1: HC0:%x HS:%x CFG:%x GC:%x GS:%x\n", ADC1_HC0, ADC1_HS,  ADC1_CFG, ADC1_GC, ADC1_GS);


    adc->enableDMA(adc_num);    
    last_isr_time = millis();
}

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
    abdma1.init(adc, dmaBuffer_isr);
#ifdef PROCECESS_2_PINS
    Serial.println("Setup ADC_1");
    adc->setAveraging(8, ADC_1); // set number of averages
    adc->setResolution(12, ADC_1); // set bits of resolution
    abdma2.init(adc, dmaBuffer_isr2, ADC_1);
    adc->analogRead(readPin2, ADC_1);
#endif

    // Start the dma operation..
    adc->analogRead(readPin, ADC_0);

    Serial.println("End Setup");
}

char c = 0;

int average_value = 2048;

void loop() {

    // Maybe only when both have triggered?
#ifdef PROCECESS_2_PINS
    if ( abdma1.interrupt_delta_time && abdma2.interrupt_delta_time) {
        if (abdma1.interrupt_delta_time) ProcessAnalogData(&abdma1, 0);
        if (abdma2.interrupt_delta_time) ProcessAnalogData(&abdma2, 1);
        Serial.println(); 
    }
#else
    if ( abdma1.interrupt_delta_time) {
        ProcessAnalogData(&abdma1, 0);
        Serial.println(); 
    }
#endif    

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
    Serial.printf(" %d - %u(%u): %u <= %u <= %u %d ", adc_num, pabdma->interrupt_count, pabdma->interrupt_delta_time, min_val, sum_values / buffer_size, max_val, rms);
     pabdma->interrupt_delta_time = 0;

}

void dmaBuffer_isr() {
    uint32_t cur_time = millis();

//    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    //Serial.printf("dmaBuffer_isr: %u\n", cur_time-last_isr_time);
    abdma1.interrupt_count++;
    abdma1.interrupt_delta_time = cur_time - abdma1.last_isr_time;
    abdma1.last_isr_time = cur_time;
    // update the internal buffer positions
    abdma1.clearInterrupt();
    asm("DSB");
}

#ifdef PROCECESS_2_PINS
void dmaBuffer_isr2() {
    uint32_t cur_time = millis();

    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    //Serial.printf("dmaBuffer_isr: %u\n", cur_time-last_isr_time);
    abdma2.interrupt_count++;
    abdma2.interrupt_delta_time = cur_time - abdma2.last_isr_time;
    abdma2.last_isr_time = cur_time;
    // update the internal buffer positions
    abdma2.clearInterrupt();
    asm("DSB");
}
#endif