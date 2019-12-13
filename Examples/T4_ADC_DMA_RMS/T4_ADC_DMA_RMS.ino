/*
    Warning this is T4 specific, although not sure how much...
*/

#include <ADCL_t4.h>
#include <DMAChannel.h>
#include <AnalogBufferDMA.h>

const int readPin = 15;
// optional to define second pin
#define PROCECESS_2_PINS
const int readPin2 = 14;

ADCL *adc = new ADCL(); // adc object
extern void dumpDMA_TCD(DMABaseClass *dmabc);

// Going to try two buffers here  using 2 dmaSettings and a DMAChannel

const uint32_t buffer_size = 1600;
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff1[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc_buff2[buffer_size];
AnalogBufferDMA abdma1(dma_adc_buff1, buffer_size, dma_adc_buff2, buffer_size);

#ifdef PROCECESS_2_PINS
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc2_buff1[buffer_size];
DMAMEM static volatile uint16_t __attribute__((aligned(32))) dma_adc2_buff2[buffer_size];
AnalogBufferDMA abdma2(dma_adc2_buff1, buffer_size, dma_adc2_buff2, buffer_size);
#endif

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
    abdma1.init(adc, ADC_0);
#ifdef PROCECESS_2_PINS
    Serial.println("Setup ADC_1");
    adc->setAveraging(8, ADC_1); // set number of averages
    adc->setResolution(12, ADC_1); // set bits of resolution
    abdma2.init(adc, ADC_1);
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
    if ( abdma1.interrupted() && abdma2.interrupted()) {
        if (abdma1.interrupted()) ProcessAnalogData(&abdma1, 0);
        if (abdma2.interrupted()) ProcessAnalogData(&abdma2, 1);
        Serial.println(); 
    }
#else
    if ( abdma1.interrupted()) {
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
    Serial.printf(" %d - %u(%u): %u <= %u <= %u %d ", adc_num, pabdma->interruptCount(), pabdma->interruptDeltaTime(), min_val, 
            sum_values / buffer_size, max_val, rms);
     pabdma->clearInterrupt();
}
