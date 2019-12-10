/*
    It doesn't work for Teensy LC yet!
*/

#include <ADCL_t4.h>
#include <DMAChannel.h>
const int readPin = 15;

ADCL *adc = new ADCL(); // adc object
extern void dumpDMA_TCD(DMABaseClass *dmabc);

volatile uint32_t dma_complete_dt = 0;
volatile uint32_t dma_interrupt_count = 0;

// Going to try two buffers here  using 2 dmaSettings and a DMAChannel

//DMAChannel* dmaChannel;

const uint32_t buffer_size = 1600;
DMAMEM static volatile int16_t __attribute__((aligned(32))) dma_adc_buff1[buffer_size];
DMAMEM static volatile int16_t __attribute__((aligned(32))) dma_adc_buff2[buffer_size];
DMASetting dmasettings_adc[2];
DMAChannel  dmachannel_adc;



#if ADC_NUM_ADCS>1
//const int buffer_size2 = 8;
//DMAMEM static volatile int16_t __attribute__((aligned(buffer_size2+0))) buffer2[buffer_size2];
//
//// use dma with ADC1
//RingBufferDMA *dmaBuffer2 = new RingBufferDMA(buffer2, buffer_size2, ADC_1);
#endif // defined

void setup() {
    while (!Serial && millis() < 5000) ;

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(readPin, INPUT); //pin 23 single ended

    Serial.begin(9600);
    Serial.println("Setup");
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
    dmasettings_adc[0].source((volatile uint16_t&)ADC1_R0);
    dmasettings_adc[0].destinationBuffer((uint16_t*)dma_adc_buff1, sizeof(dma_adc_buff1)); // 2*b_size is necessary for some reason
    dmasettings_adc[0].replaceSettingsOnCompletion(dmasettings_adc[1]);    // go off and use second one... 
    dmasettings_adc[0].interruptAtCompletion(); //interruptAtHalf or interruptAtCompletion

    dmasettings_adc[1].source((volatile uint16_t&)ADC1_R0);
    dmasettings_adc[1].destinationBuffer((uint16_t*)dma_adc_buff2, sizeof(dma_adc_buff2)); // 2*b_size is necessary for some reason
    dmasettings_adc[1].replaceSettingsOnCompletion(dmasettings_adc[0]);    // Cycle back to the first one
    dmasettings_adc[1].interruptAtCompletion(); //interruptAtHalf or interruptAtCompletion

/*
    dmachannel_adc.transferSize(2); // both SRC and DST size

    dmachannel_adc.transferCount(sizeof(dma_adc_buff1) / sizeof(dma_adc_buff1[0])); // transfer b_size values

    dmachannel_adc.interruptAtHalf(); //interruptAtHalf or interruptAtCompletion
    dmachannel_adc.interruptAtCompletion(); //interruptAtHalf or interruptAtCompletion
*/
    dmachannel_adc = dmasettings_adc[0];

    dmachannel_adc.attachInterrupt(dmaBuffer_isr);
    dmachannel_adc.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1); // start DMA channel when ADC finishes a conversion
    //arm_dcache_flush((void*)dmaChannel, sizeof(dmaChannel));
    dmachannel_adc.enable();


    dumpDMA_TCD(&dmachannel_adc);
    dumpDMA_TCD(&dmasettings_adc[0]);
    dumpDMA_TCD(&dmasettings_adc[1]);
    Serial.printf("ADC1: HC0:%x HS:%x CFG:%x GC:%x GS:%x\n", ADC1_HC0, ADC1_HS,  ADC1_CFG, ADC1_GC, ADC1_GS);


    adc->enableDMA(ADC_0);

    // ADC interrupt enabled isn't mandatory for DMA to work.
//  Serial.println("before enableInterrupts"); Serial.flush();
//  adc->enableInterrupts(ADC_0);

    // Start the dma operation..
    adc->analogRead(readPin, ADC_0);

    Serial.println("End Setup");
}

char c = 0;

int average_value = 2048;

void loop() {

    if (dma_complete_dt) {
        // Lets find min/max and compute average...
        uint32_t sum_values = 0;
        uint16_t min_val = 0xffff;
        uint16_t max_val = 0;
        volatile uint16_t *pbuffer = (dma_interrupt_count & 1)? dma_adc_buff1 : dma_adc_buff2;
        uint16_t *end_pbuffer = pbuffer + buffer_size;
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
        Serial.printf("%u(%u): %u <= %u <= %u %d\n", dma_interrupt_count, dma_complete_dt, min_val, sum_values / buffer_size, max_val, rms);
        dma_complete_dt = 0;

    }

}

uint32_t last_isr_time = 0;
void dmaBuffer_isr() {
    uint32_t cur_time = millis();

    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    //Serial.printf("dmaBuffer_isr: %u\n", cur_time-last_isr_time);
    dma_complete_dt = cur_time - last_isr_time;
    dma_interrupt_count++;
    last_isr_time = cur_time;
    // update the internal buffer positions
    dmachannel_adc.clearInterrupt();
    asm("DSB");
}
