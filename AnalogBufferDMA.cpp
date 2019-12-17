/* Teensy 3.x, LC, 4.0 ADC library
 * https://github.com/pedvide/ADC
 * Copyright (c) 2019 Pedro Villanueva
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "AnalogBufferDMA.h"

#define DEBUG_DUMP_DATA
#ifdef DEBUG_DUMP_DATA
extern void dumpDMA_TCD(DMABaseClass *dmabc);
#endif
// Global objects
AnalogBufferDMA *AnalogBufferDMA::_activeObjectPerADC[2] = {nullptr, nullptr};



void AnalogBufferDMA::init(ADCL *adc, int8_t adc_num,  uint8_t dmamux_source)
{
    // enable DMA and interrupts
    Serial.println("before enableDMA"); Serial.flush();


    // setup a DMA Channel.
    // Now lets see the different things that RingbufferDMA setup for us before

    _dmasettings_adc[0].source((volatile uint16_t&)((adc_num==1)? ADC2_R0 : ADC1_R0));
    _dmasettings_adc[0].destinationBuffer((uint16_t*)_buffer1, _buffer1_count*2); // 2*b_size is necessary for some reason
    _dmasettings_adc[0].replaceSettingsOnCompletion(_dmasettings_adc[1]);    // go off and use second one... 
    _dmasettings_adc[0].interruptAtCompletion(); //interruptAtHalf or interruptAtCompletion

    _dmasettings_adc[1].source((volatile uint16_t&)((adc_num==1)? ADC2_R0 : ADC1_R0));
    _dmasettings_adc[1].destinationBuffer((uint16_t*)_buffer2, _buffer2_count*2); // 2*b_size is necessary for some reason
    _dmasettings_adc[1].replaceSettingsOnCompletion(_dmasettings_adc[0]);    // Cycle back to the first one
    _dmasettings_adc[1].interruptAtCompletion(); //interruptAtHalf or interruptAtCompletion

    _dmachannel_adc = _dmasettings_adc[0];

    if (adc_num == 1) {
    	_activeObjectPerADC[1] = this;
	    _dmachannel_adc.attachInterrupt(&adc_1_dmaISR);
	    _dmachannel_adc.triggerAtHardwareEvent((dmamux_source == 0xff)? DMAMUX_SOURCE_ADC2 : dmamux_source); // start DMA channel when ADC finishes a conversion
	} else {		
    	_activeObjectPerADC[0] = this;
	    _dmachannel_adc.attachInterrupt(&adc_0_dmaISR);
	    _dmachannel_adc.triggerAtHardwareEvent((dmamux_source == 0xff)? DMAMUX_SOURCE_ADC1 : dmamux_source); // start DMA channel when ADC finishes a conversion
	}
    //arm_dcache_flush((void*)dmaChannel, sizeof(dmaChannel));
    _dmachannel_adc.enable();

    adc->enableDMA(adc_num);    

#ifdef DEBUG_DUMP_DATA
    dumpDMA_TCD(&_dmachannel_adc);
    dumpDMA_TCD(&_dmasettings_adc[0]);
    dumpDMA_TCD(&_dmasettings_adc[1]);
    if (adc_num == 1) {
        Serial.printf("ADC2: HC0:%x HS:%x CFG:%x GC:%x GS:%x\n", ADC2_HC0, ADC2_HS,  ADC2_CFG, ADC2_GC, ADC2_GS);
    } else {
        Serial.printf("ADC1: HC0:%x HS:%x CFG:%x GC:%x GS:%x\n", ADC1_HC0, ADC1_HS,  ADC1_CFG, ADC1_GC, ADC1_GS);
    }
#endif

    _last_isr_time = millis();
}

void  AnalogBufferDMA::processADC_DMAISR() {
    digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    uint32_t cur_time = millis();

    _interrupt_count++;
    _interrupt_delta_time = cur_time - _last_isr_time;
    _last_isr_time = cur_time;
    // update the internal buffer positions
    _dmachannel_adc.clearInterrupt();
}

void AnalogBufferDMA::adc_0_dmaISR() {
	if (_activeObjectPerADC[0]) {
		_activeObjectPerADC[0]->processADC_DMAISR();
	}
    asm("DSB");
}

void AnalogBufferDMA::adc_1_dmaISR() {
	if (_activeObjectPerADC[1]) {
		_activeObjectPerADC[1]->processADC_DMAISR();
	}
    asm("DSB");
}
