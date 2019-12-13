/*
    This example shows how to use the IntervalTimer library and the ADC library in the Teensy 4.

    It uses the SD library to try to write the data to disk.

*/
#include "ADCL_t4.h"
#include <IntervalTimer.h>
//#include <SD.h>
#include "SdFat.h"
#define SD_FAT_TYPE 3

const int ledPin = LED_BUILTIN;
//-----------------------------------------------------------------
// Define some of the analog settings, like which pins, how fast...
//-----------------------------------------------------------------
const uint8_t ADC0_pins[] = {A0, A1};
const uint8_t ADC1_pins[] = {A2, A3};
#define COUNT_PINS_PER_ADC  sizeof(ADC0_pins)

const int interval_period = 500; // us
ADCL *adc = new ADCL(); // adc object

IntervalTimer timer; // timers

//-----------------------------------------------------------------
// SD file info
//-----------------------------------------------------------------
//File log_file;
SdFs sd;
FsFile log_file;

char file_name[80] = "datalog.txt";
//-----------------------------------------------------------------
// Define a link list of buffers for us to store data in
//-----------------------------------------------------------------
typedef struct _sample_buffer_t {
  volatile struct _sample_buffer_t *next;  // pointer to next one...
  uint16_t buffer[256];   // buffer of samples to write to disk 512 bytes...
} sample_buffer_t;

volatile sample_buffer_t *current_adc_buffer = nullptr;
volatile sample_buffer_t *first_buffer_to_write_to_sd = nullptr;
volatile sample_buffer_t *free_buffers = nullptr;
uint32_t count_buffers_allocated = 0;

//-----------------------------------------------------------------
// Other globals
//-----------------------------------------------------------------
volatile uint8_t ADCPinIndex = 0;
volatile uint16_t adc_samples_buffer_index = 0;

volatile bool continue_to_collect_data = false;
volatile bool samples_completed = false;
uint32_t start_time;
uint32_t last_report_time;
uint32_t count_buffers_output;

//-----------------------------------------------------------------
// Forward references.
//-----------------------------------------------------------------
extern void timer_callback(void);


//==============================================================
// setup: Arduino setup function
//==============================================================
void setup() {

  pinMode(ledPin, OUTPUT); // led blinks every loop
  while (!Serial && millis() < 4000) ;
  Serial.begin(9600);

  delay(1000);

  ///// ADC0 ////
  Serial.printf("Configure ADC 1 and 2");
  adc->setAveraging(8); // set number of averages
  adc->setResolution(12); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed
  adc->setAveraging(8); // set number of averages
  adc->setResolution(12); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); // change the conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED); // change the sampling speed
  Serial.println("Press any key to run test");
  allocate_timer_buffer();    // make sure we have a buffer for the timer.
  delay(500);
}

//==============================================================
// Loop: Main Arduino loop
//==============================================================
void loop() {

  // First test is to run 512 samples of each analog
  if (Serial.available()) {
    while (Serial.read() != -1) ;
    // first pass just toggle on/off test
    if (!continue_to_collect_data) {
      // Lets try to open the SD File...
      if (!sd.begin(SdioConfig(DMA_SDIO))) {
        Serial.println("** SD.begin failed ***");
      }
      delay(250);
      
      if (!log_file.open(file_name, O_RDWR | O_CREAT)) {
        Serial.printf("*** Failed to create SD file (%s) ***\n", file_name);
        return;
      }

      samples_completed = false;
      ADCPinIndex = 0;

      last_report_time = start_time = millis();
      count_buffers_output = 0;

      adc->adc0->startSingleRead(ADC0_pins[ADCPinIndex]);
      adc->adc1->startSingleRead(ADC1_pins[ADCPinIndex]);
      Serial.println("Starting Timer");
      if (!timer.begin(timer_callback, interval_period)) {
        Serial.println("Failed to start interval timer");
      } else {
        Serial.println("Timer started");
        continue_to_collect_data = true;
      }
    } else {
      // we should try to kill the timer...
      continue_to_collect_data = false;
    }
  }

  // See if we have any data to try to write out to SD Card.
  if (first_buffer_to_write_to_sd) {
    digitalWriteFast(ledPin, !digitalReadFast(ledPin));
    volatile sample_buffer_t *buffer_to_write = first_buffer_to_write_to_sd;

    log_file.write((uint8_t*)buffer_to_write->buffer, 512); // note should parameterize size here
    count_buffers_output++;
    __disable_irq();
    first_buffer_to_write_to_sd = buffer_to_write->next;    // unlink us
    buffer_to_write->next = free_buffers;
    free_buffers = buffer_to_write;
    __enable_irq();
  } else  if (samples_completed) {
    Serial.printf("collection ended after %d seconds with output count: %d\n", millis() - start_time, count_buffers_output);
    samples_completed = false;  // only report once
  }
  if (continue_to_collect_data && ((millis() - last_report_time) > 5000)) {
    Serial.printf("Running for %d seconds output count:%d buffers allocated: %d\n", millis() - start_time, count_buffers_output, count_buffers_allocated);
    last_report_time = millis();
  }
  delay(10);
}

//==============================================================
// allocate_timer_buffer:
//==============================================================
bool allocate_timer_buffer() {
  //__disable_irq();
  if (free_buffers) {
    current_adc_buffer = free_buffers;
    free_buffers = free_buffers->next;
  } else {
    current_adc_buffer = (sample_buffer_t*)malloc (sizeof(sample_buffer_t));
    count_buffers_allocated++;
  }
  //__enable_irq();
  if (current_adc_buffer) {
    current_adc_buffer->next = nullptr;
    adc_samples_buffer_index = 0;
    return true;
  }
  return false;
}

//==============================================================
// Timer callbck
//==============================================================
void timer_callback(void) {

  if (!current_adc_buffer) {
    if (!allocate_timer_buffer()) {
      // something wrong!
      timer.end();
      continue_to_collect_data = false;
      samples_completed = true;
    }

  }
  // try to grab data from ADC_0
  if (adc->adc0->isComplete()) {
    current_adc_buffer->buffer[adc_samples_buffer_index++] = adc->adc0->readSingle();
  } else {
    Serial.println("ADC_0: did not complete");
    current_adc_buffer->buffer[adc_samples_buffer_index++] = 0xffff;
  }
  if (adc->adc1->isComplete()) {
    current_adc_buffer->buffer[adc_samples_buffer_index++] = adc->adc1->readSingle();
  } else {
    Serial.println("ADC_1: did not complete");
    current_adc_buffer->buffer[adc_samples_buffer_index++] = 0xffff;
  }

  // Again should paramertize this...
  if (adc_samples_buffer_index == 256) {
    // Have a full buffer...
    // Should add it to the end of the list for main code to process.
    if (first_buffer_to_write_to_sd) {
      volatile sample_buffer_t *psb = first_buffer_to_write_to_sd;
      while (psb->next) psb = psb->next;
      psb->next = current_adc_buffer;
    } else {
      first_buffer_to_write_to_sd = current_adc_buffer;
    }
    if (continue_to_collect_data) {
      if (!allocate_timer_buffer()) continue_to_collect_data = false;
    }
    if (!continue_to_collect_data) {
      timer.end();
      samples_completed = true;
    }
  }
  if (current_adc_buffer) { // something to test to know we are still active...
    ADCPinIndex++;
    if (ADCPinIndex >= sizeof(ADC0_pins)) ADCPinIndex = 0;
    adc->adc0->startSingleRead(ADC0_pins[ADCPinIndex]);
    adc->adc1->startSingleRead(ADC1_pins[ADCPinIndex]);
  }
}
