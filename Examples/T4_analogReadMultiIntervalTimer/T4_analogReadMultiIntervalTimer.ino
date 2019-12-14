/*
    This example shows how to use the IntervalTimer library and the ADC library in the Teensy 4.

    It uses the SD library to try to write the data to disk.

*/
#include "ADCL_t4.h"
#include <IntervalTimer.h>
//#include <SD.h>
#include "SdFat.h"
#define SD_FAT_TYPE 3

#define DEFAULT_OUTPUT_CSV_FORMAT

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
bool sd_begin_done = false;

FsFile log_file;

#ifdef DEFAULT_OUTPUT_CSV_FORMAT
char file_name[80] = "datalog.csv";
bool output_csv_format = true;
#else
char file_name[80] = "datalog.dat";
bool output_csv_format = false;
#endif

//-----------------------------------------------------------------
// Define a link list of buffers for us to store data in
//-----------------------------------------------------------------
typedef struct _sample_buffer_t {
    volatile struct _sample_buffer_t *next;  // pointer to next one...
    volatile uint16_t  count;                // Count of bytes in buffer;
    union {
        uint16_t w[256];                     // buffer of samples to write to disk 512 bytes...
        char     b[512];
    } buffer;
} sample_buffer_t;

volatile sample_buffer_t *current_adc_buffer = nullptr;
volatile sample_buffer_t *first_buffer_to_write_to_sd = nullptr;
volatile sample_buffer_t *free_buffers = nullptr;
uint32_t count_buffers_allocated = 0;

//-----------------------------------------------------------------
// Other globals
//-----------------------------------------------------------------
volatile uint8_t ADCPinIndex = 0;

volatile bool continue_to_collect_data = false;
volatile bool samples_completed = false;
uint32_t start_time;
uint32_t last_report_time;
uint32_t count_buffers_output;
volatile uint32_t count_adc_samples;

char command_line[80];

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
    Serial.println("If line starts with , -> CSV file, # -> binary file, followed by optional filename");
    allocate_timer_buffer();    // make sure we have a buffer for the timer.
    delay(500);
}

//==============================================================
// Loop: Main Arduino loop
//==============================================================
void loop() {

    // First test is to run 512 samples of each analog
    if (Serial.available()) {
        // lets grab command line
        char *psz = command_line;
        int ch;
        while ((ch = Serial.read()) != -1) {
            if ((ch >= ' ') && (ch <= '~')) 
                *psz++ = ch;
            else
                break; 
        }
        *psz= 0;
        while (Serial.read() != -1) ;
        // first pass just toggle on/off test
        if (!continue_to_collect_data) {
            // See if the user is trying to tell use which type of file? 
            psz = command_line;
            if (*psz == ',') {
                output_csv_format = true;
                psz++;
            } else if (*psz == '#') {
                output_csv_format = false;
                psz++;                
            }
            if (*psz)strcpy(file_name, psz);

            // Lets try to open the SD File...
            if (!sd_begin_done) {
                if (!sd.begin(SdioConfig(DMA_SDIO))) {
                    Serial.println("** SD.begin failed ***");
                } else
                    sd_begin_done = true;
                delay(250);
            }

            if (!log_file.open(file_name, O_WRITE | O_CREAT | O_TRUNC)) {
                Serial.printf("*** Failed to create SD file (%s) ***\n", file_name);
                return;
            }
            Serial.printf("Output to %s CSV:%d\n", file_name, output_csv_format);
            samples_completed = false;
            ADCPinIndex = 0;

            last_report_time = start_time = millis();
            count_buffers_output = 0;
            count_adc_samples = 0;

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
            Serial.println("Closing down sampling");
            continue_to_collect_data = false;
        }
    }

    // See if we have any data to try to write out to SD Card.
    if (first_buffer_to_write_to_sd) {
        digitalWriteFast(ledPin, !digitalReadFast(ledPin));
        volatile sample_buffer_t *buffer_to_write = first_buffer_to_write_to_sd;

        log_file.write((uint8_t*)buffer_to_write->buffer.b, buffer_to_write->count); // note should parameterize size here
        count_buffers_output++;
        __disable_irq();
        first_buffer_to_write_to_sd = buffer_to_write->next;    // unlink us
        buffer_to_write->next = free_buffers;
        free_buffers = buffer_to_write;
        __enable_irq();
    } else  if (samples_completed) {
        log_file.close();
        Serial.printf("collection ended after %d seconds sample Count:%d with buffer output count: %d\n", millis() - start_time, count_adc_samples, count_buffers_output);
        samples_completed = false;  // only report once
    }
    if (continue_to_collect_data && ((millis() - last_report_time) > 5000)) {
        Serial.printf("Running for %d seconds sample count: %d buffer output count:%d buffers allocated: %d\n", millis() - start_time, count_adc_samples, count_buffers_output, count_buffers_allocated);
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
        current_adc_buffer->count = 0;
        return true;
    }
    return false;
}


//==============================================================
//
//==============================================================
void move_adc_buffer_to_sd_list(bool force_allocate) {
    if (!current_adc_buffer) return;
    if (first_buffer_to_write_to_sd) {
        volatile sample_buffer_t *psb = first_buffer_to_write_to_sd;
        while (psb->next) psb = psb->next;
        psb->next = current_adc_buffer;
    } else {
        first_buffer_to_write_to_sd = current_adc_buffer;
    }
    if (continue_to_collect_data || force_allocate) {
        if (!allocate_timer_buffer()) {
            continue_to_collect_data = false;
            return;
        }
    } else {
        current_adc_buffer = nullptr;
    }
}

//==============================================================
//
//==============================================================
void output_bytes_to_csv_buffer(const char *buf) {
    if (!current_adc_buffer) return;
    while (*buf) {
        current_adc_buffer->buffer.b[current_adc_buffer->count++] = *buf++;

        if (current_adc_buffer->count == 512) {
            move_adc_buffer_to_sd_list(true);   // we wish to complete a complete row before quiting
            if (!current_adc_buffer)
                return; // but it might fail
        }
    }
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
    count_adc_samples++;
    uint16_t adc0_val, adc1_val;

    // try to grab data from ADC_0
    if (adc->adc0->isComplete()) {
        adc0_val = adc->adc0->readSingle();
    } else {
        Serial.println("ADC_0: did not complete");
        adc0_val = 0xffff;
    }
    if (adc->adc1->isComplete()) {
        adc1_val = adc->adc1->readSingle();
    } else {
        Serial.println("ADC_1: did not complete");
        adc1_val =  0xffff;
    }

    // Requested output in CSV (ascii data... )
    if (output_csv_format) {
        // first quick and dirty
        char num_buf[6];
        if (ADCPinIndex != 0) output_bytes_to_csv_buffer(",");
        ultoa(adc0_val, num_buf, 10);
        output_bytes_to_csv_buffer(num_buf);
        output_bytes_to_csv_buffer(",");
        ultoa(adc1_val, num_buf, 10);
        output_bytes_to_csv_buffer(num_buf);
        if (ADCPinIndex == (sizeof(ADC0_pins) - 1)) {
            output_bytes_to_csv_buffer("\n\r");
            if (!continue_to_collect_data) {
                move_adc_buffer_to_sd_list(false);
                timer.end();
                samples_completed = true;
            }
        }

    } else {
        // nope simple raw data file...
        current_adc_buffer->buffer.w[current_adc_buffer->count++] = adc0_val;
        current_adc_buffer->buffer.w[current_adc_buffer->count++] = adc1_val;
        // Again should paramertize this...
        if (current_adc_buffer->count == 256) {
            // Have a full buffer...
            // Should add it to the end of the list for main code to process.
            current_adc_buffer->count = 512; // switch it to number of bytes, to be written out.
            move_adc_buffer_to_sd_list(false);
            if (!continue_to_collect_data) {
                timer.end();
                samples_completed = true;
            }
        }
    }
    if (current_adc_buffer) { // something to test to know we are still active...
        ADCPinIndex++;
        if (ADCPinIndex >= sizeof(ADC0_pins)) ADCPinIndex = 0;
        adc->adc0->startSingleRead(ADC0_pins[ADCPinIndex]);
        adc->adc1->startSingleRead(ADC1_pins[ADCPinIndex]);
    }
}
