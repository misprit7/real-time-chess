
// There are two speedup options for some of the FFT code:

// Define this to use reciprocal multiplication for division and some more speedups that might decrease precision
//#define FFT_SPEED_OVER_PRECISION

// Define this to use a low-precision square root approximation instead of the regular sqrt() call
// This might only work for specific use cases, but is significantly faster. Only works for ArduinoFFT<float>.
//#define FFT_SQRT_APPROXIMATION

/******************************************************************************
 * Includes
 ******************************************************************************/

/*#include "Arduino.h"*/
#include "arduino_freertos.h"
#include "avr/pgmspace.h"
#include "arduinoFFT.h"
#include "core_pins.h"
#include <WS2812Serial.h>

/******************************************************************************
 * Hardware Definitions
 ******************************************************************************/

/******************************************************************************
 * ws2812Serial
 ******************************************************************************/
const int numled = 128;
const int pin = 1;

byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED

WS2812Serial leds(numled, displayMemory, drawingMemory, pin, WS2812_GRB);

/*#define RED    0xFF0000*/
/*#define GREEN  0x00FF00*/
/*#define BLUE   0x0000FF*/
/*#define YELLOW 0xFFFF00*/
/*#define PINK   0xFF1088*/
/*#define ORANGE 0xE05800*/
/*#define WHITE  0xFFFFFF*/

// Less intense...
#define RED    0x160000
#define GREEN  0x001600
#define BLUE   0x000016
#define YELLOW 0x101400
#define PINK   0x120009
#define ORANGE 0x100400
#define WHITE  0x101010
#define OFF    0x000000

/******************************************************************************
 * FFT
 ******************************************************************************/
const uint8_t input_pin = A9;
const uint16_t n_samples = 64; //This value MUST ALWAYS be a power of 2
const float samplingFrequency = 5000; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long t_us;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
float vReal[n_samples];
float vImag[n_samples];

float freq[n_samples/2];

/* Create FFT object with weighing factor storage */
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, n_samples, samplingFrequency, true);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

const double freq_output = 1000;
long t_output = 0;
uint8_t output_state;
const int output_pin = 22;


long t_input = 0;
int input_idx = 0;

long t_last_print = 0;
long t_start = 0;

const int lift_pin = 0;

/******************************************************************************
 * Helper functions
 ******************************************************************************/

void print_vector(float *vData, uint16_t bufferSize, uint8_t scaleType)
{
    for (uint16_t i = 0; i < bufferSize; i++)
    {
        float abscissa;
        /* Print abscissa value */
        switch (scaleType)
        {
            case SCL_INDEX:
                abscissa = (i * 1.0);
        break;
            case SCL_TIME:
                abscissa = ((i * 1.0) / samplingFrequency);
        break;
            case SCL_FREQUENCY:
                abscissa = ((i * 1.0 * samplingFrequency) / n_samples);
        break;
        }
        Serial.print(abscissa, 6);
        if(scaleType==SCL_FREQUENCY)
            Serial.print("Hz");
        Serial.print(" ");
        Serial.println(vData[i], 4);
    }
    Serial.println();
}

void color_wipe(int color, int wait_ms) {
    for (int i=0; i < leds.numPixels(); i++) {
        leds.setPixel(i, color);
        leds.show();
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
}

/******************************************************************************
 * Tasks
 ******************************************************************************/

static void blink_task(void*) {
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    while (true) {
        digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void led_task(void *params) {
    int idx = *((int*) params);
    while (true) {
        int delay_ms = 1500 / leds.numPixels();

        for (int i=0; i < 8; i++) {
            leds.setPixel(8*idx + i, idx % 2 == 0 ? RED : GREEN);
        }
        leds.show();
        vTaskDelay(pdMS_TO_TICKS(1000));

        /*color_wipe(RED, delay_ms);*/
        /*color_wipe(GREEN, delay_ms);*/
        /*color_wipe(BLUE, delay_ms);*/
        /*color_wipe(YELLOW, delay_ms);*/
        /*color_wipe(PINK, delay_ms);*/
        /*color_wipe(ORANGE, delay_ms);*/
        /*color_wipe(WHITE, delay_ms);*/
    }
}

static void fft_task(void*) {
    /* Serial.println(analogRead(CHANNEL)); */
    /* return; */
    /*SAMPLING*/

    /* if(micros() > t_start + 1e6){ */
    /*     digitalWrite(lift_pin, 0); */
    /* } */
    /* return; */

    if(micros() - t_output > 1e6/freq_output/2){
        output_state = !output_state;
        digitalWrite(output_pin, output_state);
        t_output += 1e6/freq_output/2;
    }

    if(micros() - t_input > sampling_period_us){
        vReal[input_idx] = analogRead(input_pin);
        vImag[input_idx] = 0;

        t_input += sampling_period_us;
        input_idx += 1;

        if(input_idx >= n_samples){
            input_idx = 0;

            if(micros() - t_last_print > 100000){
                /* Print the results of the sampling according to time */
                /* Serial.println("Data:"); */
                /* PrintVector(vReal, n_samples, SCL_TIME); */
                FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */
                FFT.dcRemoval();
                /* Serial.println("Weighed data:"); */
                /* PrintVector(vReal, samples, SCL_TIME); */
                FFT.compute(FFTDirection::Forward); /* Compute FFT */
                /* Serial.println("Computed Real values:"); */
                /* PrintVector(vReal, samples, SCL_INDEX); */
                /* Serial.println("Computed Imaginary values:"); */
                /* PrintVector(vImag, samples, SCL_INDEX); */
                FFT.complexToMagnitude(); /* Compute magnitudes */
                /* Serial.println("Computed magnitudes:"); */
                /* PrintVector(vReal, (n_samples >> 1), SCL_FREQUENCY); */
                /* float x = FFT.majorPeak(); */
                /* Serial.println(x, 6); //Print out what frequency is the most dominant. */

                float max = 0;
                int maxi = 0;
                for(int i = 5; i < n_samples/2; ++i){
                    if(vReal[i] > max){
                        maxi = i;
                        max = vReal[i];
                    }
                }
                float df = samplingFrequency / n_samples;
                if(vReal[maxi] > 2000){
                    Serial.print("Touching: ");
                    digitalWrite(lift_pin, 1);
                } else {
                    Serial.print("Not touching:");
                    digitalWrite(lift_pin, 0);
                }
                Serial.print(maxi * df, 6);
                Serial.print(", ");
                Serial.println(vReal[maxi]);
                t_last_print = micros();
            }
        }
    }


}

/******************************************************************************
 * Arduino Setup/Loop
 ******************************************************************************/

FLASHMEM __attribute__((noinline)) void setup() {
    Serial.begin(115200);
    delay(2'000);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    xTaskCreate(blink_task, "blink_task", 128, nullptr, 2, nullptr);

    static int square_idx[16];
    for(int i = 0; i < 16; ++i){
        square_idx[i] = i;
        xTaskCreate(led_task, "led_task", 128, &square_idx[i], 2, nullptr);
    }

    Serial.println("setup(): starting scheduler...");
    Serial.flush();

    /*pinMode(22, OUTPUT);*/
    /*sampling_period_us = round(1000000*(1.0/samplingFrequency));*/
    /*pinMode(lift_pin, OUTPUT);*/
    /*digitalWrite(lift_pin, 0);*/
    /*t_input = micros();*/
    /*t_output = micros();*/
    /*t_last_print = micros();*/
    /*t_start = micros();*/
    pinMode(13, arduino::OUTPUT);
    leds.begin();

    vTaskStartScheduler();
}

void loop() {}

