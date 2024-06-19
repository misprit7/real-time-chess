
// There are two speedup options for some of the FFT code:

// Define this to use reciprocal multiplication for division and some more speedups that might decrease precision
//#define FFT_SPEED_OVER_PRECISION

// Define this to use a low-precision square root approximation instead of the regular sqrt() call
// This might only work for specific use cases, but is significantly faster. Only works for ArduinoFFT<float>.
//#define FFT_SQRT_APPROXIMATION

/******************************************************************************
 * Includes
 ******************************************************************************/

// Overrides default arduino_freertos config
// Pretty sure I didn't do this right, but no 
/*#include "freeRTOSConfig.h"*/

#include "arduino_freertos.h"
#include "avr/pgmspace.h"
#include "arduinoFFT.h"
#include "core_pins.h"
#include "projdefs.h"
#include "wiring.h"
#include <WS2812Serial.h>

#include "goertzel.h"

#include <Arduino.h>
#include <vector>
#include <cmath>
#include <complex>

#define ever (;;)

/******************************************************************************
 * Hardware Definitions
 ******************************************************************************/
// All taken from kicad schematic

const int n_sqr = 16;

int em_pins[n_sqr] = {
    4, 5, 6, 7,
    8, 9, 10, 11,
    12, 28, 29, 30,
    31, 32, 33, 34,
};

int sens_pins[n_sqr] = {
    14, 15, 16, 17,
    18, 19, 20, 21,
    22, 23, 24, 25,
    26, 27, 38, 39
};

int freq_pins[2] = { 2, 3 };

const int led_pin = 1;

/******************************************************************************
 * ws2812Serial
 ******************************************************************************/
const int numled = 128;

byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED

WS2812Serial leds(numled, displayMemory, drawingMemory, led_pin, WS2812_GRB);

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
const uint16_t n_samples = 64; //This value MUST ALWAYS be a power of 2
const float freq_sample = 5000; //Hz, must be less than 10000 due to ADC
const double freq_output = 500;
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
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, n_samples, freq_sample, true);

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

long t_output = 0;


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
                abscissa = ((i * 1.0) / freq_sample);
        break;
            case SCL_FREQUENCY:
                abscissa = ((i * 1.0 * freq_sample) / n_samples);
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

std::complex<float> dft_for_frequency(const float* input, size_t N, float freq, float sampleRate) {
    std::complex<float> result(0.0, 0.0);
    const float pi = 3.14159265358979323846;

    for (size_t n = 0; n < N; ++n) {
        float angle = -2.0 * pi * freq * n / sampleRate;
        std::complex<float> expTerm(cos(angle), sin(angle));
        result += input[n] * expTerm;
    }

    return result;
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
        /*Serial.println("Blink");*/
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

static void freq_output_task(void*) {
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t output_state = 0;

    pinMode(freq_pins[0], arduino::OUTPUT);
    pinMode(freq_pins[1], arduino::OUTPUT);

    for ever {
        output_state = !output_state;
        digitalWrite(freq_pins[0], output_state);
        vTaskDelayUntil(&last_wake, pdUS_TO_TICKS(1e6/freq_output/2));
    }
}

static void fft_task(void*) {

    sampling_period_us = round(1000000*(1.0/freq_sample));
    pinMode(lift_pin, arduino::OUTPUT);
    digitalWrite(lift_pin, 0);
    t_input = micros();
    t_output = micros();
    t_last_print = micros();
    t_start = micros();

    /* Serial.println(analogRead(CHANNEL)); */
    /* return; */
    /*SAMPLING*/

    /* if(micros() > t_start + 1e6){ */
    /*     digitalWrite(lift_pin, 0); */
    /* } */
    /* return; */

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_print = xTaskGetTickCount();
    Goertzel goertzel(freq_output, freq_sample, 100);
    for ever {
        double sample = analogRead(sens_pins[0])*3.3/1024;
        /*Serial.println(analogRead(sens_pins[0]));*/
        goertzel.addSample(sample);

        TickType_t cur_tick = xTaskGetTickCount();
        if(cur_tick - last_print >= pdMS_TO_TICKS(500)){
            Serial.print(goertzel.getMagnitude());
            Serial.print(", ");
            Serial.println(sample);
            last_print = cur_tick;
        }

        vTaskDelayUntil(&last_wake, pdUS_TO_TICKS(sampling_period_us));
        /*vReal[input_idx] = analogRead(sens_pins[0]);*/
        /*vImag[input_idx] = 0;*/
        /**/
        /*t_input += sampling_period_us;*/
        /*input_idx += 1;*/
        /**/
        /*if(input_idx >= n_samples){*/
        /*    input_idx = 0;*/
        /**/
        /*    if(micros() - t_last_print > 100000){*/
        /*        FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);*/
        /*        FFT.dcRemoval();*/
        /*        FFT.compute(FFTDirection::Forward);*/
        /*        FFT.complexToMagnitude();*/
        /**/
        /*        float max = 0;*/
        /*        int maxi = 0;*/
        /*        for(int i = 5; i < n_samples/2; ++i){*/
        /*            if(vReal[i] > max){*/
        /*                maxi = i;*/
        /*                max = vReal[i];*/
        /*            }*/
        /*        }*/
        /*        float df = samplingFrequency / n_samples;*/
        /*        if(vReal[maxi] > 2000){*/
        /*            Serial.print("Touching: ");*/
        /*            digitalWrite(lift_pin, 1);*/
        /*        } else {*/
        /*            Serial.print("Not touching: ");*/
        /*            digitalWrite(lift_pin, 0);*/
        /*        }*/
        /*        Serial.print(maxi * df, 6);*/
        /*        Serial.print(", ");*/
        /*        Serial.println(vReal[maxi]);*/
        /*        t_last_print = micros();*/
        /*    }*/
        /*}*/
    }

}

/******************************************************************************
 * Arduino Setup/Loop
 ******************************************************************************/

FLASHMEM __attribute__((noinline)) void setup() {
    Serial.begin(115200);
    delay(200);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

    xTaskCreate(blink_task, "blink_task", 1024, nullptr, 2, nullptr);
    xTaskCreate(fft_task, "fft_task", 1024, nullptr, 2, nullptr);
    xTaskCreate(freq_output_task, "freq_output_task", 1024, nullptr, 2, nullptr);

    /*static int square_idx[16];*/
    /*for(int i = 0; i < 16; ++i){*/
    /*    square_idx[i] = i;*/
    /*    xTaskCreate(led_task, "led_task", 128, &square_idx[i], 2, nullptr);*/
    /*}*/

    Serial.println("setup(): starting scheduler...");
    Serial.flush();
    pinMode(13, arduino::OUTPUT);
    leds.begin();

    vTaskStartScheduler();
}

void loop() {}

