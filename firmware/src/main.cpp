
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
#include "semphr.h"
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
 * Config
 ******************************************************************************/

const int cooldown_ms = 10'000;

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
SemaphoreHandle_t leds_mut;

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
const float freq_sample = 5000; //Hz, must be less than 10000 due to ADC
double freq_output[2] = { 500, 750 };
unsigned int sampling_period_us;

/******************************************************************************
 * Helper functions
 ******************************************************************************/

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

static void led_task(void*){
    for ever {
        xSemaphoreTake(leds_mut, portMAX_DELAY);
        leds.show();
        xSemaphoreGive(leds_mut);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void square_task(void *params) {
    int idx = *((int*) params);
        /*int delay_ms = 1500 / leds.numPixels();*/
        /**/
        /*for (int i=0; i < 8; i++) {*/
        /*    leds.setPixel(8*idx + i, idx % 2 == 0 ? RED : GREEN);*/
        /*}*/
        /*vTaskDelay(pdMS_TO_TICKS(1000));*/
    sampling_period_us = round(1000000*(1.0/freq_sample));
    /*pinMode(lift_pin, arduino::OUTPUT);*/
    /*digitalWrite(lift_pin, 0);*/

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_print = xTaskGetTickCount();
    TickType_t last_led = xTaskGetTickCount();

    TickType_t last_touch = xTaskGetTickCount();

    Goertzel goertzel(freq_output[0], freq_sample, 100);
    for ever {
        TickType_t cur_tick = xTaskGetTickCount();
        double sample = analogRead(sens_pins[idx])*3.3/1024;
        /*Serial.println(analogRead(sens_pins[0]));*/
        goertzel.addSample(sample);
        if(goertzel.getMagnitude() > 25){
            last_touch = cur_tick;
        }

        if(cur_tick - last_print >= pdMS_TO_TICKS(500)){
            Serial.print(idx);
            Serial.print(", ");
            Serial.print(goertzel.getMagnitude());
            Serial.print(", ");
            Serial.println(sample);
            last_print = cur_tick;
        }

        if(cur_tick - last_touch <= pdMS_TO_TICKS(cooldown_ms) && cur_tick - last_led >= pdMS_TO_TICKS(100)){
            for (int i=0; i < 8; i++) {
                if(xSemaphoreTake(leds_mut, pdMS_TO_TICKS(100))){
                    leds.setPixel(8*idx + i, i < 8.0*pdTICKS_TO_MS(cur_tick - last_touch)/cooldown_ms ? OFF : WHITE);
                    xSemaphoreGive(leds_mut);
                } else Serial.println("Failed to take mutex!");
            }
            last_led = cur_tick;
        }

        vTaskDelayUntil(&last_wake, pdUS_TO_TICKS(sampling_period_us));
    }
}

static void freq_output_task(void *params) {
    int idx = *((int*) params);

    TickType_t last_wake = xTaskGetTickCount();
    uint8_t output_state = 0;

    pinMode(freq_pins[idx], arduino::OUTPUT);

    for ever {
        output_state = !output_state;
        digitalWrite(freq_pins[idx], output_state);
        vTaskDelayUntil(&last_wake, pdUS_TO_TICKS(1e6/freq_output[idx]/2));
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

    leds_mut = xSemaphoreCreateMutex();

    xTaskCreate(blink_task, "blink_task", 1024, nullptr, 2, nullptr);

    xTaskCreate(led_task, "led_task", 1024, nullptr, 2, nullptr);

    static int freq_idx[2] = {0, 1};
    xTaskCreate(freq_output_task, "freq_output_task_1", 1024, &freq_idx[0], 2, nullptr);
    xTaskCreate(freq_output_task, "freq_output_task_2", 1024, &freq_idx[1], 2, nullptr);

    static int square_idx[16];
    for(int i = 0; i < 2; ++i){
        square_idx[i] = i;
        xTaskCreate(square_task, "square_task", 1024, &square_idx[i], 2, nullptr);
    }

    Serial.println("setup(): starting scheduler...");
    Serial.flush();
    pinMode(13, arduino::OUTPUT);
    leds.begin();

    vTaskStartScheduler();
}

void loop() {}

