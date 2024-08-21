
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
#include "portable/portmacro.h"
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

#define INTENSE_RED    0xFF0000
#define INTENSE_GREEN  0x00FF00
#define INTENSE_BLUE   0x0000FF
#define INTENSE_YELLOW 0xFFFF00
#define INTENSE_PINK   0xFF1088
#define INTENSE_ORANGE 0xE05800
#define INTENSE_WHITE  0xFFFFFF

// Less intense...
#define RED    0x160000
#define GREEN  0x001600
#define BLUE   0x000016
#define YELLOW 0x101400
#define PINK   0x120009
#define ORANGE 0x100400
#define WHITE  0x101010
#define OFF    0x000000

const int cooldown_ms = 3'000;
const int cooldown_tk = pdMS_TO_TICKS(cooldown_ms);
int plr_clrs_lock[2] = {INTENSE_RED, INTENSE_BLUE};
int plr_clrs_unlock[2] = {RED, BLUE};

// Threshold for ewma of goertzel filter before considered touching
// Not sure if goertzel filter isn't scale invariant to e.g. window size,
// freq, etc. If not should probably be scaled appropriately but I'm too lazy
const float goertzel_thresh = 15;
// How long of a period to compute goertzel filter for
const float goertzel_win_ms = 100;

// Grace time after picking up piece before assuming placed back in same spot
const float debounce_ms = 1000;

/******************************************************************************
 * FFT/Detection
 ******************************************************************************/
const float freq_sample = 1500; //Hz, must be less than 10000 due to ADC
double freq_output[2] = { 500, 600 };

/******************************************************************************
 * Hardware Definitions
 ******************************************************************************/
// All taken from kicad schematic

const int n_sqr = 16;

int em_pins[n_sqr] = {
    4, 5, 6, 7,
    8, 9, 10, 11,
    12, 40, 41, 30,
    31, 32, 33, 36,
};

int sens_pins[n_sqr] = {
    14, 15, 16, 17,
    18, 19, 20, 21,
    22, 23, 24, 25,
    26, 27, 38, 39,
};
SemaphoreHandle_t adc_mut;

int freq_pin = 2;

const int led_pin = 1;

int jmp_pins[2] = { 37, 3 };

/*
 * Note: these are opposite to the definitions in the hardware design
 * This is because how I hooked up the pcbs physically happened to be opposite
 * to what I originally planned
 */
HardwareSerialIMXRT &serial_in = Serial8;
HardwareSerialIMXRT &serial_out = Serial7;

/******************************************************************************
 * ws2812Serial
 ******************************************************************************/
const int numled = 128;

byte drawingMemory[numled*3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled*12]; // 12 bytes per LED

WS2812Serial leds(numled, displayMemory, drawingMemory, led_pin, WS2812_GRB);
SemaphoreHandle_t leds_mut;

/******************************************************************************
 * FreeRTOS
 ******************************************************************************/

// Hacky, but arbitrary large negative ticks so don't have to wait at beginning
int neg_tick = -pdMS_TO_TICKS(10'000);


/******************************************************************************
 * Global variables
 ******************************************************************************/

// Which board this is, 0 is top left and increases counter clockwise
static unsigned int board_id;

/******************************************************************************
 * Helper functions
 ******************************************************************************/

uint32_t hsv_to_rgb(float h, float s, float v) {
    float r, g, b;

    int i = (int)(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }

    return ((int)(r * 255) << 16) | ((int)(g * 255) << 8) | (int)(b * 255);
}

uint32_t get_rainbow_color(int step) {
    float h = (step % 360) / 360.0;  // Cycle hue between 0 and 1
    float s = 1.0;                   // Full saturation
    float v = 0.2;                   // Adjust brightness here (0 to 1)

    return hsv_to_rgb(h, s, v);
}

void color_wipe(int color, int wait_ms) {
    for (int i=0; i < leds.numPixels(); i++) {
        leds.setPixel(i, color);
        leds.show();
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
    }
}

void color_spiral(int idx, int step){
    int id, x, y;
    int xx = 0, yy = 0;
    int a = idx % 32, b = idx / 32;
    if(board_id == 0){
        xx = b;
        yy = 3 - a;
    }else if(board_id == 1){
        xx = 3 - a;
        yy = 3 - b + 4;
    } else if(board_id == 2){
        xx = 3 - b + 4;
        yy = a + 4;
    } else if(board_id == 3){
        xx = a + 4;
        yy = b;
    }
    for (int layer = 0; layer < 4; layer++) {
        for (x = layer; x < 8 - layer; x++) {
            y = layer;
            for (id = 0; id < 8; id++) {
                if(xx == x && yy == y)
                    leds.setPixel(8*idx + id, get_rainbow_color(step + id));
            }
        }

        for (y = layer + 1; y < 8 - layer; y++) {
            x = 8 - layer - 1;
            for (id = 0; id < 8; id++) {
                if(xx == x && yy == y)
                    leds.setPixel(8*idx + id, get_rainbow_color(step + id));
            }
        }
        for (x = 8 - layer - 2; x >= layer; x--) {
            y = 8 - layer - 1;
            for (id = 0; id < 8; id++) {
                if(xx == x && yy == y)
                    leds.setPixel(8*idx + id, get_rainbow_color(step + id));
            }
        }
        for (y = 8 - layer - 2; y > layer; y--) {
            x = layer;
            for (id = 0; id < 8; id++) {
                if(xx == x && yy == y)
                    leds.setPixel(8*idx + id, get_rainbow_color(step + id));
            }
        }
    }

}

double calculate_ewma(double previous_ewma, double new_value, double alpha) {
    return alpha * new_value + (1 - alpha) * previous_ewma;
}

/******************************************************************************
 * Tasks
 ******************************************************************************/

static void blink_task(void*) {
    pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
    pinMode(jmp_pins[0], arduino::INPUT_PULLUP);
    pinMode(jmp_pins[1], arduino::INPUT_PULLUP);
    while (true) {
        digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
        vTaskDelay(pdMS_TO_TICKS(500));

        digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(500));
        /*Serial.println(board_id);*/
        /*Serial.print("JMP 1: ");*/
        /*Serial.println(digitalRead(jmp_pins[0]));*/
        /*Serial.print("JMP 2: ");*/
        /*Serial.println(digitalRead(jmp_pins[1]));*/
    }
}

static void led_task(void*) {
    for ever {
        if(xSemaphoreTake(leds_mut, pdMS_TO_TICKS(100))){
            /*leds.setPixel(0, WHITE);*/
            leds.show();
            xSemaphoreGive(leds_mut);
        } else {
            Serial.println("LED mutex failed!");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void square_task(void *params) {
    int idx = *((int*) params);
    unsigned int sampling_period_us = round(1000000*(1.0/freq_sample));

    pinMode(em_pins[idx], arduino::OUTPUT);
    digitalWrite(em_pins[idx], 0);

    // Test sequence
    /*for (int i=0; i < 8; i++) {*/
    /*    leds.setPixel(8*idx + i, WHITE);*/
    /*}*/
    /*for ever {}*/

    TickType_t last_print = xTaskGetTickCount();
    TickType_t last_led = xTaskGetTickCount();
    TickType_t last_wake = xTaskGetTickCount();

    unsigned int freq_ewma[2] = { 0, 0 };
    int last_touching[2] = { neg_tick, neg_tick };

    // Representation invariant: cooldown_start > 0 => last_plr != -1
    TickType_t cooldown_start = neg_tick;
    int last_plr = -1; // -1 for empty square

    unsigned int goertzel_win_samples = goertzel_win_ms * freq_sample / 1000;
    Goertzel goertzels[2] = { Goertzel(freq_output[0], freq_sample, goertzel_win_samples), Goertzel(freq_output[1], freq_sample, goertzel_win_samples) };

    // Loading screen
    TickType_t t_init = xTaskGetTickCount();
    int step = 0;
    while(xTaskGetTickCount() - t_init < pdMS_TO_TICKS(100'000)){
        color_spiral(idx, step);
        ++step;
        vTaskDelay(pdMS_TO_TICKS(100));
    }


    // Main task
    for ever {
        TickType_t cur_tk = xTaskGetTickCount();
        bool on_cooldown = cur_tk - cooldown_start <= cooldown_tk;

        // Only activate ems if on cooldown
        if(on_cooldown){
            digitalWrite(em_pins[idx], 1);
        } else {
            digitalWrite(em_pins[idx], 0);
        }

        // Sample adc
        float sample = 0;
        if(xSemaphoreTake(adc_mut, pdMS_TO_TICKS(100))){
            sample = analogRead(sens_pins[idx])*3.3/1024;
            xSemaphoreGive(adc_mut);
        }

        for(int p = 0; p < 2; ++p){
            // Update goertzel/ewma
            goertzels[p].addSample(sample);
            freq_ewma[p] = calculate_ewma(freq_ewma[p], goertzels[p].getMagnitude(), 0.3);

            // Whether been touched by this plr recently
            bool prev_touching = cur_tk - last_touching[p] < pdMS_TO_TICKS(debounce_ms);

            if(freq_ewma[p] >= goertzel_thresh){
                // New touch from empty
                if(last_plr == -1 && !prev_touching){
                    cooldown_start = cur_tk;
                    last_plr = p;
                // Reset square
                } else if(last_plr == p && !on_cooldown){
                    last_plr = -1;
                // Override square (for capture)
                } else if(last_plr == !p){
                    last_plr = -1;
                    cooldown_start = neg_tick;
                }

                last_touching[p] = cur_tk;
            }
        }

        /*if(cur_tk - last_print >= pdMS_TO_TICKS(500) && idx == 0){*/
        /*    Serial.print(idx);*/
        /*    Serial.print(", ");*/
        /*    Serial.print(goertzels[0].getMagnitude());*/
        /*    Serial.print(", ");*/
        /*    Serial.print(goertzels[1].getMagnitude());*/
        /*    Serial.print(", ");*/
        /*    Serial.println(sample);*/
        /*    last_print = cur_tk;*/
        /*}*/

        if(cur_tk - last_led >= pdMS_TO_TICKS(50)){
            if(xSemaphoreTake(leds_mut, pdMS_TO_TICKS(100))){
                int p = last_plr;
                for (int i=0; i < 8; i++) {
                    // Cooldown is bright
                    if(on_cooldown){
                        leds.setPixel(8*idx + i, i < 8.0*(cur_tk - cooldown_start)/cooldown_tk - 0.8 ? OFF : plr_clrs_lock[p]);
                    // 
                    } else if(last_plr != -1){
                        leds.setPixel(8*idx + i, plr_clrs_unlock[p]);
                    } else {
                        leds.setPixel(8*idx + i, OFF);
                    }
                    /*leds.setPixel(8*idx + i, WHITE);*/
                }
                xSemaphoreGive(leds_mut);
            } else Serial.println("Failed to take led mutex!");
            last_led = cur_tk;
        }

        if(!xTaskDelayUntil(&last_wake, pdUS_TO_TICKS(sampling_period_us))){
            // This actually gets called pretty often if you try playing around with sample rate/tick rate
            // (=>) Worth keeping an eye on the serial monitor if detection isn't working
            Serial.println("Square task not actually delayed (computation/adc reads probably taking too long)");
        }
    }
}

static void freq_output_task(void *params) {
    int idx = *((int*) params);

    TickType_t last_wake = xTaskGetTickCount();
    uint8_t output_state = 0;

    pinMode(freq_pin, arduino::OUTPUT);

    for ever {
        output_state = !output_state;
        digitalWrite(freq_pin, output_state);
        vTaskDelayUntil(&last_wake, pdUS_TO_TICKS(1e6/freq_output[idx]/2));
    }
}

static void serial_task(void *params) {
    (void) params;

    for ever {
        if(board_id == 0){
            serial_out.println("Hello world");
        } else {
            // Forward message
            while(serial_in.available()){
                int incomingByte = serial_in.read();
                serial_out.write(incomingByte);
                Serial.write(incomingByte);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/******************************************************************************
 * Arduino Setup/Loop
 ******************************************************************************/

FLASHMEM __attribute__((noinline))
void setup() {
    Serial.begin(115200);
    serial_in.begin(115200);
    serial_out.begin(115200);
    delay(200);

    pinMode(jmp_pins[0], arduino::INPUT_PULLUP);
    pinMode(jmp_pins[1], arduino::INPUT_PULLUP);

    board_id = !digitalRead(jmp_pins[0]) + 2 * !digitalRead(jmp_pins[1]);

    if (CrashReport) {
        Serial.print(CrashReport);
        Serial.println();
        Serial.flush();
    }

    Serial.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));


    leds_mut = xSemaphoreCreateMutex();
    adc_mut = xSemaphoreCreateMutex();

    xTaskCreate(blink_task, "blink_task", 1024, nullptr, 2, nullptr);

    xTaskCreate(led_task, "led_task", 1024, nullptr, 2, nullptr);

    static int freq_idx = board_id == 1 || board_id == 2 ? 0 : 1;
    /*static int freq_idx = 0;*/
    xTaskCreate(freq_output_task, "freq_output_task", 1024, &freq_idx, 2, nullptr);

    static int square_idx[n_sqr];
    for(int i = 0; i < 16; ++i){
        square_idx[i] = i;
        xTaskCreate(square_task, "square_task", 2048, &square_idx[i], 2, nullptr);
    }

    xTaskCreate(serial_task, "serial_task", 1024, nullptr, 2, nullptr);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();
    leds.begin();

    vTaskStartScheduler();
}

void loop() {}

