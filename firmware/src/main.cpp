
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

#define INTENSE_RED    0x990000
#define INTENSE_GREEN  0x009900
#define INTENSE_BLUE   0x000099
#define INTENSE_YELLOW 0x999900
#define INTENSE_PINK   0x991088
#define INTENSE_ORANGE 0xE05800
#define INTENSE_WHITE  0x999999

// Less intense...
#define RED    0x160000
#define GREEN  0x001600
#define BLUE   0x000016
#define YELLOW 0x101400
#define PINK   0x120009
#define ORANGE 0x100600
#define PURPLE 0x100020
#define WHITE  0x101010
#define OFF    0x000000

#define GOLD   0xAE8625
#define SILVER   0xC0C0C0

const uint32_t cooldown_choices_ms[] = {3'000, 7'000, 12'000};
static volatile uint32_t cooldown_ms = cooldown_choices_ms[1];
int plr_clrs_lock[2] = {INTENSE_RED, INTENSE_BLUE};
/*int plr_clrs_lock[2] = {GOLD, SILVER};*/
/*int plr_clrs_unlock[2] = {RED, BLUE};*/
/*int plr_clrs_unlock[2] = {ORANGE, ORANGE};*/
int plr_clrs_unlock[2] = {PINK, PINK};

// Threshold for ewma of goertzel filter before considered touching
// Not sure if goertzel filter isn't scale invariant to e.g. window size,
// freq, etc. If not should probably be scaled appropriately but I'm too lazy
const float goertzel_thresh = 10;
// How long of a period to compute goertzel filter for
const float goertzel_win_ms = 100;

// Grace time after picking up piece before assuming placed back in same spot
const float debounce_ms = 500;

const bool upside_down_mode = false;

/******************************************************************************
 * Structs
 ******************************************************************************/
typedef struct coord_t {
  int32_t rank;
  int32_t file;
  float x_cm = 0;
  float y_cm = 0;
} coord_t;

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

/* int em_pins[n_sqr] = { */
/*     4, 5, 6, 7, */
/*     8, 9, 10, 11, */
/*     0, 40, 41, 30, */
/*     31, 32, 33, 36, */
/* }; */

int sens_pins[n_sqr] = {
  14, 15, 16, 17,
  18, 19, 20, 21,
  22, 23, 24, 25,
  26, 27, 38, 39,
};

const float square_cm = 5.0;
const float square_center_to_led_cm = 1.8;
const coord_t coord_center = {
  .rank = -1,
  .file = -1,
  .x_cm = 4 * square_cm,
  .y_cm = 4 * square_cm,
};

int freq_pin = 2;

const int led_pin = 1;

int jmp_pins[2] = { 37, 3 };

/*
 * Note: these are opposite to the definitions in the hardware design
 * This is because how I hooked up the pcbs physically happened to be opposite
 * to what I originally planned
 */
HardwareSerialIMXRT &serial_prev = Serial8;
HardwareSerialIMXRT &serial_next = Serial7;

/******************************************************************************
 * ws2812Serial
 ******************************************************************************/
const int num_leds = 128;

byte drawingMemory[num_leds*3];         //  3 bytes per LED
DMAMEM byte displayMemory[num_leds*12]; // 12 bytes per LED

WS2812Serial leds(num_leds, displayMemory, drawingMemory, led_pin, WS2812_GRB);
SemaphoreHandle_t leds_mut;

/******************************************************************************
 * FreeRTOS
 ******************************************************************************/

#define n_cooldown_change (1 << 0)

// Hacky, but arbitrary large negative ticks so don't have to wait at beginning
int neg_tick = -pdMS_TO_TICKS(10'000);

// Semaphores
SemaphoreHandle_t adc_mut;

// Task handles
TaskHandle_t serial_handle;

/******************************************************************************
 * Inter board communication
 ******************************************************************************/

enum board_msg {
  m_cooldown_change = 'c', // format: "cX\n", X is binary encoded cooldown in ms
};

/******************************************************************************
 * Global variables
 ******************************************************************************/

// Which board this is, 0 is top left and increases counter clockwise
static unsigned int board_id;
static volatile int cooldown_selection = 1;

extern const uint32_t num_frames;
extern const float frame_rate;
extern const uint32_t frames[1000][8][8][8];

typedef enum board_mode_t {
  PLAY,
  INTRO,
  VIDEO,
  SETTINGS,
} board_mode_t;

static volatile board_mode_t board_mode = INTRO;

/******************************************************************************
 * Helper functions
 ******************************************************************************/

// idx is index of square on board, i is pixel
// i=-1 => center of square
coord_t idx_to_coord(uint32_t idx, int32_t i = -1){
  coord_t ret;
  float dx = 0, dy = 0;
  if(i != -1){
    float led_theta = i * 2.0 * PI / 8.0;
    dx = -square_center_to_led_cm * cos(led_theta);
    dy = -square_center_to_led_cm * sin(led_theta);
  }
  float old_dx = dx;
  switch(board_id) {
    case 0:
      ret.rank = (idx % 4) + 4;
      ret.file = idx / 4;
      break;
    case 1:
      ret.rank = (idx / 4);
      ret.file = 3 - (idx % 4);
      dx = -dy;
      dy = old_dx;
      break;
    case 2:
      ret.rank = 3 - (idx % 4);
      ret.file = 7 - (idx / 4);
      dx = - dx;
      dy = -dy;
      break;
    case 3:
      ret.rank = 7 - (idx / 4);
      ret.file = 4 + (idx % 4);
      dx = dy;
      dy = -old_dx;
      break;
  }
  ret.x_cm = ret.file * square_cm + dx + square_cm / 2.0;
  ret.y_cm = ret.rank * square_cm + dy + square_cm / 2.0;

  /*if(ret.file == ret.rank && i == 0){*/
  /*    float led_theta = i * 2.0 * PI / 8.0;*/
  /*    dx = -square_center_to_led_cm * cos(led_theta);*/
  /*    dy = -square_center_to_led_cm * sin(led_theta);*/
  /*    Serial.print("dx: ");*/
  /*    Serial.print(dx);*/
  /*    Serial.print(", dy: ");*/
  /*    Serial.print(dy);*/
  /*    Serial.println();*/
  /*}*/

  return ret;
}

uint32_t change_intensity(uint32_t col, float mult){
  uint32_t red = ((col >> 16) & 0xFF) * mult;
  uint32_t green = ((col >> 8) & 0xFF) * mult;
  uint32_t blue = (col & 0xFF) * mult;
  return (red << 16) | (green << 8) | blue;
}

uint32_t norm_intensity(uint32_t col, float norm){
  uint32_t r = ((col >> 16) & 0xFF);
  uint32_t g = ((col >> 8) & 0xFF);
  uint32_t b = (col & 0xFF);
  float brightness = 0.2126 * r + 0.7152 * g + 0.0722 * b;
  float mult = norm / max(1, brightness);
  r *= mult;
  g *= mult;
  b *= mult;
  float gamma = 2.2;
  r = int(pow(r / 255.0, gamma) * 255);
  g = int(pow(g / 255.0, gamma) * 255);
  b = int(pow(b / 255.0, gamma) * 255);
  return (r << 16) | (g << 8) | b;
}

uint32_t hsv_to_rgb(float h, float s, float v, float fade) {
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
    default:
    case 5: r = v, g = p, b = q; break;
  }
  float reduction = 0.25 * fade;
  r *= reduction;
  g *= reduction;
  b *= reduction;

  return ((int)(r * 255) << 16) | ((int)(g * 255) << 8) | (int)(b * 255);
}

uint32_t get_rainbow_color(float h, float fade) {
  /*float h = (step % 360) / 360.0;  // Cycle hue between 0 and 1*/
  float s = 1.0;                   // Full saturation
  float v = 0.3;                   // Adjust brightness here (0 to 1)

  return hsv_to_rgb(h, s, v, fade);
}

void color_wipe(int color, int wait_ms) {
  for (int i=0; i < leds.numPixels(); i++) {
    leds.setPixel(i, color);
    leds.show();
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
  }
}

void color_spiral(int idx, int step, int max_step){

  for(int i = 0; i < 8; ++i){
    coord_t coord = idx_to_coord(idx, i);

    float theta = atan2(coord.y_cm - coord_center.y_cm, coord.x_cm - coord_center.x_cm);// + 1.5 * PI;
    float time_adjust = (atan((step-max_step/2.0)*2/max_step * 3)-atan(-3))*max_step/500;
    float fade = min(1.0, 4.0*(max_step-step)/max_step);
    leds.setPixel(8*idx + i, get_rainbow_color(fmod(theta / 2.0 / PI + time_adjust + 1000, 1), fade));

    /*if(coord.file == 0 && coord.rank == 3){*/
    /*    Serial.print("file: ");*/
    /*    Serial.print(coord.file);*/
    /*    Serial.print(", rank: ");*/
    /*    Serial.print(coord.rank);*/
    /*    Serial.print(", i:");*/
    /*    Serial.print(i);*/
    /*    Serial.print(", x_cm: ");*/
    /*    Serial.print(coord.x_cm);*/
    /*    Serial.print(", y_cm: ");*/
    /*    Serial.print(coord.y_cm);*/
    /*    Serial.print(", ");*/
    /*    Serial.print(theta);*/
    /*    Serial.print(", ");*/
    /*    Serial.println(((unsigned int)(180.0/PI*theta / 3.0 + 1000 + 45) % 180) / 180.0);*/
    /*}*/
  }

}

void play_video(int idx, int frame){
  /*frame = min(frame, num_frames-1);*/
  frame = frame % num_frames;
  for(int i = 0; i < 8; ++i){
    coord_t coord = idx_to_coord(idx, i);

    leds.setPixel(8*idx + i, norm_intensity(frames[frame][coord.file][coord.rank][i], 60));
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
    if(xSemaphoreTake(leds_mut, pdMS_TO_TICKS(20))){
      /*leds.setPixel(0, WHITE);*/
      leds.show();
      xSemaphoreGive(leds_mut);
    } else {
      Serial.println("LED mutex failed!");
    }
    vTaskDelay(pdMS_TO_TICKS(50));
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
  coord_t coord = idx_to_coord(idx);
  if(coord.rank <= 1)
    last_plr = 0;
  else if(coord.rank >= 6)
    last_plr = 1;
  /* if((coord.rank == 0 || coord.rank == 7) && (coord.file == 0 || coord.file == 7)) */
  /*     last_plr = 0; */


  unsigned int goertzel_win_samples = goertzel_win_ms * freq_sample / 1000;
  Goertzel goertzels[2] = { Goertzel(freq_output[0], freq_sample, goertzel_win_samples), Goertzel(freq_output[1], freq_sample, goertzel_win_samples) };


  // Loading screen
  TickType_t t_init = xTaskGetTickCount();
  int step = 0;
  int intro_delay_ms = 10;
  int intro_ms = 10'000;
  /* int intro_ms = 10'000; */
  /*int intro_ms = 100000'000;*/
  int ui_ms = 10'000'000;


  // Main task
  for ever {
    TickType_t cur_tk = xTaskGetTickCount();

    bool on_cooldown = cur_tk - cooldown_start <= pdMS_TO_TICKS(cooldown_ms);

    // Only activate ems if on cooldown
    if(on_cooldown || (upside_down_mode && last_plr != -1)){
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

      // Make sure to check that this frequency is actually dominant, as there
      // can be overlap
      if(freq_ewma[p] >= goertzel_thresh && freq_ewma[p] > freq_ewma[!p]){
        switch(board_mode) {
          case INTRO:
          case VIDEO:
            board_mode = PLAY;
            break;
          case PLAY:
            {
              // New touch from empty
              if(last_plr == -1 && !prev_touching){
                last_plr = p;
                cooldown_start = cur_tk;
                // Reset square
              } else if(last_plr == p && !on_cooldown){
                last_plr = -1;
                // Override square (for capture)
              } else if(last_plr == !p){
                last_plr = -1;
                cooldown_start = neg_tick;
              }
            }
            break;
          case SETTINGS:
            break;
        }

        last_touching[p] = cur_tk;
      }
    }

    /* if(cur_tk - last_print >= pdMS_TO_TICKS(500) && (idx == 10 || idx == 11)){ */
    /*     Serial.print(idx); */
    /*     Serial.print(", "); */
    /*     Serial.print(goertzels[0].getMagnitude()); */
    /*     Serial.print(", "); */
    /*     Serial.print(goertzels[1].getMagnitude()); */
    /*     Serial.print(", "); */
    /*     Serial.println(sample); */
    /*     last_print = cur_tk; */
    /* } */


    switch(board_mode) {
      case PLAY:
      {
        if(cur_tk - last_led >= pdMS_TO_TICKS(50)){
          if(xSemaphoreTake(leds_mut, pdMS_TO_TICKS(100))){
            int p = last_plr;
            for (int i=0; i < 8; i++) {
              // Cooldown is bright
              if(on_cooldown){
                float fraction_through = (cur_tk - cooldown_start) * 1. /pdMS_TO_TICKS(cooldown_ms);
                float min_intensity = 0.05;
                if(i == (int) (fraction_through * 8)){
                  /*float intensity_modifier = (1-(fraction_through - i / 8.) * 8) * (1 - min_intensity) + min_intensity;*/
                  float intensity_modifier = (1-(fraction_through - i / 8.) * 8) * (1 - min_intensity) + min_intensity;
                  leds.setPixel(8*idx + i, change_intensity(plr_clrs_lock[p], intensity_modifier));
                }else if( i > (int) (fraction_through * 8)){
                  leds.setPixel(8*idx + i, plr_clrs_lock[p]);
                } else {
                  leds.setPixel(8*idx + i, change_intensity(plr_clrs_lock[p], min_intensity));
                }
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
        break;
      }
      case SETTINGS:
        break;
      case VIDEO:
        break;
      case INTRO:
      {
        if(xTaskGetTickCount() - last_wake > pdMS_TO_TICKS(intro_delay_ms)){
          leds.setPixel(8*idx + 0, WHITE);
          /* color_spiral(idx, step, intro_ms/intro_delay_ms); */
          /* play_video(idx, (int)(pdTICKS_TO_MS(xTaskGetTickCount() - t_init) / 1000.0 * frame_rate)); */
          ++step;
          last_wake = xTaskGetTickCount();
        }
        if(xTaskGetTickCount() - t_init > pdMS_TO_TICKS(intro_ms)){
          board_mode = PLAY;
        }
        break;
      }
    }


    // Read cooldown interface
    /*if(board_id == 0 && cur_tk - t_init < pdMS_TO_TICKS(intro_ms + ui_ms)){*/
    /*    int clrs[] = {RED, YELLOW, GREEN, WHITE};*/
    /*    int clrs_intense[] = {INTENSE_RED, INTENSE_YELLOW, INTENSE_GREEN, INTENSE_WHITE};*/
    /**/
    /*    bool cur_touching = last_touching[0] == (int)cur_tk || last_touching[1] == (int)cur_tk;*/
    /**/
    /*    if(idx % 4 == 0){*/
    /*        int i = idx/4;*/
    /*        // No cooldown if ui button*/
    /*        cooldown_start = neg_tick;*/
    /*        if(cur_touching && i < 3){*/
    /*            cooldown_selection = i;*/
    /*            cooldown_ms = cooldown_choices_ms[i];*/
    /*            xTaskNotify(serial_handle, n_cooldown_change, eSetBits);*/
    /*        }*/
    /*        if(xSemaphoreTake(leds_mut, pdMS_TO_TICKS(100))){*/
    /*            for(int j = 0; j < 8; ++j){*/
    /*                leds.setPixel(8*idx + j, cooldown_selection == i ? clrs_intense[i] : clrs[i]);*/
    /*            }*/
    /*            xSemaphoreGive(leds_mut);*/
    /*        } else Serial.println("Failed to take led mutex!");*/
    /*    }*/
    /*}*/


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

  char prev_msg[100];
  uint32_t prev_msg_len = 0;

  for ever {
    // Check for notifications
    uint32_t notification;
    if(xTaskNotifyWait(0, 0xFFFFFFFF, &notification, 0)){
      Serial.println("Got notification!");
      if(notification & n_cooldown_change){
        Serial.println("Cooldown notification!");
        serial_next.write(m_cooldown_change);
        serial_next.write((cooldown_ms >> 24) & 0xFF);
        serial_next.write((cooldown_ms >> 16) & 0xFF);
        serial_next.write((cooldown_ms >> 8) & 0xFF);
        serial_next.write(cooldown_ms & 0xFF); 
        serial_next.println();
      }
    }
    if(board_id == 0){
      /*serial_next.println("Hello world");*/
    } else {
      // Forward message
      while(serial_prev.available()){
        unsigned char in_byte = serial_prev.read();
        serial_next.write(in_byte);

        if(prev_msg_len + 1 < sizeof prev_msg / sizeof prev_msg[0])
          prev_msg[prev_msg_len++] = in_byte;

        if(in_byte == '\n'){
          switch(prev_msg[0]) {
            case m_cooldown_change:
              Serial.println("Changing cooldown");
              if(prev_msg_len >= 5)
                cooldown_ms = prev_msg[1] << 24 |
                  prev_msg[2] << 16 |
                  prev_msg[3] << 8 |
                  prev_msg[4];
              break;
            default:
              break;
          }
          prev_msg_len = 0;
        }
        Serial.write(in_byte);
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
    serial_prev.begin(115200);
    serial_next.begin(115200);
    delay(200);

    pinMode(jmp_pins[0], arduino::INPUT_PULLUP);
    pinMode(jmp_pins[1], arduino::INPUT_PULLUP);

    board_id = !digitalRead(jmp_pins[0]) + 2 * !digitalRead(jmp_pins[1]);

    // Ugly hack for one of the boards I made, still not sure why I had to change this out
    /* pinMode(em_pins[8], arduino::INPUT_PULLDOWN); */
    /* if(board_id == 1 && digitalRead(em_pins[8])){ */
    /*     em_pins[8] = 0; */
    /* } */

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
    xTaskCreate(freq_output_task, "freq_output_task", 1024, &freq_idx, 2, nullptr);

    static int square_idx[n_sqr];
    for(int i = 0; i < 16; ++i){
      square_idx[i] = i;
      xTaskCreate(square_task, "square_task", 2048, &square_idx[i], 2, nullptr);
    }

    xTaskCreate(serial_task, "serial_task", 1024, nullptr, 2, &serial_handle);

    Serial.println("setup(): starting scheduler...");
    Serial.flush();




    /*delay(50*board_id + 50);*/
    leds.begin();
    /*delay(200 - 50*board_id);*/

    for(int i = 0; i < 128; ++i){
      leds.setPixel(i, OFF);
    }

    vTaskStartScheduler();
  }

void loop() {}

