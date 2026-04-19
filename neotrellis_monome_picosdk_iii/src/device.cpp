/*
 * device.cpp - NeoTrellis device implementation for iii (PALETTED EDITION V4 - ULTIMATE)
 * FORENSICALLY CORRECTED ENGINE WITH HARDWARE ANTI-CRUSH
 */

#include "MonomeSerialDevice.h"
#include "Adafruit_seesaw/Adafruit_NeoTrellis.h"
#include "config.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include <string.h>

extern "C" {
#include "device.h"
#include "device_ext.h"
#include "serial.h"
#include "util.h"
#include "flash.h"
}

extern uint8_t mode;
extern uint8_t g_monome_mode;

#if GRIDCOUNT == SIXTEEN
Adafruit_NeoTrellis trellis_array[NUM_ROWS / 4][NUM_COLS / 4] = {
    {Adafruit_NeoTrellis(addrRowOne[0])}};
#endif

#if GRIDCOUNT == SIXTYFOUR
Adafruit_NeoTrellis trellis_array[NUM_ROWS / 4][NUM_COLS / 4] = {
    {Adafruit_NeoTrellis(addrRowOne[0]), Adafruit_NeoTrellis(addrRowOne[1])},
    {Adafruit_NeoTrellis(addrRowTwo[0]), Adafruit_NeoTrellis(addrRowTwo[1])}};
#endif

#if GRIDCOUNT == ONETWENTYEIGHT
Adafruit_NeoTrellis trellis_array[NUM_ROWS / 4][NUM_COLS / 4] = {
    {Adafruit_NeoTrellis(addrRowOne[0]), Adafruit_NeoTrellis(addrRowOne[1]),
     Adafruit_NeoTrellis(addrRowOne[2]), Adafruit_NeoTrellis(addrRowOne[3])},
    {Adafruit_NeoTrellis(addrRowTwo[0]), Adafruit_NeoTrellis(addrRowTwo[1]),
     Adafruit_NeoTrellis(addrRowTwo[2]), Adafruit_NeoTrellis(addrRowTwo[3])}};
#endif

Adafruit_MultiTrellis trellis((Adafruit_NeoTrellis *)trellis_array, NUM_ROWS / 4, NUM_COLS / 4);

MonomeSerialDevice mdp;
static int prevLedBuffer[MonomeSerialDevice::MAXLEDCOUNT];

static uint8_t local_leds[NUM_ROWS * NUM_COLS];
static uint8_t mmap[NUM_ROWS * NUM_COLS];
static bool    grid_dirty = false;

#define GRID_EVENTS_MAX 64
static uint8_t grid_event_count = 0;
static struct {
    uint8_t x;
    uint8_t y;
    uint8_t z;
} grid_events[GRID_EVENTS_MAX];

static void grid_add_event(uint8_t x, uint8_t y, uint8_t z) {
    if (grid_event_count < GRID_EVENTS_MAX) {
        grid_events[grid_event_count].x = x;
        grid_events[grid_event_count].y = y;
        grid_events[grid_event_count].z = z;
        grid_event_count++;
    }
}

// ===========================================================================
// INYECCIÓN FORENSE: MATRIZ EXACTA + PALETAS MEJORADAS
// ===========================================================================

static const uint8_t allpalettes[25][3][16] = {
  // 0 a 14: EXACTAMENTE IGUALES A SU .INO ORIGINAL
  {{0,45,55,65,75,85,96,107,118,133,152,171,190,210,230,250},{0,45,55,65,75,85,96,107,118,133,152,171,190,210,230,250},{0,45,55,65,75,85,96,107,118,133,152,171,190,210,230,250}},
  {{0,43,52,66,80,95,110,125,140,155,170,185,200,215,230,250},{0,43,52,66,80,95,110,125,140,155,170,185,200,215,230,250},{0,22,31,33,40,48,55,62,70,77,85,92,100,107,115,125}},
  {{0,22,31,33,40,48,55,62,70,77,85,92,100,107,115,125},{0,43,52,66,80,95,110,125,140,155,170,185,200,215,230,250},{0,43,52,66,80,95,110,125,140,155,170,185,200,215,230,250}},
  {{0, 105, 115, 153, 189, 223 ,252, 255, 255, 255, 255, 255, 255, 255, 255, 255},{0, 43, 61, 89, 115, 140, 163, 184, 201, 214, 224, 231, 236, 238 ,239 ,240},{0, 0, 0, 0, 2, 5, 11, 20, 32, 50, 72, 97, 125, 156, 187, 220}},
  {{0, 77, 89, 101, 107, 124, 135, 146, 156, 166, 175, 184, 193, 201 ,210 ,218},{0, 21, 24, 28, 33, 39 ,46, 54, 64, 76, 90, 105, 122, 140, 158, 177},{0, 4, 5, 6, 7, 8, 9, 11, 15, 25, 39, 57, 78, 101, 126, 152}},
  {{0, 10 ,11, 12, 13, 14, 15, 16, 20, 25, 39, 57, 78, 101, 126, 152},{0, 45, 51, 58, 66, 75, 84, 94, 105, 116, 128, 141, 155, 168 ,182 ,196},{0, 83, 89, 101, 113, 124, 135, 146, 156, 166, 175, 184, 193, 201, 210, 218}},
  {{0, 21, 24, 28, 33, 39 ,46, 54, 64, 76, 90, 105, 122, 140, 158, 177},{0, 77, 89, 101, 107, 124, 135, 146, 156, 166, 175, 184, 193, 201 ,210 ,218},{0, 4, 5, 6, 7, 8, 9, 11, 15, 25, 39, 57, 78, 101, 126, 152}},
  {{0,43,52,66,80,95,110,125,140,155,170,185,200,215,230,250},{0,22,31,33,40,48,55,62,70,77,85,92,100,107,115,125},{0,43,52,66,80,95,110,125,140,155,170,185,200,215,230,250}},
  {{0, 24, 30, 47, 62, 79, 96, 111, 127, 143, 159, 175, 191, 207, 223, 240},{0, 15, 25, 39, 64, 86, 107, 129, 150, 169, 188, 200, 211, 224, 236, 244},{77, 101, 128, 132, 136, 138, 139, 140, 140, 149, 158, 175, 191, 209, 224, 237}},
  {{0, 10 ,11, 12, 13, 14, 15, 16, 20, 25, 35, 45, 55, 65, 75, 88},{100, 115, 133, 150, 159, 167, 175, 180, 185, 189, 197, 205, 213, 220, 230, 240},{102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102}},
  {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},{0, 14, 30, 47, 62, 79, 96, 111, 127, 143, 159, 175, 191, 207, 223, 240},{255, 247, 239, 231, 223, 215, 207, 199, 190, 183, 175, 167, 159, 150, 143, 135}},
  {{0, 14, 30, 47, 62, 79, 96, 111, 127, 143, 159, 175, 191, 207, 223, 240},{127, 135, 143, 150, 159, 167, 175, 183, 191, 199, 207, 215, 223, 231, 239, 247},{102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102}},
  {{0, 14, 30, 47, 62, 79, 96, 111, 127, 143, 159, 175, 191, 207, 223, 240},{127, 135, 143, 150, 159, 167, 175, 183, 191, 199, 207, 215, 223, 231, 239, 247},{0, 14, 30, 37, 42, 59, 66, 71, 87, 93, 109, 115, 121, 127, 127, 127}},
  {{107, 115, 123, 130, 139, 147, 155, 163, 171, 179, 187, 195, 203, 211, 229, 247},{0, 14, 30, 37, 42, 75, 96, 111, 127, 143, 159, 175, 191, 207, 223, 240},{30, 30, 30, 30, 30, 40, 50, 60, 67, 73, 89, 95, 101, 102, 102, 103}},
  {{0,43,52,66,80,95,110,125,140,155,170,185,200,215,230,250},{0,43,52,66,80,95,110,125,140,155,170,185,200,215,230,250},{0, 4, 5, 6, 7, 8, 9, 11, 15, 25, 39, 57, 78, 101, 126, 152}},
  
  // 15: Rojo fuerte a Rosa Claro (Mejorada)
  {{0, 100, 120, 140, 160, 180, 200, 215, 230, 240, 250, 255, 255, 255, 255, 255},{0, 0, 0, 0, 0, 10, 25, 45, 65, 85, 110, 135, 160, 185, 210, 230},{0, 0, 0, 5, 15, 30, 50, 70, 90, 110, 135, 160, 185, 210, 230, 245}},
  // 16: Rojo -> Naranja -> Warm White (Mejorada)
  {{0, 120, 140, 160, 180, 200, 220, 240, 255, 255, 255, 255, 255, 255, 255, 255},{0, 0, 10, 25, 45, 65, 85, 105, 125, 145, 165, 185, 200, 215, 230, 245},{0, 0, 0, 0, 0, 0, 0, 0, 10, 30, 50, 75, 100, 130, 160, 190}},
  // 17: Blanco Azulado (Mejorada)
  {{0, 5, 15, 30, 50, 70, 90, 110, 130, 150, 170, 190, 210, 225, 240, 255},{0, 25, 45, 65, 85, 105, 125, 145, 165, 185, 205, 220, 235, 245, 255, 255},{0, 60, 80, 100, 120, 140, 160, 180, 200, 215, 230, 240, 250, 255, 255, 255}},
  
  // 18 y 19: ORIGINALES
  {{0, 5, 15, 27, 40, 51, 58, 66, 92, 125, 152, 178, 188, 201, 219, 237},{0, 17, 55, 87, 114, 132, 141, 150, 160, 169, 175, 182, 170, 166, 182, 212},{0, 117, 120, 123, 126, 118, 99, 79, 75, 83, 88, 93, 98, 119, 159, 206}},
  {{0, 46, 57, 70, 93, 114, 136, 159, 184, 206, 228, 251, 255, 255, 255, 255},{0, 58, 72, 95, 110, 124, 139, 152, 164, 177, 191, 199, 208, 212, 213, 226},{0, 53, 54, 73, 77, 82, 92, 103, 114, 127, 151, 173, 195, 214, 227, 247}},
  
  // 20: Invertida -> Ahora normal, brillo ascendente (Mejorada)
  {{0, 10, 20, 35, 50, 70, 90, 110, 130, 150, 170, 190, 210, 230, 245, 255},{0, 5, 15, 30, 45, 65, 85, 105, 125, 145, 165, 185, 205, 225, 240, 255},{0, 40, 60, 80, 100, 120, 140, 160, 180, 200, 215, 230, 240, 250, 255, 255}},
  // 21: Fire / Fuego con brillo marcado (Mejorada)
  {{0, 70, 100, 130, 160, 190, 220, 240, 255, 255, 255, 255, 255, 255, 255, 255},{0, 0, 0, 10, 25, 45, 70, 95, 120, 145, 170, 195, 215, 235, 245, 255},{0, 0, 0, 0, 0, 0, 0, 0, 10, 30, 50, 80, 110, 150, 200, 255}},
  // 22: Synthwave con brillo marcado (Mejorada)
  {{0, 25, 45, 70, 100, 130, 160, 190, 215, 235, 250, 255, 255, 255, 255, 255},{0, 0, 0, 5, 15, 30, 50, 75, 100, 130, 160, 190, 215, 235, 250, 255},{0, 70, 100, 130, 160, 190, 215, 235, 250, 255, 255, 255, 255, 255, 255, 255}},
  // 23: Frio a Caliente con brillo marcado (Mejorada)
  {{0, 10, 25, 45, 70, 100, 130, 160, 190, 215, 235, 250, 255, 255, 255, 255},{0, 0, 5, 15, 30, 50, 70, 90, 110, 130, 150, 170, 190, 210, 230, 255},{0, 70, 100, 120, 140, 150, 140, 120, 90, 60, 30, 10, 0, 0, 50, 150}},
  
  {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}
};

// ESTADO POR DEFECTO: Escala 12 (Index 12), Nivel 4 (Fila 3)
static uint8_t selected_palette = 12;
static uint8_t current_page = 1; 
static uint8_t dynamic_gamma_table[16];

static bool key_13_7 = false;
static bool key_14_7 = false;
static bool key_15_7 = false;
static bool palette_ui_active = false;
static bool palette_preview_active = false;
static uint32_t palette_preview_start = 0;

// LA FÓRMULA MATEMÁTICA EXACTA DE SU .INO ORIGINAL
static void set_dynamic_gamma(uint8_t row) {
    float val_f = (float)row;
    for (int i = 0; i < 16; i++) {
        dynamic_gamma_table[i] = (uint8_t)( (((8.0 - val_f) / 8.0) * 255.0 / 15.0) * i );
    }
}

// FUNCIÓN MAESTRA: Calcula el color y aplica el Anti-Crush de Hardware
static void get_color_for_level(uint8_t pal_idx, uint8_t val, uint32_t *r_out, uint32_t *g_out, uint32_t *b_out) {
    if (val == 0) {
        *r_out = 0; *g_out = 0; *b_out = 0;
        return;
    }

    uint32_t r = allpalettes[pal_idx][0][val];
    uint32_t g = allpalettes[pal_idx][1][val];
    uint32_t b = allpalettes[pal_idx][2][val];
    uint32_t gam = dynamic_gamma_table[val];

    // El aplastamiento cuadrático original que genera su contraste
    r = (r * gam) / 255;
    g = (g * gam) / 255;
    b = (b * gam) / 255;

    // ANTI-CRUSH DE HARDWARE (Para BRIGHTNESS 96)
    // La librería Adafruit hace: (color * 96) >> 8.
    // Si color es 1 o 2, el resultado es 0 (Apagado).
    // Por lo tanto, el mínimo absoluto para que un LED encienda es 3.
    if (allpalettes[pal_idx][0][val] > 0 && r < 3) r = 3;
    if (allpalettes[pal_idx][1][val] > 0 && g < 3) g = 3;
    if (allpalettes[pal_idx][2][val] > 0 && b < 3) b = 3;

    *r_out = r;
    *g_out = g;
    *b_out = b;
}

static inline uint32_t level_to_color(uint8_t val) {
    uint32_t r, g, b;
    get_color_for_level(selected_palette, val, &r, &g, &b);
    return (r << 16) | (g << 8) | b;
}

static void draw_palette_ui() {
    // Columna 0: Muestra los niveles de brillo
    for(int y=0; y<7; y++){
        float val_f = (float)y;
        uint8_t gam_val = (uint8_t)( (((8.0 - val_f) / 8.0) * 255.0 / 15.0) * 15 );
        if (gam_val > 0 && gam_val < 3) gam_val = 3; // Anti-crush para el menú
        trellis.setPixelColor(y * NUM_COLS, (gam_val << 16) | (gam_val << 8) | gam_val);
    }
    trellis.setPixelColor(7 * NUM_COLS, 0x444444); 

    // Dibujar paletas
    for(int y=0; y<8; y++){
        int pal_idx = y + (8 * current_page);
        for(int x=1; x<NUM_COLS; x++){
            if (pal_idx < 25) {
                uint32_t r, g, b;
                get_color_for_level(pal_idx, x, &r, &g, &b);
                trellis.setPixelColor(y * NUM_COLS + x, (r << 16) | (g << 8) | b);
            } else {
                trellis.setPixelColor(y * NUM_COLS + x, 0);
            }
        }
    }
    trellis.show();
}

static void check_palette_preview() {
    if (palette_preview_active) {
        if (to_ms_since_boot(get_absolute_time()) - palette_preview_start >= 1000) {
            palette_preview_active = false;
            if (mode == 0) {
                grid_dirty = true;
            } else {
                memset(prevLedBuffer, 255, sizeof(prevLedBuffer)); 
            }
        }
    }
}

static void sendLeds_iii() {
    if (palette_ui_active || palette_preview_active) return;
    for (int i = 0; i < NUM_ROWS * NUM_COLS; i++) {
        trellis.setPixelColor(i, level_to_color(local_leds[i]));
    }
    trellis.show();
}

static void sendLeds_monome() {
    if (palette_ui_active || palette_preview_active) return;
    bool dirty = false;
    for (int i = 0; i < NUM_ROWS * NUM_COLS; i++) {
        uint8_t val = mdp.leds[i];
        if (val != (uint8_t)prevLedBuffer[i]) {
            trellis.setPixelColor(i, level_to_color(val));
            prevLedBuffer[i] = val;
            dirty = true;
        }
    }
    if (dirty) trellis.show();
}

static TrellisCallback keyCallback(keyEvent evt) {
    if (evt.bit.EDGE != SEESAW_KEYPAD_EDGE_RISING && evt.bit.EDGE != SEESAW_KEYPAD_EDGE_FALLING) return 0;

    uint8_t x = evt.bit.NUM % NUM_COLS;
    uint8_t y = evt.bit.NUM / NUM_COLS;
    uint8_t z = (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) ? 1 : 0;

    if (x == 13 && y == 7) key_13_7 = (z == 1);
    if (x == 14 && y == 7) key_14_7 = (z == 1);
    if (x == 15 && y == 7) key_15_7 = (z == 1);

    if (key_13_7 && key_14_7 && key_15_7 && !palette_ui_active && !palette_preview_active) {
        palette_ui_active = true;
        draw_palette_ui();
        return 0; 
    }

    if (palette_ui_active) {
        if (z == 1) { 
            if (x == 0) {
                if (y < 7) {
                    set_dynamic_gamma(y);
                    draw_palette_ui();
                } else if (y == 7) {
                    current_page = (current_page + 1) % 4; 
                    draw_palette_ui();
                }
            } else {
                int chosen = y + (8 * current_page);
                if (chosen < 25) {
                    selected_palette = chosen;
                    palette_ui_active = false;
                    palette_preview_active = true;
                    palette_preview_start = to_ms_since_boot(get_absolute_time());
                    for(int i=0; i<NUM_ROWS; i++){
                        for(int j=0; j<NUM_COLS; j++){
                            uint8_t val = j % 16;
                            uint32_t r, g, b;
                            get_color_for_level(selected_palette, val, &r, &g, &b);
                            trellis.setPixelColor(i * NUM_COLS + j, (r << 16) | (g << 8) | b);
                        }
                    }
                    trellis.show();
                }
            }
        }
        return 0; 
    }

    if (palette_preview_active) return 0;

    if (mode == 0) {
        grid_add_event(x, y, z);
    } else {
        mdp.sendGridKey(x, y, z);
    }
    return 0;
}

static TrellisCallback keyCheck(keyEvent evt) {
    uint8_t x = evt.bit.NUM % NUM_COLS;
    uint8_t y = evt.bit.NUM / NUM_COLS;
    uint8_t z = (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) ? 1 : 0;
    if (x == 0 && y == 0 && z == 1) {
        mode = 1; 
    }
    return 0;
}

extern "C" bool check_device_key() {
    trellis_array[0][0].setKeypadEvent(NEO_TRELLIS_KEY(0), SEESAW_KEYPAD_EDGE_HIGH, true);
    bool detected = false;
    for (int i = 0; i < 50 && !detected; i++) {
        sleep_ms(10);
        uint8_t count = trellis_array[0][0].getKeypadCount();
        sleep_us(500);
        if (count > 0) {
            keyEventRaw e[count];
            trellis_array[0][0].readKeypad(e, count);
            for (int j = 0; j < count; j++) {
                uint8_t key = NEO_TRELLIS_SEESAW_KEY(e[j].bit.NUM);
                if (key == 0 && e[j].bit.EDGE == SEESAW_KEYPAD_EDGE_HIGH) {
                    mode = (mode == 0) ? 1 : 0;
                    detected = true;
                    break;
                }
            }
        }
    }
    trellis_array[0][0].setKeypadEvent(NEO_TRELLIS_KEY(0), SEESAW_KEYPAD_EDGE_HIGH, false);
    trellis_array[0][0].setKeypadEvent(NEO_TRELLIS_KEY(0), SEESAW_KEYPAD_EDGE_RISING, true);
    trellis_array[0][0].setKeypadEvent(NEO_TRELLIS_KEY(0), SEESAW_KEYPAD_EDGE_FALLING, true);
    uint8_t stale = trellis_array[0][0].getKeypadCount();
    sleep_us(500);
    if (stale > 0) {
        keyEventRaw tmp[stale + 2];
        trellis_array[0][0].readKeypad(tmp, stale + 2);
    }
    return detected;
}

extern "C" void mode_check() {
    uint8_t saved = flash_read_mode();
    mode = saved;
    check_device_key();   
    if (mode != saved) {
        flash_write_mode(mode);
        g_monome_mode = mode;
    }
}

extern "C" void device_init() {
    trellis.begin();
    for (uint8_t x = 0; x < NUM_COLS; x++) {
        for (uint8_t y = 0; y < NUM_ROWS; y++) {
            trellis.activateKey(x, y, SEESAW_KEYPAD_EDGE_RISING, true);
            trellis.activateKey(x, y, SEESAW_KEYPAD_EDGE_FALLING, true);
            trellis.registerCallback(x, y, keyCallback);
        }
    }
    for (uint8_t x = 0; x < NUM_COLS / 4; x++) {
        for (uint8_t y = 0; y < NUM_ROWS / 4; y++) {
            trellis_array[y][x].pixels.setBrightness(BRIGHTNESS);
        }
    }

    memset(local_leds,   0, sizeof(local_leds));
    memset(mmap,         0, sizeof(mmap));
    memset(prevLedBuffer, 0, sizeof(prevLedBuffer));
    
    set_dynamic_gamma(3); // Inicializa en Nivel 4 (Fila 3)
    sendLeds_iii();

    gpio_put(LED_PIN, 0);
    mode_check();

    if (mode == 0){
        trellis.setPixelColor(0, 0xFFFFFF);
        trellis.setPixelColor(7, 0xFFFFFF);
        trellis.setPixelColor(15, 0xFFFFFF);
        trellis.show();
        sleep_ms(100);
        trellis.setPixelColor(0, 0x000000);
        trellis.setPixelColor(7, 0x000000);
        trellis.setPixelColor(15, 0x000000);
        trellis.show();
    }else{
        trellis.setPixelColor(0, 0xFFFFFF);
        trellis.show();
        sleep_ms(100);
        trellis.setPixelColor(0, 0x000000);
        trellis.show();   
    }
}

extern "C" void device_task() {
    trellis.read();
    check_palette_preview(); 

    uint8_t ewr = 0;
    while (grid_event_count > 0) {
        vm_handle_grid_key(grid_events[ewr].x, grid_events[ewr].y, grid_events[ewr].z);
        ewr++;
        if (ewr == grid_event_count) {
            grid_event_count = 0;
            ewr = 0;
        }
    }

    if (grid_dirty) {
        sendLeds_iii();
        grid_dirty = false;
    }
    tud_cdc_n_write_flush(0);
}

extern "C" void device_handle_serial(uint8_t *data, uint32_t len) {
    (void)data;
    (void)len;
}

extern "C" void device_led_set(int x, int y, int z, int rel) {
    if (x < 0 || x >= NUM_COLS || y < 0 || y >= NUM_ROWS) return;
    int idx = y * NUM_COLS + x;
    int8_t z8 = (int8_t)(rel ? clamp(z + (int)mmap[idx], 0, 15) : clamp(z, 0, 15));
    local_leds[idx] = (uint8_t)z8;
    mmap[idx]       = (uint8_t)z8;
    grid_dirty = true;
}

extern "C" int device_led_get(int x, int y) {
    x = ((x % NUM_COLS) + NUM_COLS) % NUM_COLS;
    y = ((y % NUM_ROWS) + NUM_ROWS) % NUM_ROWS;
    return (int)mmap[y * NUM_COLS + x];
}

extern "C" void device_led_all(int z, int rel) {
    if (rel) {
        for (int i = 0; i < NUM_ROWS * NUM_COLS; i++) {
            int8_t zz = (int8_t)clamp(z + (int)mmap[i], 0, 15);
            mmap[i]       = (uint8_t)zz;
            local_leds[i] = (uint8_t)zz;
        }
    } else {
        uint8_t z8 = (uint8_t)clamp(z, 0, 15);
        memset(mmap,       z8, sizeof(mmap));
        memset(local_leds, z8, sizeof(local_leds));
    }
    grid_dirty = true;
}

extern "C" void device_intensity(int z) {
    if (z > 15) z = 15;
    uint8_t brightness = (uint8_t)((z * 255) / 15);
    for (uint8_t xi = 0; xi < NUM_COLS / 4; xi++) {
        for (uint8_t yi = 0; yi < NUM_ROWS / 4; yi++) {
            trellis_array[yi][xi].pixels.setBrightness(brightness);
        }
    }
    grid_dirty = true;
}

extern "C" void device_mark_dirty(void) { grid_dirty = true; }
extern "C" int  device_cols(void)       { return NUM_COLS; }
extern "C" int  device_rows(void)       { return NUM_ROWS; }
extern "C" void device_color_set(int r, int g, int b) {
    grid_dirty = true;
}

static const char *device_help_str =
    "grid\n"
    "  event_grid(x,y,z)  (alias: grid)\n"
    "  grid_led(x,y,z,rel)\n"
    "  grid_led_get(x,y)\n"
    "  grid_led_all(z,rel)\n"
    "  grid_intensity(z)\n"
    "  grid_refresh()\n"
    "  grid_size_x()\n"
    "  grid_size_y()\n";

extern "C" const char *device_help_txt() { return device_help_str; }
extern "C" const char *device_id()       { return "neotrellis-grid"; }
extern "C" const char *device_str1()     { return "monome"; }
extern "C" const char *device_str2()     { return (mode == 0) ? "iii grid" : "grid"; }
extern "C" const char *device_version()  { return DEVICE_VERSION; }

extern "C" void device_monome_loop() {
    mdp.isMonome = true;
    mdp.deviceID = deviceID;
    mdp.setupAsGrid(NUM_ROWS, NUM_COLS);

    for (int i = 0; i < 8; i++) {
        tud_task();
        mdp.poll();
        sleep_ms(100);
    }
    mdp.getDeviceInfo();

    mdp.setAllLEDs(0);
    sendLeds_monome();

    uint32_t last_refresh = to_ms_since_boot(get_absolute_time());

    while (true) {
        tud_task();
        mdp.poll();
        check_palette_preview(); 

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_refresh >= 16) {
            trellis.read();
            sendLeds_monome();
            last_refresh = now;
        }
        tud_cdc_write_flush();
    }
}