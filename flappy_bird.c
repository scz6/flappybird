/**
 * Helen Ni, Selena Zhang, Matthew Amorocho
 *
 * HARDWARE CONNECTIONS
  - GPIO 16 ---> VGA Hsync
  - GPIO 17 ---> VGA Vsync
  - GPIO 18 ---> VGA Green lo-bit --> 470 ohm resistor --> VGA_Green
  - GPIO 19 ---> VGA Green hi_bit --> 330 ohm resistor --> VGA_Green
  - GPIO 20 ---> 330 ohm resistor ---> VGA-Blue
  - GPIO 21 ---> 330 ohm resistor ---> VGA-Red
  - RP2040 GND ---> VGA-GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels (2, by claim mechanism)
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga16_graphics_v2.h"
#include "assets.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "mpu6050.h"
#include "point_sound.h"
#include "flap_sound.h"
#include "hit_sound.h"
// Include protothreads
#include "pt_cornell_rp2040_v1_4.h"

// === the fixed point macros ===
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
#define hitBottom(b) (b>int2fix15(460))
#define hitTop(b) (b<int2fix15(0))

// uS per frame
#define FRAME_RATE 33000

#define BUTTON_PIN 15
#define ARCADE_BUTTON_PIN 13

// bird on core 0
fix15 bird_x ;
fix15 bird_y ;
fix15 bird_vy ;

fix15 bird_angle_deg = 0.0f;

// === arcade button fsms ===
#define STATE_START 0
#define STATE_FALL 1
#define STATE_END 2

volatile int state = STATE_START;

// === mode fsm states ===
#define STATE_BUTTON 0
#define STATE_MIC 1
#define STATE_IMU 2

volatile int mode_state = STATE_BUTTON;

const fix15 GRAVITY = float2fix15(0.75f);

#define SCREEN_WIDTH 640
#define SCREEN_HEIGHT 480

// ======= PIPE VARIABLES =========
#define PIPE_SPEED int2fix15(2)
#define PIPE_WIDTH 32
#define PIPE_HEIGHT 25

#define MIN_GAP_SIZE 75   
#define MAX_GAP_SIZE 100

#define GAP_MARGIN 100  

#define PIPE_CAP_H      6    
#define PIPE_CAP_EXTRA  10  

#define NUM_PIPE_PAIRS 4

// horizontal spacing between pipe pairs 
#define PIPE_SPACING 200

// pipes on core 0 
fix15 pipeb_x[NUM_PIPE_PAIRS];
fix15 pipeb_y[NUM_PIPE_PAIRS];

fix15 pipet_x[NUM_PIPE_PAIRS];
fix15 pipet_y[NUM_PIPE_PAIRS];

// pipe drawing
extern unsigned char vga_data_array[];

static uint8_t pipe_body_rows8[16][16];
static uint8_t pipe_cap_rows8[16][16];

#define CAP_EXTRA_EACH_SIDE 4
#define CAP_WIDE_WIDTH (PIPE_WIDTH + CAP_EXTRA_EACH_SIDE * 2)  // 40 pixels
#define CAP_WIDE_BYTES (CAP_WIDE_WIDTH / 2)  // 20 bytes

static uint8_t pipe_cap_wide_rows8[16][CAP_WIDE_BYTES];         // Top pipe cap
static uint8_t pipe_cap_wide_flipped_rows8[16][CAP_WIDE_BYTES]; // Bottom pipe cap (flipped 180)

// score
int score = 0;
int high_score = 0;
int last_score_on_death;
bool pipe_scored[NUM_PIPE_PAIRS];
bool pipe_just_reset[NUM_PIPE_PAIRS];

// mic on core 1
fix15 noise_avg;
#define MIC_THRESHOLD 3500

// imu on core 1
fix15 acceleration[3], gyro[3];
static fix15 gyro_angle;

// bird tilt
// how much tilt counts as a flap
#define TILT_UP_THRESHOLD   float2fix15(0.80f) 
// how close to level the IMU must be before the next flap is allowed  
#define TILT_RESET_THRESHOLD float2fix15(0.20f)  

// ======= SOUND DEFINES/GLOBALS =========

// Timer alarm we use for audio
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// SPI/DAC pins and port 
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC   8

#define SPI_PORT spi0

// GPIO just for timing the ISR 
#define ISR_GPIO 2

// audio parameters
#define Fs 50000  // sample rate (50 kHz)
#define DELAY 20  // 1/Fs in microseconds
#define BGM_FS Fs 

// A-channel DAC
#define DAC_config_chan_A 0b0011000000000000
// B-channel DAC
#define DAC_config_chan_B 0b1011000000000000

// ==== BACKGROUND MUSIC & SOUND EFFECTS  ========

#define BGM_TABLE_LEN 256
uint16_t bgm_table[BGM_TABLE_LEN];

volatile uint32_t bgm_phase_acc = 0;
volatile uint32_t bgm_phase_inc = 0;  

#define BGM_NUM_NOTES 8

uint32_t bgm_phase_inc_notes[BGM_NUM_NOTES];

volatile uint8_t bgm_note_index = 0;

#define BGM_NOTE_DURATION_SAMPLES (BGM_FS / 3)

volatile uint32_t bgm_note_sample_count = 0;

// === SFX identifiers for DMA playback ===
enum {
  SFX_HIT = 0,
  SFX_POINT,
  SFX_FLAP,
  NUM_SFX
};

// Pointers to 16-bit preformatted DAC words (in .c files)
static const uint16_t* sfx_buffers[NUM_SFX] = {
  hit_sfx_data,
  point_sfx_data,
  flap_sfx_data
};

static const uint32_t sfx_lengths[NUM_SFX] = {
  HIT_SFX_LEN,
  POINT_SFX_LEN,
  FLAP_SFX_LEN
};

// DMA channel + config for SFX
static int sfx_dma_chan;
static dma_channel_config sfx_dma_cfg;

// ====== PRIORITIZED SFX PLAY FUNCTION ======
// HIT → highest priority: ALWAYS override
// POINT → must always play: ALWAYS override
// FLAP → lowest priority: skip if busy

static inline void sfx_play(int which)
{
  if (which < 0 || which >= NUM_SFX) return;

  const uint16_t* buf = sfx_buffers[which];
  uint32_t len = sfx_lengths[which];
  if (!buf || len == 0) return;

  bool dma_busy = dma_channel_is_busy(sfx_dma_chan);

  if (which == SFX_FLAP) {
    if (dma_busy) return;
  } 
  else {
    // point or hit -> override ongoing SFX
    dma_channel_abort(sfx_dma_chan);
    dma_channel_acknowledge_irq0(sfx_dma_chan);
  }

  // Start DMA transfer
  dma_channel_configure(
    sfx_dma_chan,
    &sfx_dma_cfg,
    &spi_get_hw(SPI_PORT)->dr,   // SPI TX FIFO
    buf,                         // source buffer
    len,                         // number of samples
    true                         // start immediately
  );
}


// ============================================================
// PIPE INITIALIZATION & UPDATE SYSTEM
// ============================================================

// Create a new random pipe pair at base_x_px
static void init_pipe_pair(int i, int base_x_px)
{
  int gap_size = MIN_GAP_SIZE + (rand() % (MAX_GAP_SIZE - MIN_GAP_SIZE + 1));

  int max_gap_top = SCREEN_HEIGHT - GAP_MARGIN - gap_size;
  if (max_gap_top < GAP_MARGIN) {
    max_gap_top = GAP_MARGIN;
  }
  int gap_top    = GAP_MARGIN + (rand() % (max_gap_top - GAP_MARGIN + 1));
  int gap_bottom = gap_top + gap_size;

  pipet_x[i] = int2fix15(base_x_px);
  pipeb_x[i] = int2fix15(base_x_px);

  // top pipe stored as: pipet_y + PIPE_HEIGHT = gap_top
  pipet_y[i] = int2fix15(gap_top - PIPE_HEIGHT);

  // bottom pipe y = gap_bottom
  pipeb_y[i] = int2fix15(gap_bottom);

  pipe_scored[i] = false;
  pipe_just_reset[i] = true;
}

// Initialize all pipe pairs before a game begins
static void init_all_pipes(void)
{
  int start_x = SCREEN_WIDTH + 50;

  for (int i = 0; i < NUM_PIPE_PAIRS; i++) {
    init_pipe_pair(i, start_x + i * PIPE_SPACING);
  }
}

// Move pipes left; recycle off-screen pipes as new random ones on right
static void update_pipes(void)
{
  if (state == STATE_START || state == STATE_END) {
    return;
  }

  for (int i = 0; i < NUM_PIPE_PAIRS; i++) {

    pipet_x[i] -= PIPE_SPEED;
    pipeb_x[i] -= PIPE_SPEED;

    int pipe_right_px = fix2int15(pipet_x[i]) + PIPE_WIDTH;

    pipe_just_reset[i] = false;

    if (pipe_right_px < 0) {
      int max_x = 0;
      for (int j = 0; j < NUM_PIPE_PAIRS; j++) {
        int xj = fix2int15(pipet_x[j]);
        if (xj > max_x) max_x = xj;
      }

      int new_x = max_x + PIPE_SPACING;

      init_pipe_pair(i, new_x);
      pipe_just_reset[i] = true;
    }
  }
}

void update(fix15* x, fix15* y, fix15* vy) {
  // Do NOT update anything once the game has ended
  if (state == STATE_END) {
    return;
  }

  // handle top/bottom wall death
  if (hitTop(*y) || hitBottom(*y)) {
    *x = int2fix15(75);
    *y = int2fix15(216);
    *vy = int2fix15(1);

    last_score_on_death = score;

    // update high scores
    if (score > high_score) {
      high_score = score;
    }
    // reset score on death
    score = 0;
    
    state = STATE_END;
    
    // hit sound plays
    sfx_play(SFX_HIT);

  }

  // scoring
  if (state != STATE_START && state != STATE_END) {  
    for (int i = 0; i < NUM_PIPE_PAIRS; i++) {
  
      // right edge of the pipe pair
      int pipe_right_px = fix2int15(pipet_x[i]) + PIPE_WIDTH;
      // if point not already counted, and bird moved past the left edge of pipe
      if (!pipe_scored[i] && pipe_right_px < fix2int15(*x)) {
        score++;
        pipe_scored[i] = true;

        // Update the high score if the current score exceeds the high score
        if (score > high_score) {
          high_score = score;
        }

        // point sound plays
        sfx_play(SFX_POINT);

      }
    }
  }

  // collision logic

  fix15 bird_top    = *y;
  fix15 bird_bottom = *y + int2fix15(16);

  int bird_x_int = fix2int15(*x);
  int bird_right_int = bird_x_int + 32; 

  // 1. Find the closest pipe that could possibly collide in X
  int closest_i  = -1;
  int min_dx     = 100000; 

    for (int i = 0; i < NUM_PIPE_PAIRS; i++) {

      int pipe_x_int     = fix2int15(pipet_x[i]);       // left edge of pipe (int)
      int pipe_right_int = pipe_x_int + PIPE_WIDTH;     // right edge (int)

      // if pipe is completely to the left of the bird, skip
      if (pipe_right_int < bird_x_int - 10) {
        continue;
      }

    // If pipe is completely to the right of the bird, skip
      if (pipe_x_int > bird_right_int + 200) {
        continue;
      }

      // distance from bird to pipe in X (if overlapping, treat as 0)
      int dx = pipe_x_int - bird_x_int;
      if (dx < 0) dx = 0;

      if (dx < min_dx) {
        min_dx    = dx;
        closest_i = i;
      }
    }

  // 2. If we found a candidate pipe, do full Y overlap check just for that one
  if (closest_i != -1) {
    int i = closest_i;

    // X overlap in fix15
    bool overlap_x = (*x + int2fix15(32)) >= pipet_x[i] && *x <= (pipet_x[i] + int2fix15(PIPE_WIDTH));

    if (overlap_x) {
      // compute pipe bounds in Y
      fix15 top_pipe_bottom    = pipet_y[i] + int2fix15(PIPE_HEIGHT);
      fix15 bottom_pipe_top    = pipeb_y[i];
      fix15 bottom_pipe_bottom = int2fix15(SCREEN_HEIGHT);

      bool hit_top_pipe    = (bird_bottom >= 0) && (bird_top    <= top_pipe_bottom);
      bool hit_bottom_pipe = (bird_bottom >= bottom_pipe_top) && (bird_top    <= bottom_pipe_bottom);

      if (hit_top_pipe || hit_bottom_pipe) {
        sfx_play(SFX_HIT);
        // === collision response same as before ===
        *x  = int2fix15(75);
        *y  = int2fix15(216);
        *vy = int2fix15(1);

        // store score for end screen
        last_score_on_death = score;

        // update high score before resetting score
        if (score > high_score) {
          high_score = score;
        }

        // reset score on death
        score = 0;
        for (int j = 0; j < NUM_PIPE_PAIRS; j++) {
          pipe_scored[j] = false;
        }


        state = STATE_END;
      }
    }
  }

  // bird motion
  if (state != STATE_START) {
    *vy = *vy + GRAVITY;
  }

  *y = *y + *vy;

}

// ============================================================
// PIPE SPRITE DRAWING
// ============================================================

// takes two 4-bit color values a and b packs them into one byte 
static inline uint8_t pack2(uint8_t a, uint8_t b) { 
  return (a & 0xF) | ((b & 0xF) << 4); 
}

// takes raw asset format and turns it into byte-pack rows
// 1 row = 2 words, and 1 word = 8 pix 
static void pack_tile32(const unsigned long long tile[16][2], uint8_t out[16][16]) {
  for (int r = 0; r < 16; r++) {
    // columns 0-15
    for (int c = 0; c < 16; c += 2) {
      int s = 15 - c;
      uint8_t p0 = (tile[r][0] >> (4 * (s - 1))) & 0xF;
      uint8_t p1 = (tile[r][0] >> (4 * s)) & 0xF;
      out[r][c >> 1] = pack2(p1, p0); 
    }
    // columns 16-31
    for (int c = 0; c < 16; c += 2) {
      int s = 15 - c;
      uint8_t p0 = (tile[r][1] >> (4 * (s - 1))) & 0xF;
      uint8_t p1 = (tile[r][1] >> (4 * s)) & 0xF;
      out[r][8 + (c >> 1)] = pack2(p1, p0);  
    }
  }
}

static void init_pipe_sprites(void) {
  pack_tile32(PIPE_BODY_TILE, pipe_body_rows8);
  pack_tile32(PIPE_CAP_TILE,  pipe_cap_rows8);
  
  // CAP
  uint8_t green_pair = pack2(2, 2);  // MED_GREEN = color 2
  int ext_bytes = CAP_EXTRA_EACH_SIDE / 2;  
  
  for (int r = 0; r < 16; r++) {
    int idx = 0;
    // Left green extension (4 pixels = 2 bytes)
    for (int i = 0; i < ext_bytes; i++) {
        pipe_cap_wide_rows8[r][idx++] = green_pair;
    }
    // Original 32-pixel cap (16 bytes)
    for (int i = 0; i < 16; i++) {
        pipe_cap_wide_rows8[r][idx++] = pipe_cap_rows8[r][i];
    }
    // Right green extension (4 pixels = 2 bytes)
    for (int i = 0; i < ext_bytes; i++) {
        pipe_cap_wide_rows8[r][idx++] = green_pair;
    }
  }
  
  // Create flipped version for bottom pipe ( flipped 180 degrees)
  for (int r = 0; r < 16; r++) {
    int src_row = 15 - r;  
    for (int i = 0; i < CAP_WIDE_BYTES; i++) {
      pipe_cap_wide_flipped_rows8[r][i] = pipe_cap_wide_rows8[src_row][i];
    }
  }
}

// draws one row in the pipe body
static inline void pipebody_row32(int xi, int y, const uint8_t* row) {
    memcpy(&vga_data_array[((SCREEN_WIDTH * y) + xi) >> 1], row, 16);
}

// draws one row for the top cap
static inline void cap_sprite_row(int xi, int y, int row) {
    if ((unsigned)y >= SCREEN_HEIGHT) return;
    int cap_x = xi - CAP_EXTRA_EACH_SIDE;  
    if (cap_x < 0 || cap_x + CAP_WIDE_WIDTH > SCREEN_WIDTH) return;
    memcpy(&vga_data_array[((SCREEN_WIDTH * y) + cap_x) >> 1], pipe_cap_wide_rows8[row], CAP_WIDE_BYTES);
}

// draws one row for the bottom cap
static inline void cap_sprite_row_flipped(int xi, int y, int row) {
    if ((unsigned)y >= SCREEN_HEIGHT) return;
    int cap_x = xi - CAP_EXTRA_EACH_SIDE;  
    if (cap_x < 0 || cap_x + CAP_WIDE_WIDTH > SCREEN_WIDTH) return;
    memcpy(&vga_data_array[((SCREEN_WIDTH * y) + cap_x) >> 1], pipe_cap_wide_flipped_rows8[row], CAP_WIDE_BYTES);
}

// Draw only the top portion of a top pipe (y=0 to y=top_rows-1)
static void draw_top_pipe_top_rows(int x, int gap_top, int top_rows) {
    int xi = x & ~1; // align position
    if (xi < 0 || xi + PIPE_WIDTH > SCREEN_WIDTH) return;
    
    int cap_start = gap_top - 16;
    if (cap_start < 0) cap_start = 0;
    
    int end_y = (top_rows < cap_start) ? top_rows : cap_start;
    for (int y = 0; y < end_y; y++) {
        pipebody_row32(xi, y, pipe_body_rows8[y & 15]);
    }
}

// Draw the rest of a top pipe (from y=top_rows downward)
static void draw_top_pipe(int x, int gap_top, int top_rows) {
    int xi = x & ~1;
    if (xi < 0 || xi + PIPE_WIDTH > SCREEN_WIDTH) return;
    
    int cap_start = gap_top - 16;
    if (cap_start < 0) cap_start = 0;

    // body: from top_rows to cap_start-1
    for (int y = top_rows; y < cap_start; y++) {
        pipebody_row32(xi, y, pipe_body_rows8[y & 15]);
    }
    // cap: 16 rows ending at gap_top-1 (wide cap, symmetrical)
    for (int r = 0; r < 16 && cap_start + r < gap_top && cap_start + r < SCREEN_HEIGHT; r++) {
        cap_sprite_row(xi, cap_start + r, r);
    }
}

static void draw_bottom_pipe(int x, int gap_bottom) {
    int xi = x & ~1;
    // Only draw if pipe is fully on screen
    if (xi < 0 || xi + PIPE_WIDTH > SCREEN_WIDTH) return;

    // cap at gap_bottom..gap_bottom+15 (flipped wide cap for bottom pipe)
    for (int r = 0; r < 16 && gap_bottom + r < SCREEN_HEIGHT; r++) {
        cap_sprite_row_flipped(xi, gap_bottom + r, r);
    }
    // body from gap_bottom+16 to bottom
    for (int y = gap_bottom + 16; y < SCREEN_HEIGHT; y++) {
        pipebody_row32(xi, y, pipe_body_rows8[y & 15]);
    }
}

// ============================================================
// FLAPPY BIRD SPRITE DRAWING
// ============================================================

// Draw the bird rotated around its center by bird_angle_deg (fix15 degrees)
// (cx, cy) is the center of the bird in screen coordinates
void drawFlappyBirdSpriteRotated(int cx, int cy, fix15 angle_deg_fix) {

  const int cx_local = 16;
  const int cy_local = 8;

  // Convert angle from degrees (fix15) -> radians (fix15)
  const fix15 DEG2RAD = float2fix15((float)M_PI / 180.0f);
  fix15 angle_rad_fix = multfix15(angle_deg_fix, DEG2RAD);

  // Compute cos/sin in float ONCE, then store as fix15
  float angle_rad_f = fix2float15(angle_rad_fix);
  fix15 cos_a = float2fix15(cosf(angle_rad_f));
  fix15 sin_a = float2fix15(sinf(angle_rad_f));

  // determine where each sprite pixel lands on the screen after rotation
  for (int sy = 0; sy < 16; sy++) {
    for (int sx = 0; sx < 32; sx++) {

      // read color + transparency from your sprite arrays
      int half = (sx < 16) ? 0 : 1;
      int col  = sx & 15;  // sx % 16

      uint64_t rowData = BIRD_TILE[sy][half];
      uint16_t mask    = BIRD_MASK[sy][half];

      int bitIndex    = 15 - col;
      int transparent = (mask >> bitIndex) & 1;
      if (transparent) continue;

      int nibbleShift = (15 - col) * 4;
      uint8_t color   = (rowData >> nibbleShift) & 0xF;

      // local coordinates relative to sprite center
      int dx_i = sx - cx_local;   
      int dy_i = sy - cy_local;   

      // Convert dx,dy to fix15
      fix15 dx = int2fix15(dx_i);
      fix15 dy = int2fix15(dy_i);

      // Rotate: rx = dx*cos - dy*sin ; ry = dx*sin + dy*cos (in fix15)
      fix15 rx_fix = multfix15(dx, cos_a) - multfix15(dy, sin_a);
      fix15 ry_fix = multfix15(dx, sin_a) + multfix15(dy, cos_a);

      int px = cx + fix2int15(rx_fix);
      int py = cy + fix2int15(ry_fix);

      if (px < 0 || px >= SCREEN_WIDTH || py < 0 || py >= SCREEN_HEIGHT) {
        continue;
      }

      drawPixel(px, py, color);
    }
  }
}


// Smoothly update tilt angle based on vertical velocity (all fix15)
static void update_bird_angle(void) {

  int vy_int = fix2int15(bird_vy);

  // Target angle in degrees, as fix15
  fix15 target_angle;

  if (vy_int < -2) {
    // Going up -> tilt up 
    target_angle = int2fix15(-15);   
  } else if (vy_int > 2) {
    // Falling -> tilt down
    target_angle = int2fix15(20);    
  } else {
    // Near zero -> neutral
    target_angle = int2fix15(0);
  }

  // Smooth factor alpha in (0,1) as fix15
  const fix15 alpha = float2fix15(0.15f);

  // bird_angle_deg += alpha * (target - current)
  fix15 diff = target_angle - bird_angle_deg;
  bird_angle_deg = bird_angle_deg + multfix15(alpha, diff);
}

// ============================================================
// START AND END SCREEN
// ============================================================
void draw_start_screen(void) {
  char buf[64];
  // Text setup
  setTextColor(GREEN);
  setTextWrap(0);

  // Big title
  setTextSize(4);
  setCursor(190, 120);
  writeString("Flappy Bird");

  // Gameplay mode label
  setTextSize(2);
  setCursor(240, 200);
  writeString("Gameplay mode:");

  // Show which mode is currently selected
  setCursor(190, 230);
  if (mode_state == STATE_BUTTON) {
    writeString("BUTTON (click to flap)");
  } else if (mode_state == STATE_MIC) {
    writeString("MIC (speak to flap)");
  } else if (mode_state == STATE_IMU) {
    writeString("IMU (tilt to flap)");
  }

  setTextSize(1);
  setCursor(250, 280);
  writeString("By: hn347, scz6, ma969");
}

void draw_end_screen(void) {
  char buf[64];
  // Text setup
  setTextColor(GREEN);
  setTextWrap(0);

  // Big title
  setTextSize(4);
  setCursor(215, 120);
  writeString("GAME OVER");

  // Score earned
  setTextSize(3);
  setCursor(250, 190);
  sprintf(buf, "Score: %d", last_score_on_death);
  writeString(buf);

  // High score
  setTextSize(3);
  setCursor(200, 230);
  sprintf(buf, "High Score: %d", high_score);
  writeString(buf);

  setTextSize(1);
  setCursor(265, 280);
  writeString("click to play again");
}

// ============================================================
// BACKGROUND MUSIC
// ============================================================

// convert frequency (Hz) to 32-bit DDS phase increment for BGM
static uint32_t bgm_freq_to_phase_inc_uint(uint32_t freq_hz) {
    // phase_inc = freq_hz * 2^32 / BGM_FS
    uint64_t num = (uint64_t)freq_hz * (1ULL << 32);
    return (uint32_t)(num / (uint64_t)BGM_FS);
}

void init_bgm_arpeggio(void) {
  // frequencies in Hz (rounded is fine for lo-fi)
  static const uint32_t notes_hz[BGM_NUM_NOTES] = {
    220,  // A3
    262,  // ~C4
    330,  // ~E4
    392,  // G4
    330,  // E4
    262,  // C4
    220,  // A3
    175   // ~F3
  };

  for (int i = 0; i < BGM_NUM_NOTES; i++) {
    bgm_phase_inc_notes[i] = bgm_freq_to_phase_inc_uint(notes_hz[i]);
  }

  bgm_phase_acc         = 0;
  bgm_note_index        = 0;
  bgm_note_sample_count = 0;
  bgm_phase_inc         = bgm_phase_inc_notes[0];
}


static void alarm_irq(void) {
  // Clear the alarm irq
  hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);
  // Reset the alarm for the next sample
  timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

  // Background music synthesis
  bgm_phase_acc += bgm_phase_inc;

  // Index into wavetable (BGM_TABLE_LEN = 256 => top 8 bits of phase)
  uint32_t index = bgm_phase_acc >> (32 - 8);
  uint16_t bgm_sample = bgm_table[index & (BGM_TABLE_LEN - 1)];

  // Make BGM quieter than SFX
  int32_t centered = (int32_t)bgm_sample - 2048;
  centered >>= 2;  // divide amplitude by 4
  bgm_sample = (uint16_t)(2048 + centered);

  uint16_t DAC_data_B = DAC_config_chan_B | (bgm_sample & 0x0FFF);
  spi_write16_blocking(SPI_PORT, &DAC_data_B, 1);

  // Advance note duration counter
  bgm_note_sample_count++;
  if (bgm_note_sample_count >= BGM_NOTE_DURATION_SAMPLES) {
      bgm_note_sample_count = 0;
      bgm_note_index = (bgm_note_index + 1) % BGM_NUM_NOTES;
      bgm_phase_inc = bgm_phase_inc_notes[bgm_note_index];
  }
}

// sine-like table for calm background audio
void init_bgm_table(void) {
  for (int i = 0; i < BGM_TABLE_LEN; i++) {
    int phase = i & 0xFF;  // 0..255

    int sample;
    if (phase < 128) {
      // ramp up 0 -> 4095
      sample = (phase * 4095) / 127;
    } else {
      // ramp down 4095 -> 0
      sample = ((255 - phase) * 4095) / 127;
    }

    if (sample < 0)     sample = 0;
    if (sample > 4095)  sample = 4095;

    bgm_table[i] = (uint16_t)sample;
  }
}

// ============================================================
// ANIMATION THREAD
// ============================================================

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt)) {
  // Mark beginning of thread
  PT_BEGIN(pt);

  // Variables for maintaining frame rate
  static int begin_time ;
  static int spare_time ;
  
  while(1) {
    // Measure time at start of thread
    begin_time = time_us_32() ;      

    if (state == STATE_START) {
      bird_x = int2fix15(75);
      bird_y = int2fix15(216);
      bird_vy = int2fix15(0);

      for (int i = 0; i < NUM_PIPE_PAIRS; i++) {
        pipe_scored[i] = false;
      }
    }

    // move pipes & recycle as needed
    update_pipes();
    
    // update position and velocity
    update(&bird_x, &bird_y, &bird_vy);
    update_bird_angle();

    // Wait for VSYNC then clear and redraw
    while(gpio_get(VSYNC)){};

    clearLowFrame(0, 0);

    if (state == STATE_START) {
      draw_start_screen();
    } 
    else if (state == STATE_END) {
      draw_end_screen();
    } 
    else {
            
      // Pass 1: Draw top rows of all top pipes ASAP
      for (int i = 0; i < NUM_PIPE_PAIRS; i++) {
        int px = fix2int15(pipet_x[i]);
        int top_y_int = fix2int15(pipet_y[i]);
        int xi = px & ~1;
        
        if (xi < 0 || xi + PIPE_WIDTH > SCREEN_WIDTH) continue;
        if (pipe_just_reset[i]) continue;
        
        int gap_top = top_y_int + PIPE_HEIGHT;
        draw_top_pipe_top_rows(px, gap_top, 48);
      }
      
      // Pass 2: Draw the rest of each pipe (top pipe remainder + bottom pipe)
      for (int i = 0; i < NUM_PIPE_PAIRS; i++) {
        int px = fix2int15(pipet_x[i]);
        int top_y_int    = fix2int15(pipet_y[i]);
        int bottom_y_int = fix2int15(pipeb_y[i]);
        int xi = px & ~1;

        if (xi < 0 || xi + PIPE_WIDTH > SCREEN_WIDTH) continue;
        if (pipe_just_reset[i]) continue;
   
        int gap_top = top_y_int + PIPE_HEIGHT;
        draw_top_pipe(px, gap_top, 48);
        draw_bottom_pipe(px, bottom_y_int);
      }

      // draw the bird sprite at its new position
      int bird_px = fix2int15(bird_x);
      int bird_py = fix2int15(bird_y);

      drawFlappyBirdSpriteRotated(bird_px + 16, bird_py + 8, bird_angle_deg);

      static char buf[64];
      setTextColor(WHITE);
      setTextSize(3);
      setTextWrap(0);

      // Score
      setCursor(320, 50);
      sprintf(buf, "%d", score);
      writeString(buf);
    }

    spare_time = FRAME_RATE - (time_us_32() - begin_time);

    if (spare_time > 0) {
      PT_YIELD_usec(spare_time);
    } else {
      PT_YIELD_usec(1);
    }

  }

  PT_END(pt);
}

// ==================================================
// === play mode thread
// ==================================================
// button, arcade, mic, imu

// alpha beta algorithm approx for sqrt of fix15 values of a^2 + b^2
static inline fix15 approx_sqrt(fix15 a, fix15 b) {
  fix15 ax = (a < 0) ? -a : a;
  fix15 ay = (b < 0) ? -b : b;
  fix15 max_v = (ax > ay) ? ax : ay;
  fix15 min_v = (ax > ay) ? ay : ax;
  return max_v + (min_v >> 2);
}

static PT_THREAD(protothread_play_mode(struct pt *pt)) {
  PT_BEGIN(pt);
  
  // Initialize mode button
  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);
  gpio_pull_up(BUTTON_PIN);
  
  // Initialize arcade button
  gpio_init(ARCADE_BUTTON_PIN);
  gpio_set_dir(ARCADE_BUTTON_PIN, GPIO_IN);
  gpio_pull_up(ARCADE_BUTTON_PIN);

  // Button state tracking
  static bool button_prev = true;
  static bool arcade_prev = true;
  
  // IMU state tracking
  static fix15 baseline_tilt = 0;
  static bool baseline_set = false;
  static int imu_fsm_state = 0;
  
  // Timing counters for different polling rates
  static uint32_t arcade_counter = 0;
  static uint32_t imu_counter = 0;
  
  while (1) {
    // ===== MODE BUTTON =====
    bool button_now = gpio_get(BUTTON_PIN);
    if (button_prev && !button_now) {
      if (state == STATE_END) {
        // Reset game
        init_all_pipes();
        for (int i = 0; i < NUM_PIPE_PAIRS; i++) {
          pipe_scored[i] = false;
        }
        bird_x = int2fix15(75);
        bird_y = int2fix15(216);
        bird_vy = int2fix15(0);
        state = STATE_START;
        // Reset IMU state
        baseline_set = false;
        imu_fsm_state = 0;
        
      } else if (state != STATE_FALL) {
        // Cycle through modes
        if (mode_state == STATE_BUTTON) {
          mode_state = STATE_MIC;
        } else if (mode_state == STATE_MIC) {
          mode_state = STATE_IMU;
          baseline_set = false;
          imu_fsm_state = 0;
        } else if (mode_state == STATE_IMU) {
          mode_state = STATE_BUTTON;
        }
      }
    }
    button_prev = button_now;

    // ===== ARCADE BUTTON  =====
    arcade_counter++;
    if (arcade_counter >= 5) {
      arcade_counter = 0;
      if (mode_state == STATE_BUTTON && state != STATE_END) {
        bool arcade_now = gpio_get(ARCADE_BUTTON_PIN);
        if (arcade_prev && !arcade_now) {
          if (state == STATE_START) {
            state = STATE_FALL;
          }
          bird_vy = int2fix15(-7);
          sfx_play(SFX_FLAP);
        }
        arcade_prev = arcade_now;
      }
    }

    // ===== MICROPHONE  =====
    // detect sudden changes in volume
    if (mode_state == STATE_MIC && state != STATE_END) {
      // get current audio level from mic 0-4095
      uint16_t sample = adc_read();
      // constant averaging to keep a noise baseline level
      noise_avg = noise_avg * int2fix15(sample) * 0.25;
      // compare sample to baseline
      int diff = sample - noise_avg;
      
      if (diff > MIC_THRESHOLD) {
        if (state == STATE_START) {
          state = STATE_FALL;
        }
        bird_vy = int2fix15(-6);
        sfx_play(SFX_FLAP);
      }
    }

    // ===== IMU  =====
    imu_counter++;
    if (imu_counter >= 20) {
      imu_counter = 0;
      if (mode_state == STATE_IMU && state != STATE_END) {
        mpu6050_read_raw(acceleration, gyro);

        fix15 ax = acceleration[0];
        fix15 ay = acceleration[1];
        fix15 az = acceleration[2];

        fix15 horizontal = approx_sqrt(ay, az);
        if (horizontal < 10) horizontal = 10;

        fix15 abs_ax = (ax < 0) ? -ax : ax;
        fix15 tilt = divfix(abs_ax, horizontal);

        if (!baseline_set) {
          baseline_tilt = tilt;
          baseline_set = true;
        }

        fix15 delta_tilt = tilt - baseline_tilt;
        if (delta_tilt < 0) delta_tilt = -delta_tilt;

        switch (imu_fsm_state) {
          case 0:
            if (delta_tilt > TILT_UP_THRESHOLD) {
              if (state == STATE_START) state = STATE_FALL;
              bird_vy = int2fix15(-7);
              sfx_play(SFX_FLAP);
              imu_fsm_state = 1;
            }
            break;
          case 1:
            if (delta_tilt < TILT_RESET_THRESHOLD) {
              imu_fsm_state = 2;
            }
            break;
          case 2:
            if (delta_tilt < TILT_RESET_THRESHOLD) {
              imu_fsm_state = 0;
            }
            break;
        }
      } else if (mode_state != STATE_IMU) {
        baseline_set = false;
        imu_fsm_state = 0;
      }
    }

    PT_YIELD_usec(1000);  
  }

  PT_END(pt);
}


// ========================================
// === core 1 main
// ========================================
void core1_main() {

  // adc initialize
  adc_init();
  adc_gpio_init(26); // ADC0
  adc_select_input(0);

  // i2c configuration for mpu6050
  i2c_init(I2C_CHAN, I2C_BAUD_RATE);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

  // MPU6050 initialization
  mpu6050_reset();
  mpu6050_read_raw(acceleration, gyro);

  // gameplay input thread
  pt_add_thread(protothread_play_mode);

  // Start the scheduler
  pt_schedule_start ;

}


// ========================================
// === core 0 main
// ========================================
int main() {
  // Overclock
  set_sys_clock_khz(150000, true) ;
  // Initialize stdio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;
  
  init_pipe_sprites();

  // initialize all pipe positions
  init_all_pipes();

  // ======= AUDIO INITIALIZATION =======
  
  // Initialize SPI channel for DAC - baud rate 20 MHz
  spi_init(SPI_PORT, 20000000);
  // Format SPI channel (channel, data bits per transfer, polarity, phase, order)
  spi_set_format(SPI_PORT, 16, 0, 0, 0);

  // SPI signals to GPIO ports
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_CS,   GPIO_FUNC_SPI);

  // Claim a free DMA channel
  sfx_dma_chan = dma_claim_unused_channel(true);

  // Get default config and tweak it
  sfx_dma_cfg = dma_channel_get_default_config(sfx_dma_chan);
  channel_config_set_transfer_data_size(&sfx_dma_cfg, DMA_SIZE_16); // 16-bit transfers
  channel_config_set_read_increment(&sfx_dma_cfg, true);           // increment through SFX buffer
  channel_config_set_write_increment(&sfx_dma_cfg, false);         // always write to same SPI TX register

  // Use DMA timer 0 to pace audio at 50 kHz to matche DELAY = 20 us
  dma_timer_set_fraction(0, 1, 3000);

  // DMA timer 0 is 0x3b on RP2040
  channel_config_set_dreq(&sfx_dma_cfg, 0x3b);

  // preconfigure DMA destination
  dma_channel_configure(
    sfx_dma_chan,
    &sfx_dma_cfg,
    &spi_get_hw(SPI_PORT)->dr,   // SPI0 TX FIFO
    NULL,                        // no source yet
    0,                           // no transfers yet
    false                        // DO NOT start
  );

  // LDAC pin keep low
  gpio_init(LDAC);
  gpio_set_dir(LDAC, GPIO_OUT);
  gpio_put(LDAC, 0);

  // ===== BACKGROUND MUSIC INIT (DAC B) =====
  init_bgm_table();
  init_bgm_arpeggio();

  // ==== SET UP AUDIO TIMER ISR ====
  hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM);
  irq_set_exclusive_handler(ALARM_IRQ, alarm_irq);
  irq_set_enabled(ALARM_IRQ, true);
  timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY;

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // core 0 used for drawing/animation
  pt_add_thread(protothread_anim);
  
  // start scheduler
  pt_schedule_start ;
}
