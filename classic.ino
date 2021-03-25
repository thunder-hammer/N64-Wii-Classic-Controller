/*
 * Sample sketch which makes the Arduino to impersonate a Classic Controller.
 *
 * Use a PushButton connected to the digital pin 2 and GND. It will trigger
 * a A button press of the impersonated Classic Controller.
 *
 * Copyright (c) 2011 Peter Brinkmann (peter.brinkmann@gmail.com)
 *
 * For information on usage and redistribution, and for a DISCLAIMER OF ALL
 * WARRANTIES, see the file, "LICENSE.txt," in this distribution.
 */

 /**
 * Gamecube controller to Nintendo 64 adapter
 * by Andrew Brown
 * Rewritten for N64 to HID by Peter Den Hartog
 */

/**
 * To use, hook up the following to the Arduino Duemilanove:
 * Digital I/O 2: N64 serial line
 * All appropriate grounding and power lines
 */

//#define DEBUG

#include "pins_arduino.h"

#define N64_PIN 2
#define N64_PIN_DIR DDRD
// these two macros set arduino pin 2 to input or output, which with an
// external 1K pull-up resistor to the 3.3V rail, is like pulling it high or
// low.  These operations translate to 1 op code, which takes 2 cycles
#define N64_HIGH DDRD &= ~0x04
#define N64_LOW DDRD |= 0x04
#define N64_QUERY (PIND & 0x04)


int pin_y_pot = A3;
int pin_x_pot = A2;
int total_3v = 0;
int max_3v = 0;
int min_3v = 10000;
int count_3v = 0;
int average_3v = 0;
//int x_rest_val = 0;
//int x_rest_val_count = 0;
//int x_rest_val_total = 0;
//int x_rest_val_max = 0;
//int x_rest_val_min = 10000;
//int y_rest_val = 0;
//int y_rest_val_total = 0;
//int y_rest_val_count = 0;
//int y_rest_val_max = 0;
//int y_rest_val_min = 10000;

// 8 bytes of data that we get from the controller
struct {
    // bits: 0, 0, 0, start, y, x, b, a
    unsigned char data1;
    // bits: 1, L, R, Z, Dup, Ddown, Dright, Dleft
    unsigned char data2;
    char stick_x;
    char stick_y;
} N64_status;
char N64_raw_dump[33]; // 1 received bit per byte


void N64_send(unsigned char *buffer, char length);
void N64_get();
void print_N64_status();
void translate_raw_data();


#include "crc_table.h"

#include "Wire.h" // This seems redundant, but we need to declare this
                  // dependency in the pde file or else it won't be included
                  // in the build.

#include "wiimote.h"

// Classic Controller Buttons
int bdl = 0; // D-Pad Left state
int bdr = 0; // D-Pad Right state
int bdu = 0; // D-Pad Up state
int bdd = 0; // D-Pad Down state
int ba = 0; // A button state
int bb = 0; // B button state
int bx = 0; // X button state
int by = 0; // Y button state
int bl = 0; // L button state
int br = 0; // R button state
int bm = 0; // MINUS button state
int bp = 0; // PLUS button state
int bhome = 0; // HOME button state
int bzl = 0; // ZL button state
int bzr = 0; // ZR button state

/*
 * Analog Buttons.
 * They are initialized with center values from the calibration buffer.
 */
byte lx = calbuf[2]>>2;
byte ly = calbuf[5]>>2;
byte rx = calbuf[8]>>3;
byte ry = calbuf[11]>>3;

//int pinRight = 2;


// Wiimote button data stream
byte *stream_callback(byte *buffer) {
	wiimote_write_buffer(buffer, bdl, bdr, bdu, bdd, ba, bb, bx, by, bl, br,
			bm, bp, bhome, lx, ly, rx, ry, bzl, bzr);

	return buffer;
}



void translate_raw_data()
{
    // The get_N64_status function sloppily dumps its data 1 bit per byte
    // into the get_status_extended char array. It's our job to go through
    // that and put each piece neatly into the struct N64_status
    int i;
    memset(&N64_status, 0, sizeof(N64_status));
    // line 1
    // bits: A, B, Z, Start, Dup, Ddown, Dleft, Dright
    for (i=0; i<8; i++) {
        N64_status.data1 |= N64_raw_dump[i] ? (0x80 >> i) : 0;
    }
    // line 2
    // bits: 0, 0, L, R, Cup, Cdown, Cleft, Cright
    for (i=0; i<8; i++) {
        N64_status.data2 |= N64_raw_dump[8+i] ? (0x80 >> i) : 0;
    }
    // line 3
    // bits: joystick x value
    // These are 8 bit values centered at 0x80 (128)
    for (i=0; i<8; i++) {
        N64_status.stick_x |= N64_raw_dump[16+i] ? (0x80 >> i) : 0;
    }
    for (i=0; i<8; i++) {
        N64_status.stick_y |= N64_raw_dump[24+i] ? (0x80 >> i) : 0;
    }
}

/**
 * This sends the given byte sequence to the controller
 * length must be at least 1
 * Oh, it destroys the buffer passed in as it writes it
 */
void N64_send(unsigned char *buffer, char length)
{
    // Send these bytes
    char bits;
    
    bool bit;

    // This routine is very carefully timed by examining the assembly output.
    // Do not change any statements, it could throw the timings off
    //
    // We get 16 cycles per microsecond, which should be plenty, but we need to
    // be conservative. Most assembly ops take 1 cycle, but a few take 2
    //
    // I use manually constructed for-loops out of gotos so I have more control
    // over the outputted assembly. I can insert nops where it was impossible
    // with a for loop
    
    asm volatile (";Starting outer for loop");
outer_loop:
    {
        asm volatile (";Starting inner for loop");
        bits=8;
inner_loop:
        {
            // Starting a bit, set the line low
            asm volatile (";Setting line to low");
            N64_LOW; // 1 op, 2 cycles

            asm volatile (";branching");
            if (*buffer >> 7) {
                asm volatile (";Bit is a 1");
                // 1 bit
                // remain low for 1us, then go high for 3us
                // nop block 1
                asm volatile ("nop\nnop\nnop\nnop\nnop\n");
                
                asm volatile (";Setting line to high");
                N64_HIGH;

                // nop block 2
                // we'll wait only 2us to sync up with both conditions
                // at the bottom of the if statement
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              );

            } else {
                asm volatile (";Bit is a 0");
                // 0 bit
                // remain low for 3us, then go high for 1us
                // nop block 3
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\nnop\n"  
                              "nop\n");

                asm volatile (";Setting line to high");
                N64_HIGH;

                // wait for 1us
                asm volatile ("; end of conditional branch, need to wait 1us more before next bit");
                
            }
            // end of the if, the line is high and needs to remain
            // high for exactly 16 more cycles, regardless of the previous
            // branch path

            asm volatile (";finishing inner loop body");
            --bits;
            if (bits != 0) {
                // nop block 4
                // this block is why a for loop was impossible
                asm volatile ("nop\nnop\nnop\nnop\nnop\n"  
                              "nop\nnop\nnop\nnop\n");
                // rotate bits
                asm volatile (";rotating out bits");
                *buffer <<= 1;

                goto inner_loop;
            } // fall out of inner loop
        }
        asm volatile (";continuing outer loop");
        // In this case: the inner loop exits and the outer loop iterates,
        // there are /exactly/ 16 cycles taken up by the necessary operations.
        // So no nops are needed here (that was lucky!)
        --length;
        if (length != 0) {
            ++buffer;
            goto outer_loop;
        } // fall out of outer loop
    }

    // send a single stop (1) bit
    // nop block 5
    asm volatile ("nop\nnop\nnop\nnop\n");
    N64_LOW;
    // wait 1 us, 16 cycles, then raise the line 
    // 16-2=14
    // nop block 6
    asm volatile ("nop\nnop\nnop\nnop\nnop\n"
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\n");
    N64_HIGH;

}

void N64_get()
{
    // listen for the expected 8 bytes of data back from the controller and
    // blast it out to the N64_raw_dump array, one bit per byte for extra speed.
    // Afterwards, call translate_raw_data() to interpret the raw data and pack
    // it into the N64_status struct.
    asm volatile (";Starting to listen");
    unsigned char timeout;
    char bitcount = 32;
    char *bitbin = N64_raw_dump;

    // Again, using gotos here to make the assembly more predictable and
    // optimization easier (please don't kill me)
read_loop:
    timeout = 0x3f;
    // wait for line to go low
    while (N64_QUERY) {
        if (!--timeout)
            return;
    }
    // wait approx 2us and poll the line
    asm volatile (
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
                  "nop\nnop\nnop\nnop\nnop\n"  
            );
    *bitbin = N64_QUERY;
    ++bitbin;
    --bitcount;
    if (bitcount == 0)
        return;

    // wait for line to go high again
    // it may already be high, so this should just drop through
    timeout = 0x3f;
    while (!N64_QUERY) {
        if (!--timeout)
            return;
    }
    goto read_loop;

}

//void add_3v_datapoint(int val){
//  if(val < min_3v){
//    min_3v = val;
//  }
//  if(val > max_3v){
//    max_3v = val;
//  }
//  total_3v += val;
//  count_3v++;
//}
//
//void set_3v_average(){
//  average_3v = total_3v/count_3v;
//}

void n64_to_wii()
{

  //WII BUTTON DEFINITIONS FOR THIS CODE. SET THESE IN THIS FUNCTION
  //int bdl // D-Pad Left state
  //int bdr // D-Pad Right state
  //int bdu // D-Pad Up state
  //int bdd // D-Pad Down state
  //int ba // A button state
  //int bb // B button state
  //bx // X button state
  //int by // Y button state
  //int bl // L button state
  //br // R button state
  //bm // MINUS button state
  //int bp // PLUS button state
  //bhome // HOME button state
  //bzl // ZL button state
  //bzr // ZR button state
  /*
   * Analog Buttons.
   * They are initialized with center values from the calibration buffer.
   */ 
  //byte lx = calbuf[2]>>2;
  //byte ly = calbuf[5]>>2;
  //byte rx = calbuf[8]>>3;
  //byte ry = calbuf[11]>>3;

  //N64 BUTTON MAPINGS THESE CAN BE ASSIGNED FROM
  //Start: N64_status.data1 & 16 ? 1:0);
  uint8_t n64_start = N64_status.data1 & 16 ? 1:0;
  //Z: N64_status.data1 & 32 ? 1:0
  uint8_t n64_z = N64_status.data1 & 32 ? 1:0;
  //B: N64_status.data1 & 64 ? 1:0
  uint8_t n64_b = N64_status.data1 & 64 ? 1:0;
  //A: N64_status.data1 & 128 ? 1:0
  uint8_t n64_a = N64_status.data1 & 128 ? 1:0;
  //L: N64_status.data2 & 32 ? 1:0
  uint8_t n64_l = N64_status.data2 & 32 ? 1:0;
  //R: N64_status.data2 & 16 ? 1:0
  uint8_t n64_r = N64_status.data2 & 16 ? 1:0;

  //Cup: N64_status.data2 & 0x08 ? 1:0
  uint8_t n64_cup = N64_status.data2 & 0x08 ? 1:0;//31:16;
  //Cdown: N64_status.data2 & 0x04 ? 1:0
  uint8_t n64_cdown = N64_status.data2 & 0x04 ? 1:0;//0:16;
  //Cright: N64_status.data2 & 0x01 ? 1:0
  uint8_t n64_cright = N64_status.data2 & 0x01 ? 1:0;//31:16;
  //Cleft: N64_status.data2 & 0x02 ? 1:0
  uint8_t n64_cleft = N64_status.data2 & 0x02 ? 1:0;//0:16;
  
  //Dup: N64_status.data1 & 0x08 ? 1:0
  uint8_t n64_dup = N64_status.data1 & 0x08 ? 1:0;
  //Ddown: N64_status.data1 & 0x04 ? 1:0
  uint8_t n64_ddown = N64_status.data1 & 0x04 ? 1:0;
  //Dright: N64_status.data1 & 0x01 ? 1:0
  uint8_t n64_dright = N64_status.data1 & 0x01 ? 1:0;
  //Dleft: N64_status.data1 & 0x02 ? 1:0
  uint8_t n64_dleft = N64_status.data1 & 0x02 ? 1:0;

  //Stick X: N64_status.stick_x, DEC //-70 left to 70 right
  //int8_t n64_x = N64_status.stick_x / 2 + 32;
  //Stick Y: N64_status.stick_y, DEC //60 up to -80 down
  //int8_t n64_y = N64_status.stick_y / 2 + 32;

//  if(x_rest_val == 0){
//    x_rest_val_count +=1;
//    int v = analogRead(pin_y_pot);
//    x_rest_val_total += v;
//    if(v > x_rest_val_max){
//      x_rest_val_max = v;
//    }
//    if (v < x_rest_val_min){
//      x_rest_val_min = v;
//    }
//    if(x_rest_val_count == 30){
//      x_rest_val = x_rest_val_total/30;
//    }
//  }
//  
//  if(y_rest_val == 0){
//    y_rest_val_count +=1;
//    int v = analogRead(pin_y_pot);
//    y_rest_val_total += v;
//    if(v > y_rest_val_max){
//      y_rest_val_max = v;
//    }
//    if (v < y_rest_val_min){
//      y_rest_val_min = v;
//    }
//    if(y_rest_val_count == 30){
//      y_rest_val = y_rest_val_total/30;
//    }
//  }

  int val_x_pot = analogRead(pin_x_pot);
  int val_y_pot = analogRead(pin_y_pot);

  
  //assume that this will be used.
//  int d3v = val_3v - average_3v;
//  int px = val_x_pot * 75 / val_3v;
//  int py = val_y_pot * 75 / val_3v;
//  int subx = px * d3v / 75;
//  int suby = py * d3v / 75;
//  //Stick X: N64_status.stick_x, DEC //-70 left to 70 right
//  //int8_t n64_x = N64_status.stick_x / 2 + 32;
//  int n64_x_i = 32;
//  //Stick Y: N64_status.stick_y, DEC //60 up to -80 down
//  //int8_t n64_y = N64_status.stick_y / 2 + 32;
//  int n64_y_i = 32;
//
//  if(count_3v < 100){
//    add_3v_datapoint(analogRead(pin_3v));
//  }
//  else if(count_3v == 100){
//    set_3v_average();
//    count_3v = 101;
//  }


  int8_t n64_x;
  int8_t n64_y;

  n64_x = 32;
  n64_y = 32;

  //center y is 390 bottom 85 top 664. this puts 274 above and 305 below
  //center x is 480 bottom 207 top 699 this puts 220 above and 270 below

  n64_x = ((val_x_pot - 525)*32)/250 + 32;
  n64_y = ((val_y_pot - 525)*32)/250 + 32;
  
  if(n64_x > 63){n64_x = 63;}
  if(n64_x < 0){n64_x = 0;}
  if(n64_y > 63){n64_y = 63;}
  if(n64_y < 0){n64_y = 0;}

  
  #ifdef DEBUG
  //delay(100);
  Serial.print(n64_x);
  Serial.print("\t");
  Serial.print(n64_y);
  Serial.print("\t");
  Serial.println();
  #endif

  //BUTTON MAPINGS ARE DEFINED BELOW THIS POINT. JUST DEFINE THE ONE YOU WANT.


  #define SUPER_SMASH_BROS_BRAWL

  ly = n64_y;
  lx = n64_x;

  #ifdef MARIO_KART_WII
  bdl   =   n64_dleft;  // D-Pad Left state
  bdr   =   n64_dright; // D-Pad Right state
  bdu   =   n64_r;      // D-Pad Up state
  bdd   =   n64_ddown;  // D-Pad Down state
  ba    =   n64_a;      // A button state
  bb    =   n64_b;      // B button state
  bx    =   n64_cdown;  // X button state
  by    =   n64_cleft;  // Y button state
  bl    =   n64_z;      // L button state
  br    =   n64_l;      // R button state
  bm    =   0;          // MINUS button state
  bp    =   n64_start;  // PLUS button state
  bhome =   0;          // HOME button state
  bzl   =   0;          // ZL button state
  bzr   =   0;          // ZR button state
  ry = n64_cup ? 31:16;
  if(ry == 16){
    ry = n64_cdown ? 0:16;
  }
  rx = n64_cright ? 31:16;
  if(rx == 16){
    rx = n64_cleft ? 0:16;
  }
  #endif

  #if defined(MARIO_PARTY_2_N64) ||\
  defined(MARIO_KART_N64)

  bdl   =   n64_dleft;  // D-Pad Left state
  bdr   =   n64_dright; // D-Pad Right state
  bdu   =   n64_dup;    // D-Pad Up state
  bdd   =   n64_ddown;  // D-Pad Down state
  ba    =   n64_a;      // A button state
  bb    =   n64_b;      // B button state
  bx    =   0;          // X button state
  by    =   0;          // Y button state
  bl    =   n64_z;      // L button state
  br    =   n64_r;      // R button state
  bm    =   0;          // MINUS button state
  bp    =   n64_start;  // PLUS button state
  bhome =   0;          // HOME button state
  bzl   =   n64_l;      // ZL button state
  bzr   =   0;          // ZR button state
  ry = n64_cup ? 31:16;
  if(ry == 16){
    ry = n64_cdown ? 0:16;
  }
  rx = n64_cright ? 31:16;
  if(rx == 16){
    rx = n64_cleft ? 0:16;
  }
  #endif


  #ifdef SUPER_SMASH_BROS_N64

  bdl   =   0;        // D-Pad Left state
  bdr   =   0;        // D-Pad Right state
  bdu   =   n64_l;        // D-Pad Up state
  bdd   =   0;        // D-Pad Down state
  ba    =   n64_a;    // A button state
  bb    =   n64_b;    // B button state
  bx    =   0;        // X button state
  by    =   0;        // Y button state
  bl    =   n64_z;    // L button state
  br    =   0;        // R button state
  bm    =   0;        // MINUS button state
  bp    =   n64_start;// PLUS button state
  bhome =   0;        // HOME button state
  bzl   =   0;        // ZL button state
  bzr   =   n64_r;    // ZR button state
  ry = n64_cup ? 31:16;
  if(ry == 16){
    ry = n64_cdown ? 0:16;
  }
  rx = n64_cright ? 31:16;
  if(rx == 16){
    rx = n64_cleft ? 0:16;
  }
  #endif

  #ifdef SUPER_SMASH_BROS_BRAWL
  bdl   =   n64_l;      // D-Pad Left state
  bdr   =   0;          // D-Pad Right state
  bdu   =   0;          // D-Pad Up state
  bdd   =   0;          // D-Pad Down state
  ba    =   n64_a;      // A button state
  bb    =   n64_b;      // B button state
  bx    =   n64_cdown || n64_cup;  // X button state
  by    =   n64_cleft || n64_cright;  // Y button state
  bl    =   n64_z;      // L button state
  br    =   0;//n64_cup;    // R button state
  bm    =   0;//n64_cright; // MINUS button state
  bp    =   n64_start;  // PLUS button state
  bhome =   0;          // HOME button state
  bzl   =   0;          // ZL button state
  bzr   =   n64_r;      // ZR button state
  ry    =   16;
  rx    =   16;
  #endif

  #ifdef SUPER_NINTENDO_VIRTUAL_CONSOLE
  bdl   =   n64_dleft;  // D-Pad Left state
  bdr   =   n64_dright; // D-Pad Right state
  bdu   =   n64_dup;    // D-Pad Up state
  bdd   =   n64_ddown;  // D-Pad Down state
  ba    =   n64_cdown;  // A button state
  bb    =   n64_a;      // B button state
  bx    =   n64_cleft;  // X button state
  by    =   n64_b;      // Y button state
  bl    =   n64_l;      // L button state
  br    =   n64_r;      // R button state
  bm    =   n64_z;      // MINUS button state
  bp    =   n64_start;  // PLUS button state
  bhome =   0;          // HOME button state
  bzl   =   0;          // ZL button state
  bzr   =   0;          // ZR button state
  #endif
  
//  
//  //Start: N64_status.data1 & 16 ? 1:0);
//  bp = N64_status.data1 & 16 ? 1:0;
//  //Z: N64_status.data1 & 32 ? 1:0
//  bl = N64_status.data1 & 32 ? 1:0;
//  //B: N64_status.data1 & 64 ? 1:0
//  bb = N64_status.data1 & 64 ? 1:0;
//  //A: N64_status.data1 & 128 ? 1:0
//  ba = N64_status.data1 & 128 ? 1:0;
//  //L: N64_status.data2 & 32 ? 1:0
//  bx = N64_status.data2 & 32 ? 1:0;
//  //R: N64_status.data2 & 16 ? 1:0
//  bdu = N64_status.data2 & 16 ? 1:0;
//
//  //Cup: N64_status.data2 & 0x08 ? 1:0
//  ry = N64_status.data2 & 0x08 ? 31:16;
//  //Cdown: N64_status.data2 & 0x04 ? 1:0
//  if(ry == 16){
//    ry = N64_status.data2 & 0x04 ? 0:16;
//  }
//  //Cright: N64_status.data2 & 0x01 ? 1:0
//  rx = N64_status.data2 & 0x01 ? 31:16;
//  //Cleft: N64_status.data2 & 0x02 ? 1:0
//  if(rx == 16){
//    rx = N64_status.data2 & 0x02 ? 0:16;
//  }
//  
//  //Dup: N64_status.data1 & 0x08 ? 1:0
//  //bdu = N64_status.data1 & 0x08 ? 1:0;
//  //Ddown: N64_status.data1 & 0x04 ? 1:0
//  bdd = N64_status.data1 & 0x04 ? 1:0;
//  //Dright: N64_status.data1 & 0x01 ? 1:0
//  bdr = N64_status.data1 & 0x01 ? 1:0;
//  //Dleft: N64_status.data1 & 0x02 ? 1:0
//  bdl = N64_status.data1 & 0x02 ? 1:0;
//  
//  //Stick X: N64_status.stick_x, DEC //-70 left to 70 right
//  int x_temp = N64_status.stick_x / 2 + 32;
//  //Stick Y: N64_status.stick_y, DEC //60 up to -80 down
//  int y_temp = N64_status.stick_y / 2 + 32;
//  if(x_temp > 63){x_temp = 63;}
//  if(x_temp < 0){x_temp = 0;}
//  if(y_temp > 63){y_temp = 63;}
//  if(y_temp < 0){y_temp = 0;}
//  if(N64_status.stick_y < 5 && N64_status.stick_y > -5){y_temp = calbuf[5] >> 2;}
//  if(N64_status.stick_x < 5 && N64_status.stick_x > -5){x_temp = calbuf[2] >> 2;}
//  lx = x_temp;
//  ly = y_temp;
  
  #ifdef DEBUG
    //Serial.print("Stick X:");
    //Serial.print(N64_status.stick_x, DEC);
    //Serial.print(" out: ");
    //Serial.println(x_temp, DEC);
    //Serial.print("Stick Y:");
    //Serial.print(N64_status.stick_y, DEC);
    //Serial.print(" out: ");
    //Serial.println(y_temp, DEC);
    
   for (int i=0; i<16; i++) {
       Serial.print(N64_raw_dump[i], DEC);
    }
    Serial.print(' ');
    Serial.print(N64_status.stick_x, DEC);
    Serial.print(' ');
    Serial.print(N64_status.stick_y, DEC);
    Serial.print(" \n");
  #endif
}

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  
	// Set PushButton pins as input, turning pull-up on
	//pinMode(pinRight, INPUT);
	//digitalWrite(pinRight, HIGH);

 
  // Communication with gamecube controller on this pin
  // Don't remove these lines, we don't want to push +5V to the controller
  digitalWrite(N64_PIN, LOW);  
  pinMode(N64_PIN, INPUT);


  // Initialize the gamecube controller by sending it a null byte.
  // This is unnecessary for a standard controller, but is required for the
  // Wavebird.
  unsigned char initialize = 0x00;
  noInterrupts();
  N64_send(&initialize, 1);

  // Stupid routine to wait for the gamecube controller to stop
  // sending its response. We don't care what it is, but we
  // can't start asking for status if it's still responding
  int x;
  for (x=0; x<64; x++) {
      // make sure the line is idle for 64 iterations, should
      // be plenty.
      if (!N64_QUERY)
          x = 0;
  }

  // Query for the gamecube controller's status. We do this
  // to get the 0 point for the control stick.
  unsigned char command[] = {0x01};
  N64_send(command, 1);
  // read in data and dump it to N64_raw_dump
  N64_get();
  interrupts();
  translate_raw_data();  

  
  // Prepare wiimote communications
  wiimote_stream = stream_callback;
  wiimote_init();
}

void loop() {
	/*
	 * If PushButton is pressed, pinRight will be LOW.
	 * This value is then inverted and given to variable "bdr", that represents
	 * D-Pad RIGHT button.
	 */
	//ba = digitalRead(pinRight);

	    int i;
    unsigned char data, addr;

    // Command to send to the gamecube
    // The last bit is rumble, flip it to rumble
    // yes this does need to be inside the loop, the
    // array gets mutilated when it goes through N64_send
    unsigned char command[] = {0x01};

    // don't want interrupts getting in the way
    noInterrupts();
    // send those 3 bytes
    N64_send(command, 1);
    // read in data and dump it to N64_raw_dump
    N64_get();
    // end of time sensitive code
    interrupts();

    // translate the data in N64_raw_dump to something useful
    translate_raw_data();

    n64_to_wii();
}
