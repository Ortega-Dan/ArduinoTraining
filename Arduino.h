/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2005-2013 Arduino Team.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#define Arduino_h

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
  

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "binary.h"

#ifdef __cplusplus
extern "C"{
#endif

void yield(void);

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#define DEFAULT 0
#define EXTERNAL 1
#define INTERNAL 2
#else  
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
#define INTERNAL1V1 2
#define INTERNAL2V56 3
#else
#define INTERNAL 3
#endif
#define DEFAULT 1
#define EXTERNAL 0
#endif

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define interrupts() sei()
#define noInterrupts() cli()

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

    //Innobot Define

#define M1  1
#define M2  2
#define M3  3
#define M4  4

#define FORWARD 1       // Direction Forward
#define REVERSE 0       // Direction Reverse

#define J1 A0 // Pin Sensor Line Left
#define J2 A1 // Pin Sensor Line Right
#define J3 A2
#define J4 A3
#define J5 A4
#define J6 A5



#define BLACK 0
#define WHITE 1
#define VOID  3

#define LED1  13

#define CALIBRATE 0.80

#if defined(__AVR_ATmega32U4__)

#define M1_PWM  5     // Pin PWM H-Bridge 1A
#define M1_DIR  4     // Pin DIR H-Bridge 1A
#define M2_PWM  6     // Pin PWM H-Bridge 1B
#define M2_DIR  7           // Pin DIR H-Bridge 1B
#define M3_PWM  9     // Pin PWM H-Bridge 2A
#define M3_DIR  10          // Pin DIR H-Bridge 2A
#define M4_PWM  11      // Pin PWM H-Bridge 2B
#define M4_DIR  12            // Pin DIR H-Bridge 2B
//End Innobot Define

//RaveBot Define
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__)

#define M1_PWM  9     // Pin PWM H-Bridge 1A
#define M1_DIR  8     // Pin DIR H-Bridge 1A
#define M2_PWM  6     // Pin PWM H-Bridge 1B
#define M2_DIR  7           // Pin DIR H-Bridge 1B
#define M3_PWM  5     // Pin PWM H-Bridge 2A
#define M3_DIR  4           // Pin DIR H-Bridge 2A
#define M4_PWM  3     // Pin PWM H-Bridge 2B
#define M4_DIR  2           // Pin DIR H-Bridge 2B

#else

#define M1_PWM  5     // Pin PWM H-Bridge 1A
#define M1_DIR  4     // Pin DIR H-Bridge 1A
#define M2_PWM  6     // Pin PWM H-Bridge 1B
#define M2_DIR  7           // Pin DIR H-Bridge 1B
#define M3_PWM  9     // Pin PWM H-Bridge 2A
#define M3_DIR  10          // Pin DIR H-Bridge 2A
#define M4_PWM  11      // Pin PWM H-Bridge 2B
#define M4_DIR  12

#endif

#define TOWER   1
#define NO_TOWER  0 

//End RaveBot Define

//Transmedia Define
#define  Do0  16.351598
#define  DoS0 17.323914
#define  Re0  18.354048
#define  ReS0 19.445436
#define  Mi0  20.601722
#define  Fa0  21.826764
#define  FaS0 23.124651
#define  Sol0   24.499715
#define  SolS0  25.956544
#define  La0  27.5
#define  LaS0 29.135235
#define  Si0  30.867706
  
#define  Do1  32.703196
#define  DoS1 34.647829
#define  Re1  36.708096
#define  ReS1 38.890873
#define  Mi1  41.203445
#define  Fa1  43.653529
#define  FaS1 46.249303
#define  Sol1   48.999429
#define  SolS1  51.913087
#define  La1  55
#define  LaS1 58.27047
#define  Si1  61.735413
  
#define  Do2  65.406391
#define  DoS2 69.295658
#define  Re2  73.416192
#define  ReS2 77.781746
#define  Mi2  82.406889
#define  Fa2  87.307058
#define  FaS2 92.498606
#define  Sol2 97.998859
#define  SolS2  103.826174
#define  La2  110
#define  LaS2 116.54094
#define  Si2  123.470825
  
  
#define  Do3  130.812783
#define  DoS3 138.591315
#define  Re3  146.832384
#define  ReS3 155.563492
#define  Mi3  164.813778
#define  Fa3  174.614116
#define  FaS3 184.997211
#define  Sol3   195.997718
#define  SolS3  207.652349
#define  La3  220
#define  LaS3 233.081881
#define  Si3  246.941651
  
#define  Do4  261.625565
#define  DoS4 277.182631
#define  Re4  293.664768
#define  ReS4 311.126984
#define  Mi4  329.627557
#define  Fa4  349.228231
#define  FaS4 369.994423
#define  Sol4   391.995436
#define  SolS4  415.304698
#define  La4  440
#define  LaS4 466.163762
#define  Si4  493.883301
  
#define  Do5  523.251131
#define  DoS5 554.365262
#define  Re5  587.329536
#define  ReS5 622.253967
#define  Mi5  659.255114
#define  Fa5  698.456463
#define  FaS5 739.988845
#define  Sol5   783.990872
#define  SolS5  830.609395
#define  La5  880
#define  LaS5 932.327523
#define  Si5  987.766603
  
#define  Do6  1046.502261
#define  DoS6 1108.730524
#define  Re6  1174.659072
#define  ReS6 1244.507935
#define  Mi6  1318.510228
#define  Fa6  1396.912926
#define  FaS6 1479.977691
#define  Sol6   1567.981744
#define  SolS6  1661.21879
#define  La6  1760
#define  LaS6 1864.655046
#define  Si6  1975.533205
  
#define  Do7  2093.004522
#define  DoS7 2217.461048
#define  Re7  2349.318143
#define  ReS7 2489.01587
#define  Mi7  2637.020455
#define  Fa7  2793.825851
#define  FaS7 2959.955382
#define  Sol7   3135.963488
#define  SolS7  3322.437581
#define  La7  3520
#define  LaS7 3729.310092
#define  Si7  3951.06641
  
#define  Do8  4186.009045
#define  DoS8 4434.922096
#define  Re8  4698.636287
#define  ReS8 4978.03174
#define  Mi8  5274.040911
#define  Fa8  5587.651703
#define  FaS8 5919.910763
#define  Sol8   6271.926976
#define  SolS8  6644.875161
#define  La8  7040
#define  LaS8 7458.620184
#define  Si8  7902.13282
  
#define  Do9  8372.01809
#define  DoS9 8869.844191
#define  Re9  9397.272573
#define  ReS9 9956.063479
#define  Mi9  10548.08182
#define  Fa9  11175.30341
#define  FaS9 11839.82153
#define  Sol9   12543.85395
#define  SolS9  13289.75032
#define  La9  14080
#define  LaS9 14917.24037
#define  Si9  15804.26564
  
  
#define  Do10 16744.03618
#define  DoS10  17739.68838
#define  Re10 18794.54515
#define  ReS10  19912.12696
#define  Mi10 21096.16364
#define  Fa10 22350.60681
#define  FaS10  23679.64305
#define  Sol10  25087.7079
#define  SolS10 26579.50065
#define  La10 28160
#define  LaS10  29834.48074
#define  Si10 31608.53128


#define  redonda 1000
#define  blanca  500
#define  negra 250
#define  corchea  125
#define  semicorchea 62.5
#define  fusa 31.25
#define  semifusa  15.625

//#define MAX_DATA_POINTS 20

//End Transmedia Define
    
//Innobot Library functions

//Public
  void ledOn();
  void ledOff();
  void motorOn(uint8_t mtr, uint8_t dir);
  void motorOff(uint8_t mtr);
  void motorsOff(uint8_t m1, uint8_t m2);
  void motorSpeed(uint8_t mtr, uint8_t spd);
  int proximityRead(uint8_t sen);
  int lineRead(uint8_t sen);
  void lineCompare(uint8_t sen, int compare);
  void lineCalibrate(uint8_t sen, float calibrate);
  int ultrasoundRead(uint8_t sen);
  int sensorRead(uint8_t sen);
  void goForward(uint8_t m1, uint8_t m2);
  void goReverse(uint8_t m1, uint8_t m2);
  void turnRight(uint8_t m1, uint8_t m2);
  void turnLeft(uint8_t m1, uint8_t m2);
  int towerRead(void);

  void myTone(long unsigned int NOTE, unsigned long DURATION);
  void myTime(int times);

  //void Makey(unsigned int newDataPointsCount);

  //float process(float in);
//End Innobot Library functions 
    
// avr-libc defines _NOP() since 1.6.2
#ifndef _NOP
#define _NOP() do { __asm__ volatile ("nop"); } while (0)
#endif
typedef unsigned int word;

#define bit(b) (1UL << (b))

typedef bool boolean;
typedef uint8_t byte;    
    
void init(void);
void initVariant(void);

int atexit(void (*func)()) __attribute__((weak));

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
int analogRead(uint8_t);
void analogReference(uint8_t mode);
void analogWrite(uint8_t, int);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

void attachInterrupt(uint8_t, void (*)(void), int mode);
void detachInterrupt(uint8_t);

void setup(void);
void loop(void);

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.

#define analogInPinToBit(P) (P)

// On the ATmega1280, the addresses of some of the port registers are
// greater than 255, so we can't store them in uint8_t's.
extern const uint16_t PROGMEM port_to_mode_PGM[];
extern const uint16_t PROGMEM port_to_input_PGM[];
extern const uint16_t PROGMEM port_to_output_PGM[];

extern const uint8_t PROGMEM digital_pin_to_port_PGM[];
// extern const uint8_t PROGMEM digital_pin_to_bit_PGM[];
extern const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[];
extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
// 
// These perform slightly better as macros compared to inline functions
//
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )
#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

#define NOT_A_PIN 0
#define NOT_A_PORT 0

#define NOT_AN_INTERRUPT -1

#ifdef ARDUINO_MAIN
#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 10
#define PK 11
#define PL 12
#endif

#define NOT_ON_TIMER 0
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER1C 5
#define TIMER2  6
#define TIMER2A 7
#define TIMER2B 8

#define TIMER3A 9
#define TIMER3B 10
#define TIMER3C 11
#define TIMER4A 12
#define TIMER4B 13
#define TIMER4C 14
#define TIMER4D 15
#define TIMER5A 16
#define TIMER5B 17
#define TIMER5C 18

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
#include "WCharacter.h"
#include "WString.h"
#include "HardwareSerial.h"
#include "USBAPI.h"
#if defined(HAVE_HWSERIAL0) && defined(HAVE_CDCSERIAL)
#error "Targets with both UART0 and CDC serial not supported"
#endif

uint16_t makeWord(uint16_t w);
uint16_t makeWord(byte h, byte l);

#define word(...) makeWord(__VA_ARGS__)

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);

void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
void noTone(uint8_t _pin);

// WMath prototypes
long random(long);
long random(long, long);
void randomSeed(unsigned long);
long map(long, long, long, long, long);

#endif

#include "pins_arduino.h"


