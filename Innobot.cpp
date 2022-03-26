/*
 Innobot.cpp - Innobot library for Innobot IDE using 16 bit timers- Version 1.1
 Copyright (c) 2015 Alejandro Martinez - Daniel Hernandez.  All right reserved.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.3 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/* 
V1.3 - 06/02/2016
Include new functions myTone myTime transmedia definitions
Include new function innoMakey
Inlcude Innobot functions to Arduino.h
Fixed turnLeft bug
Fixed motorsOff bug

V1.1 - 22/06/2015
Fixed ultrasoundRead function bugs
Inlcude Innobot functions to Arduino.h
Include lineCompare function
Inlcude motorsOff function
*/


#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "pins_arduino.h"
#include "WCharacter.h"
#include "WString.h"
#include "HardwareSerial.h"
#include "USBAPI.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <avr/io.h>
#include "binary.h"
    
    
  static uint8_t motor_speed[4]={255,255,255,255};
  static int line_level=450;
  static int lineJ1=450;
  static int lineJ2=450;
  static int lineJ3=450;
  static int lineJ4=450;
  static int lineJ5=450;
  static int lineJ6=450;
  static int last_duration = 0;
  static int time;
  

void ledOn(void){
	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
}

void ledOff(void){
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
}

void motorOn(uint8_t mtr, uint8_t dir){
    pinMode(M1_PWM, OUTPUT);
	pinMode(M2_PWM, OUTPUT);
	pinMode(M3_PWM, OUTPUT);
	pinMode(M4_PWM, OUTPUT);
	pinMode(M1_DIR, OUTPUT);
	pinMode(M2_DIR, OUTPUT);
	pinMode(M3_DIR, OUTPUT);
	pinMode(M4_DIR, OUTPUT);
	

	switch(mtr){
		case M1:
			digitalWrite(M1_DIR, !dir);
			analogWrite(M1_PWM, motor_speed[0]);
		break;
		
		case M2:
			digitalWrite(M2_DIR, dir);
			analogWrite(M2_PWM, motor_speed[1]);
		break;
		
		case M3:
			digitalWrite(M3_DIR, dir);
			analogWrite(M3_PWM, motor_speed[2]);
		break;
		
		case M4:
			digitalWrite(M4_DIR, dir);
			analogWrite(M4_PWM, motor_speed[3]);
		break;
	}
}

void motorOff(uint8_t mtr){

	pinMode(M1_PWM, OUTPUT);
	pinMode(M2_PWM, OUTPUT);
	pinMode(M3_PWM, OUTPUT);
	pinMode(M4_PWM, OUTPUT);
	pinMode(M1_DIR, OUTPUT);
	pinMode(M2_DIR, OUTPUT);
	pinMode(M3_DIR, OUTPUT);
	pinMode(M4_DIR, OUTPUT);

	switch(mtr){
		case M1:
			analogWrite(M1_PWM, 0);
			break;
		case M2:
			analogWrite(M2_PWM, 0);
			break;
		case M3:
			analogWrite(M3_PWM, 0);
			break;
		case M4:
			analogWrite(M4_PWM, 0);
			break;
	}
}

void motorsOff(uint8_t m1, uint8_t m2){
	motorOff(m1);
	motorOff(m2);
}

void motorSpeed(uint8_t mtr, uint8_t spd){
    pinMode(M1_PWM, OUTPUT);
	pinMode(M2_PWM, OUTPUT);
	pinMode(M3_PWM, OUTPUT);
	pinMode(M4_PWM, OUTPUT);

	spd = (int)(((long)spd*255)/100);
	switch(mtr){
		case M1:
			motor_speed[0]  = spd;
			break;
		case M2:
			motor_speed[1]  = spd;
			break;
		case M3:
			motor_speed[2]  = spd;
			break;
		case M4:
			motor_speed[3]  = spd;
			break;
	}
}



int proximityRead(uint8_t sen){
	if(analogRead(sen) > 700){
		return 0;
	}
	else{
		return 1;
	}
}

int towerRead(void){
	if(analogRead(A2) > 256){
		return 0;
	}
	else{
		return 1;
	}
}

int lineRead(uint8_t sen){
  

	int level = 0;
	level = analogRead(sen);
		
	if(level < 50){
		return 3;
	}
	
	switch(sen){
		case J1:
			return (level > lineJ1) ?  1 :  0;
		break;
		case J2:
			return (level > lineJ2) ?  1 :  0;
		break;
		case J3:
			return (level > lineJ3) ?  1 :  0;
		break;
		case J4:
			return (level > lineJ4) ?  1 :  0;
		break;
		case J5:
			return (level > lineJ5) ?  1 :  0;
		break;
		
		case J6:
			return (level > lineJ6) ?  1 :  0;
		break;
		
		default:
			return (level > line_level) ?  1 :  0;
		break;
	}
}

void lineCompare(uint8_t sen, int compare){
 
	switch(sen){
		case J1:
			lineJ1 = compare;
		break;
		case J2:
			lineJ2 = compare;
		break;
		case J3:
			lineJ3 = compare;
		break;
		case J4:
			lineJ4 = compare;
		break;
		case J5:
			lineJ5 = compare;
		break;
		case J6:
			lineJ6 = compare;
		break;
	}
}

void lineCalibrate(uint8_t sen, float calibrate){
  
  	int level = 0;
	
	if (calibrate == WHITE){
		calibrate = 0.7;
	}else if(calibrate == BLACK){
		calibrate = 1.7;
	}
	
	level = analogRead(sen);
	switch(sen){
		case J1:
			lineJ1 = int((float)level * (calibrate));
		break;
		case J2:
			lineJ2 = int((float)level * (calibrate));
		break;
		case J3:
			lineJ3 = int((float)level * (calibrate));
		break;
		case J4:
			lineJ4 = int((float)level * (calibrate));
		break;
		case J5:
			lineJ5 = int((float)level * (calibrate));
		break;
		case J6:
			lineJ6 = int((float)level * (calibrate));
		break;
	}
}


int ultrasoundRead(uint8_t sen){	
int count = 5;
int RangeInCentimeters = 0;
		pinMode(sen, OUTPUT);
		digitalWrite(sen, LOW);
		delayMicroseconds(2);
		digitalWrite(sen, HIGH);
		delayMicroseconds(10);
		digitalWrite(sen,LOW);
		pinMode(sen,INPUT);
		digitalWrite(sen, HIGH); //Pull-up por probar
		int duration;
		duration = (int)pulseIn(sen,HIGH,7000);
		RangeInCentimeters += ((long)duration/29/2);
	if (RangeInCentimeters < 1 || RangeInCentimeters > 120)
	{
		return 120;
	}
	return RangeInCentimeters;
	
}

int sensorRead(uint8_t sen){
	return analogRead(sen);
}


void goForward(uint8_t mL, uint8_t mR){
	motorOn(mL, FORWARD);
	motorOn(mR, FORWARD);
}

void goReverse(uint8_t mL, uint8_t mR){
	motorOn(mL, REVERSE);
	motorOn(mR, REVERSE);
}

void turnRight(uint8_t mL, uint8_t mR){
	motorOn(mL, FORWARD); //Left
	motorOn(mR, REVERSE); //Right
}

void turnLeft(uint8_t mL, uint8_t mR){
	motorOn(mL, REVERSE); //Left
	motorOn(mR, FORWARD); //Right
}


void myTime(int times) {
	time=times;	
 } 
void myTone(long unsigned int NOTE, unsigned long DURATION) {
  tone(8,NOTE,DURATION); 
  delay(time);
 } 

