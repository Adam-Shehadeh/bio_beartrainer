/*	bio_beartrainer.c 	
 * 	Adam Shehadeh
 * 	11/14/2022
 * 	This is a C program to control a raspberry pi setup with breadboard, lcd screen, 9 buttons, and 6 lights. 
 * 	
 * 	Notes:
 * 		-LCD Tutorial: https://www.circuitbasics.com/raspberry-pi-lcd-set-up-and-programming-in-c-with-wiringpi/
 */

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <signal.h>
#include <pthread.h>
#include <lcd.h>
#include <string.h>
#include "bio_beartrainer.h"

// Starts signals to ensure that cleanup gets called on closing
void start_signals()
{
	signal(SIGHUP, bbt_cleanup_pins);	//	Hangup 
	signal(SIGINT, bbt_cleanup_pins);	//	Interrupt
	signal(SIGQUIT, bbt_cleanup_pins);	//	Quit
	signal(SIGABRT, bbt_cleanup_pins);	//	Abort
	printf("start_signals() has completed\n");
}

void start_event_handlers()
{
	wiringPiISR(BTN_K1, INT_EDGE_FALLING, BTN_K1_ONCLICK);	
	wiringPiISR(BTN_K2, INT_EDGE_FALLING, BTN_K2_ONCLICK);	
	wiringPiISR(BTN_K3, INT_EDGE_FALLING, BTN_K3_ONCLICK);	
	wiringPiISR(BTN_K4, INT_EDGE_FALLING, BTN_K4_ONCLICK);	
	wiringPiISR(BTN_M1, INT_EDGE_FALLING, BTN_M1_ONCLICK);	
	wiringPiISR(BTN_PWR, INT_EDGE_RISING, BTN_PWR_ONCLICK);	
	printf("start_event_handlers() has completed\n");
}

int main()
{
	start_signals();		//	Handle closing of app in many ways to ensure memory is cleaned
	bbt_initialize();		//	Initialize our devices
	start_event_handlers();
	bbt_run_screen(S_SPLASH);

	getchar();	//	Stop console from exiting until key press occurs
	return 0;	//	End of file
}

