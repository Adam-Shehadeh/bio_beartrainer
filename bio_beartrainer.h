#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <lcd.h>
#include <string.h>
#include <time.h>
#include <errno.h>

// LCD PINS 
#define LCD_RS 	31			//	REAL PIN 28 	BROWN	1/1		Register select pin 	
#define LCD_E 	30			//	REAL PIN 27		PURPLE	1/1		Enable pin				
#define LCD_D4 	5			//	REAL PIN 18		PURPLE	1/4
#define LCD_D5 	4			//	REAL PIN 16		GRAY	2/4
#define LCD_D6 	0			//	REAL PIN 11		WHITE	3/4
#define LCD_D7 	7			//	REAL PIN 7		BLACK	4/4

// PIEZO BEEPER
#define PIB_1	1			// 	REAL PIN 12		GREEN 	1/1

// 10 BUTTON PINS
#define BTN_K4 26			// 	REAL PIN 32		BLUE 	1/4
#define BTN_K3 27			//	REAL PIN 36		GREEN 	2/4
#define BTN_K2 28			//	REAL PIN 38		YELLOW 	3/4
#define BTN_K1 29			//	REAL PIN 40		LT ORNG	4/4		Cycles menu
#define BTN_M1 	21			//	REAL PIN 29		BROWN 	1/1		Disperses treat at any time. Pauses counter while doing so
#define BTN_PWR 9			//	REAL PIN 5		COLOR	1/1		Calls cleanup & shutdown

// LED PINS
#define LED_10 14			//	REAL PIN 23		WHITE 	1/4
#define LED_20 6			//	REAL PIN 22		GRAY	2/4
#define LED_30 10			//	REAL PIN 24		PURPLE	3/4
#define LED_40 11			//	REAL PIN 26		BLUE	4/4

// STEPPER MOTOR CONTROLLER PINS 
// (1 is left, 2 is right)
#define SMC1_IN1 2			//	REAL PIN 13 	BLUE 	1/4
#define SMC1_IN2 3			//	REAL PIN 15 	PURPLE	2/4
#define SMC1_IN3 12			//	REAL PIN 19 	GRAY	3/4
#define SMC1_IN4 13			//	REAL PIN 21 	WHITE	4/4
#define SMC2_IN1 25			//	REAL PIN 37		YELLOW 	1/4
#define SMC2_IN2 24			//	REAL PIN 35		GREEN 	2/4
#define SMC2_IN3 23			//	REAL PIN 33		BLUE 	3/4
#define SMC2_IN4 22			//	REAL PIN 31		PURPLE 	4/4

// STEPPER MOTOR DEGREES TO CYCLES
#define STEP_TURN_45 64
#define STEP_TURN_90 128
#define STEP_TURN_180 256
#define STEP_TURN_360 512

// ENUMS
typedef enum {
	SPLASH, 
	READY_START,
	OPT_MAX_INT,
	OPT_BASE_INT,
	OPT_INCREMENT,
	OPT_INCREMENT_MODE,
	RUNNING,
	DISPENSING
} screen_def;
int SCREEN_ENUM_LENGTH = 8;


typedef enum {
	false = 0,
	true = 1,
} bool;

typedef enum {
	SMC1,
	SMC2,
	BOTH
} motor_turn_select;

typedef enum {			//Used to manage menu state. We have 2 of them which dictate what the buttons do
	MAIN_MENU_MODE,
	TIMER_MENU_MODE
} btn_menu_mode;

typedef enum {
	NONE,			//Increment will not change after each interval
	ADD_ONE,		//Increment will ++ after each interval
	ADD_TWO,		//Increment += 2
	ADD_FIVE,		//Increment += 5
	ADD_SELF_1_4,	//Increment += Increment * 0.25
	ADD_SELF_1_2,	//Increment += Increment * 0.5
	ADD_SELF_3_4,	//Increment += Increment * 0.75
	DBL				//Doubles itself, or Increment += Increment
} increment_mode;
int INC_MODE_ENUM_LENGTH = 8;

// GLOBAL VARIABLES
int lcd;									//The LCD device itseld 
int cur_interval = 10;						//Current step max interval before giving treat and incrementing
int cur_sec = 0;							//Current second we are on 
screen_def cur_screen = 0;					//tells us which screen we are on. 
btn_menu_mode cur_menu_mode = MAIN_MENU_MODE;	//Holds state of which menu we are using, main or timer

// APP SETTINGS THAT USER CAN CHANGE AT RUNTIME
//
int max_interval = 180;		//Maximum we want our interval to go. Will not grow past this
int base_interval = 10;		//Starting value for interval	
int increment = 2;			//Value we will add to our current interval
increment_mode inc_mode = 0;		//bool 1 if we want increment to double itself when interval is reached. 0 increment will stay the same value entire time. 

// BIG PRIVATE FUNCTIONS
// DONT CALL THESE IN IMPLEMENTATION


// Makes sure the physical contactor isn't causing multiple button presses
bool btn_is_pressed(unsigned short int btn)
{
	static struct timespec lastCall;
	struct timespec thisCall;
	
	clock_gettime(CLOCK_REALTIME, &thisCall);
	float timeDiff = (thisCall.tv_sec + thisCall.tv_nsec/1E9 - lastCall.tv_sec - lastCall.tv_nsec/1E9)*5;
	lastCall = thisCall;
	
	return timeDiff > 1 ? true : false;
}


char* get_increment_short_string(increment_mode mode)
{
	char* local[6];
	switch(mode){
		case NONE:;
			*local = "";
			break;
		case ADD_ONE:;
			*local = "ADD1";
			break;
		case ADD_TWO:;
			*local = "ADD2";
			break;
		case ADD_FIVE:;
			*local = "ADD5";
			break;
		case ADD_SELF_1_4:;
			*local = "AS1/4";
			break;
		case ADD_SELF_1_2:;
			*local = "AS1/2";
			break;
		case ADD_SELF_3_4:;
			*local = "AS3/4";
			break;
		case DBL:;
			*local = "DBL";
			break;
	}
	
	return *local;
}


char* get_bbt_interval_string()
{
	// 10+DBL(2)~180
	char* local;
	char* inc_str = get_increment_short_string(inc_mode);
	char i_buff[6] = "";
	sprintf(i_buff, "%d", increment);
	//strcat(local, i_buff);
	//strcat(local, "+");
	//strcat(local, inc_str);
	
	local = get_increment_short_string(inc_mode);
	return local;
}

// BBT FUNCTIONS
// CALL THESE IN IMPLEMENTATION

//	Sets pin state back to start of program
void bbt_init_pins()
{	
	printf("bbt_reset_pins: Setting default pin modes & voltages.\n");
	
	pinMode(LCD_RS, OUTPUT);	// Set LCD pins to output and write 0
	pinMode(LCD_E,  OUTPUT);
	pinMode(LCD_D4, OUTPUT);
	pinMode(LCD_D5, OUTPUT);
	pinMode(LCD_D6, OUTPUT);
	pinMode(LCD_D7, OUTPUT);
	digitalWrite(LCD_RS, LOW);
	digitalWrite(LCD_E, LOW);
	digitalWrite(LCD_D4, LOW);
	digitalWrite(LCD_D5, LOW);
	digitalWrite(LCD_D6, LOW);
	digitalWrite(LCD_D7, LOW);
	
	pinMode(PIB_1, OUTPUT);		// Sets Beeper pin to output
	digitalWrite(PIB_1, LOW);
	
	pinMode(BTN_K1, INPUT);		// Set BTN pins to input
	pinMode(BTN_K2, INPUT);
	pinMode(BTN_K3, INPUT);
	pinMode(BTN_K4, INPUT);
	pinMode(BTN_M1, INPUT);
	pinMode(BTN_PWR, INPUT);
	pullUpDnControl(BTN_K1, PUD_UP);	// Set pull down resistor of buttons to be to 3.3V PUD_UP
	pullUpDnControl(BTN_K2, PUD_UP);
	pullUpDnControl(BTN_K3, PUD_UP);
	pullUpDnControl(BTN_K4, PUD_UP);
	pullUpDnControl(BTN_M1, PUD_UP);
	pullUpDnControl(BTN_PWR, PUD_UP);
	
	pinMode(LED_10, OUTPUT);	// Set LED pins to output and set voltage to 0
	pinMode(LED_20, OUTPUT);
	pinMode(LED_30, OUTPUT);
	pinMode(LED_40, OUTPUT);
	digitalWrite(LED_10, LOW);	
	digitalWrite(LED_20, LOW);
	digitalWrite(LED_30, LOW);
	digitalWrite(LED_40, LOW);
	
	pinMode(SMC1_IN1, OUTPUT);	// Set stepper motor controller pins to output
	pinMode(SMC1_IN2, OUTPUT);
	pinMode(SMC1_IN3, OUTPUT);
	pinMode(SMC1_IN4, OUTPUT);
	pinMode(SMC2_IN1, OUTPUT);	
	pinMode(SMC2_IN2, OUTPUT);
	pinMode(SMC2_IN3, OUTPUT);
	pinMode(SMC2_IN4, OUTPUT);
	digitalWrite(SMC1_IN1, LOW);	// Make sure all pins are turned off for stepepr motor controller.
	digitalWrite(SMC1_IN2, LOW);
	digitalWrite(SMC1_IN3, LOW);
	digitalWrite(SMC1_IN4, LOW);
	digitalWrite(SMC2_IN1, LOW);	
	digitalWrite(SMC2_IN2, LOW);
	digitalWrite(SMC2_IN3, LOW);
	digitalWrite(SMC2_IN4, LOW);
	printf("bbt_init_pins: Function completed.\n");
}

// We call this at the end of program to make sure we didn't fuck shit up
void bbt_cleanup_pins()
{
	lcdClear(lcd);		//clear lcd
	bbt_init_pins();	//resets smc pins 
	
	pullUpDnControl(BTN_K1, PUD_DOWN);	// Set pull down resistor to off position so there's no 3.3V on button
	pullUpDnControl(BTN_K2, PUD_DOWN);
	pullUpDnControl(BTN_K3, PUD_DOWN);
	pullUpDnControl(BTN_K4, PUD_DOWN);
	pullUpDnControl(BTN_M1, PUD_DOWN);
	pullUpDnControl(BTN_PWR, PUD_DOWN);
}

void bbt_shutdown()
{
	printf("bbt_shutdown(): SYSTEM SHUTDOWN HAS BEEN DISPATCHED");
	bbt_cleanup_pins();
	system("sudo shutdown -h now");
}

// Writes to LCD and also to console output 
void bbt_lcd_write(char msg[])
{
	lcdClear(lcd);
	lcdPuts(lcd, msg);
	printf("bbt_lcd_write: Wrote message: '%s'\n", msg);
}

// turns our stepper motor based on params
void bbt_turn_stepper(int turn_cycles, motor_turn_select mts)
{
	int half_step_seq[8][4] = 
	{
		{1,0,0,0},
		{1,1,0,0},
		{0,1,0,0},
		{0,1,1,0},
		{0,0,1,0},
		{0,0,1,1},
		{0,0,0,1},
		{1,0,0,1}	
	};
	
	void run_turn(int pins[4])
	{
		for (int i = 0; i < turn_cycles; i++)			//Loop through full cycle. 1 revolution = 8 cycles * 32 gear reduction = 256 cycles for full turn
		{
			for(int h = 0; h < 8; h++)					//Loop through each full step
			{
				for (int p = 0; p < 4; p++)				//Through each pin state
				{
					digitalWrite(pins[p], half_step_seq[h][p]);
				}
				delay(1);
			}
		}
	}
	
	int smc1_pins[4] = {SMC1_IN1, SMC1_IN2, SMC1_IN3, SMC1_IN4};
	int smc2_pins[4] = {SMC2_IN1, SMC2_IN2, SMC2_IN3, SMC2_IN4};
	
	if (mts == SMC1){
		run_turn(smc1_pins);
	}else if (mts == SMC2){
		run_turn(smc2_pins);
	}else {
		run_turn(smc1_pins);
		run_turn(smc2_pins);
	}
}

void bbt_toggle_led (unsigned short int LED)
{
	digitalWrite(LED, !digitalRead(LED));
}

void bbt_run_screen(screen_def screen_to_run)
{
	cur_screen = screen_to_run;
	char top_str[17] = "";
	char bot_str[17] = "";
	char final_str[34] = "";
	
	
	
	//final_str = "MAX_INTERVAL:   ";
	//sprintf(setting_str, "%d", max_interval);
	//strcat(*final_str, );
	
	bool write_final = true;
	
	switch(cur_screen)
	{
		case SPLASH:;
			bbt_lcd_write("Bio Bearing Bear Trainer v1.0");
			delay(1500);
			bbt_run_screen(READY_START);
			write_final = false;
			break;
		case READY_START:;
			sprintf(top_str, "%s", "HIT K2 TO START:");
			char * modestr = get_increment_short_string(DBL);
			//char* interval_short_str = get_bbt_interval_string();
			sprintf(bot_str, "%s", modestr);
			break;
		case OPT_MAX_INT:;
			sprintf(top_str, "%s", "MAX_INTERVAL:   ");
			sprintf(bot_str, "%d", max_interval);
			break;
		case OPT_BASE_INT:;
			sprintf(top_str, "%s", "BASE_INTERVAL:  ");
			sprintf(bot_str, "%d", base_interval);
			break;
		case OPT_INCREMENT:;
			sprintf(top_str, "%s", "INCREMENT:      ");
			sprintf(bot_str, "%d", increment);
			break;
		case OPT_INCREMENT_MODE:;
			sprintf(top_str, "%s", "INCREMENT_MODE: ");
			sprintf(bot_str, "%d", base_interval);
			break;
		case RUNNING:;
			bbt_lcd_write("RUNNING");
			write_final = false;
			break;
		case DISPENSING:;
			bbt_lcd_write("DISPENSING");
			write_final = false;
			break;
		
	}
	if (write_final){
		strcat(final_str, top_str);
		strcat(final_str, bot_str);
		bbt_lcd_write(final_str);
	}
}

// Called on start of function
void bbt_initialize()	//Resets our application to starting state and initializes important components
{
	printf("bbt_initialize: Function started.\n");
	wiringPiSetup();
	bbt_init_pins();
	lcd = lcdInit (2, 16, 4, LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7, 0, 0, 0, 0);
	lcdClear(lcd);	
	bbt_run_screen(SPLASH);
	printf("bbt_initialize: Function completed.\n");
}

static int led_flash_delay = 50;
void* posix_flash_led(void* led_arg)
{
	int led = *(int*)led_arg;
	printf("posix_flash_led(%d) function started\n", led);
	digitalWrite(led, HIGH);
	delay(led_flash_delay);
	digitalWrite(led, LOW);
	printf("posix_flash_led(%d) function completed\n", led);
	return 0;
}
void bbt_flash_led(unsigned short int led)
{
	printf("bbt_flash_led(%d) function started\n", led);
	pthread_t t1;
	int* ledptr = malloc(sizeof(int));
	*ledptr = led;
	pthread_create(&t1, NULL, &posix_flash_led, ledptr);
	pthread_detach(t1);
	printf("bbt_flash_led(%d) function completed\n", led);
}

//	EVENT HANDLERS & RELATED STUFF
//
// Prevent multiple presses from button due to physical contactors
// https://youtu.be/gymfmJIrc3g?t=431

//	This button will cycle through the menu 
void BTN_K1_ONCLICK()
{
	if (btn_is_pressed(BTN_K1)==1)
	{
		switch(cur_menu_mode)
		{
			case MAIN_MENU_MODE:;
				printf("BTN_K1_ONCLICK() Event handler has been called");
				bbt_flash_led(LED_10);
				if ((int)cur_screen < 5){
					cur_screen++;
				}else{
					cur_screen = 1;
				}
				bbt_run_screen(cur_screen);
				break;
			case TIMER_MENU_MODE:;
				
				break;
		}
	}
}

// This button is the select button for the menu
void BTN_K2_ONCLICK()
{
	if (btn_is_pressed(BTN_K2)==1)
	{
		switch(cur_menu_mode)
		{
			case MAIN_MENU_MODE:;
				printf("BTN_K2_ONCLICK() Event handler has been called");
				bbt_flash_led(LED_20);
				
				break;
			case TIMER_MENU_MODE:;
				
				break;
		}
		
	}
}

void BTN_K3_ONCLICK()
{
	if (btn_is_pressed(BTN_K3)==1)
	{
		switch(cur_menu_mode)
		{
			case MAIN_MENU_MODE:;
				printf("BTN_K3_ONCLICK() Event handler has been called");
				
				bbt_flash_led(LED_30);
				
				if (cur_screen == OPT_MAX_INT){
					max_interval = max_interval > 10 ? max_interval + 10 : 10;
				}else if (cur_screen == OPT_BASE_INT){
					base_interval++;
				}else if (cur_screen == OPT_INCREMENT){
					increment++;
				}else if (cur_screen == OPT_INCREMENT_MODE){
					//dbl_increment = !dbl_increment;
				}
				
				bbt_run_screen(cur_screen);
				break;
			case TIMER_MENU_MODE:;
				
				break;
		}
		
		
	}
}

void BTN_K4_ONCLICK()
{
	if (btn_is_pressed(BTN_K4)==1)
	{
		switch(cur_menu_mode)
		{
			case MAIN_MENU_MODE:;
			
				printf("BTN_K4_ONCLICK() Event handler has been called");
				bbt_lcd_write("Hello from K4");
				bbt_flash_led(LED_40);
				break;
			case TIMER_MENU_MODE:;
				
				break;
		}
		
	}
}

void BTN_M1_ONCLICK()
{
	if (btn_is_pressed(BTN_K2)==1)
	{
		bbt_lcd_write("Hello from M1");
		bbt_flash_led(LED_10);
		bbt_flash_led(LED_20);
		bbt_flash_led(LED_30);
		bbt_flash_led(LED_40);
		printf("BTN_M1_ONCLICK() Event handler has been called");
	}
}

void BTN_PWR_ONCLICK()
{
	if (btn_is_pressed(BTN_K2)==1)
	{
		printf("BTN_PWR_ONCLICK() Event handler has been called"); 
		bbt_lcd_write("SYSTEM SHUT DOWN");
		bbt_cleanup_pins();
		bbt_shutdown();
	}
}


