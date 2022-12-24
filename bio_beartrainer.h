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
#include <softPwm.h>
#include <sys/types.h>

// TYPEDEF ENUMS
typedef enum T_CMD {		//used in bbt_run_timer to tell timer what to do
	T_RUN,					//CMD to run and continue timer
	T_PAUSE,		//Pauses timer variables and thread
	T_RESUME,		//Continues after pause
	T_NEXT,			//CMD to move to next interval
	T_PREV,			//CMD to go back to previous interval
	T_RESET			//Resets our vars and stops timer
} timer_cmds;
typedef enum T_STATE {	// for storing current timer state
	T_RUNNING,
	T_PAUSED,
	T_STOPPED	
} timer_state;
typedef enum S_SCREEN {		// all the screens we can possibly use
	S_SPLASH, 
	S_READY_START,
	S_OPT_MAX_INT,
	S_OPT_BASE_INT,
	S_OPT_INCREMENT,
	S_OPT_INCREMENT_MODE,
	S_OPT_MOTOR_OUTPUTS,
	S_OPT_MOTOR_STEP_TURNS,
	S_OPT_SOUND_ON,
	S_RUNNING,
	S_DISPENSING
} screen_def; int SCREEN_ENUM_LENGTH = 11;	//C is dumb
typedef enum BOOLEAN {	//Point continued
	false = 0,
	true = 1,
} bool;
typedef enum SMC_MODE {
	SMC_MOTOR_1,
	SMC_MOTOR_2,
	SMC_BOTH
} smc_mode; int SMC_MODE_LENGTH = 3;
typedef enum MENU_MODE{			//Used to manage menu state. We have 2 of them which dictate what the buttons do
	MAIN_MENU_MODE,
	TIMER_MENU_MODE
} btn_menu_mode;
typedef enum INC_MODE{
	NONE,			//Increment will not change after each interval
	ADD_ONE,		//Increment will ++ after each interval
	ADD_TWO,		//Increment += 2
	ADD_FIVE,		//Increment += 5
	ADD_SELF_1_4,	//Increment += Increment * 0.25
	ADD_SELF_1_2,	//Increment += Increment * 0.5
	ADD_SELF_3_4,	//Increment += Increment * 0.75
	DBL				//Doubles itself, or Increment += Increment
} increment_mode; int INC_MODE_ENUM_LENGTH = 8;

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
#define STEP_TURN_EIGHTH 64
#define STEP_TURN_QUARTER 128
#define STEP_TURN_HALF 256
#define STEP_TURN_1 512
#define STEP_TURN_1_HALF 768
#define STEP_TURN_2 1024
#define STEP_TURN_3 1536
#define STEP_TURN_5 2560

int all_steps_max = 8;
int all_step_turns[8] = { STEP_TURN_EIGHTH, STEP_TURN_QUARTER, STEP_TURN_HALF, STEP_TURN_1, STEP_TURN_1_HALF, STEP_TURN_2, STEP_TURN_3, STEP_TURN_5};

// OUTPUT SETTINGS
#define WRITE_TO_FILE 		0 
#define WRITE_TO_CONSOLE 	1

// APP SETTINGS
unsigned short int start_interval_threshold = 5;	//Value that we +/- to base interval per each button press
unsigned short int max_interval_threshold = 5;		//Value that we +/- to max interval per each button press
static unsigned int led_flash_delay = 50;			//how fast leds flash
unsigned short int buzz_intensity = 50;				//How hard to buzz bb
unsigned short int buzz_max_range = 100;			//Sets value for scale of how high it can go?
unsigned short int buzz_duration = 500;				//How many milliseconds our buzzer will beep for on beep called

bool smc_direction_clockwise = true;				//controls if motors spin clockwise or counter clockwise

// MAIN VARIABLES AND THEIR DEFAULTS
unsigned short int max_interval = 60;				//Maximum we want our interval to go. Will not grow past this
unsigned short int start_interval = 5;				//Starting value for interval	
unsigned short int increment = 1;					//Value we will add to our current interval
increment_mode inc_mode = NONE;						//Holds state of which increment mode is selected
smc_mode smcm = SMC_MOTOR_2;						//Controls which stepper motors are output to. 1, 2 or both.
unsigned int motor_step_turns = 3;					//This is an index for the all_step_turns array
bool sound_on = false;								//Controls if we use buzz

// LOGICAL DONT TOUCH GLOBAL VARIABLES
int lcd;										//The LCD device itseld 
int cur_interval = 0;							//Current step max interval before giving treat and incrementing
int cur_sec = 0;								//Current second we are on 
screen_def cur_screen = 0;						//tells us which screen we are on. 
btn_menu_mode cur_menu_mode = MAIN_MENU_MODE;	//Holds state of which menu we are using, main or timer
timer_state cur_timer_state = T_STOPPED;		//wtf our timer is doing
char log_file_name[25] = "";
char log_folder_path[] = "/home/bear/Desktop/Programs/bio_beartrainer/logs/";
char out_buff[500];
bool bbt_enable_flash_loop = false;

// BIG PRIVATE FUNCTIONS
// DONT CALL THESE IN IMPLEMENTATION
// NO LOGS WRITEN TO THESE FUNCTIONS
//
bool btn_is_pressed(unsigned short int btn){ // Makes sure the physical contactor isn't causing multiple button presses
	static struct timespec lastCall;
	struct timespec thisCall;
	
	clock_gettime(CLOCK_REALTIME, &thisCall);
	float timeDiff = (thisCall.tv_sec + thisCall.tv_nsec/1E9 - lastCall.tv_sec - lastCall.tv_nsec/1E9)*6;
	lastCall = thisCall;
	
	return timeDiff > 1 ? true : false;
}
void set_log_file_name(char out_str[]){	// Function that initializes the name of our log file
	// Basing on int val of current time
	
	log_file_name[0] = '\0';
	sprintf(log_file_name, "bbt_%d.txt", (int)time(NULL));
}
void set_inc_mode_str(char out_str[], increment_mode mode, bool long_desc){	// Prints short or long desc of increment adder mode enum
	switch(mode)
	{
		case NONE:;
			if (long_desc == true){
				strcpy(out_str, "NONE");
			}else{
				strcpy(out_str, "");
			}
			break;
		case ADD_ONE:;
			if (long_desc == true){
				strcpy(out_str, "+ 1 SECOND");
			}else{
				strcpy(out_str, "ADD1");
			}
			break;
		case ADD_TWO:;
			if (long_desc == true){
				strcpy(out_str, "+ 2 SECONDS");
			}else{
				strcpy(out_str, "ADD2");
			}
			break;
		case ADD_FIVE:;
			if (long_desc == true){
				strcpy(out_str, "+ 5 SECONDS");
			}else{
				strcpy(out_str, "ADD5");
			}
			break;
		case ADD_SELF_1_4:;
			if (long_desc == true){
				strcpy(out_str, "+ (SELF x 0.25)");
			}else{
				strcpy(out_str, "AS25%");
			}
			break;
		case ADD_SELF_1_2:;
			if (long_desc == true){
				strcpy(out_str, "+ (SELF x 0.5)");
			}else{
				strcpy(out_str, "AS50%");
			}
			break;
		case ADD_SELF_3_4:;
			if (long_desc == true){
				strcpy(out_str, "+ (SELF x 0.75)");
			}else{
				strcpy(out_str, "AS75%");
			}
			break;
		case DBL:;
			if (long_desc == true){
				strcpy(out_str, "DOUBLE SELF (x2)");
			}else{
				strcpy(out_str, "DBL");
			}
			break;
	}
}
void set_timer_display_str(char out_str[]){
	// EXAMPLE FORMAT: 10+DBL(2)~180
	char md_str[6] = "";
	set_inc_mode_str(md_str, inc_mode, 0);
	char i_buff[6] = "";
	sprintf(i_buff, "%d", start_interval);
	strcpy(out_str, i_buff);
	strcat(out_str, "+");
	strcat(out_str, md_str);
	strcat(out_str, "(");
	sprintf(i_buff, "%d", increment);
	strcat(out_str, i_buff);
	strcat(out_str, ")~");
	sprintf(i_buff, "%d", max_interval);
	strcat(out_str, i_buff);
}
void set_timer_run_top_str(char out_str[]){
	// EXAMPLE FORMAT:
	// TREAT IN: cur_sec/cur_interval
	
	char buff[16] = "";
	sprintf(buff, "TREAT %d~%d", cur_sec,cur_interval);
	sprintf(out_str, "%-16s", buff);
}
void set_timer_run_bot_str(char out_str[]){
	// EXAMPLE FORMAT: MAX: max_interval NEXT: (next_interval)
	char buff[6] = "";
	set_inc_mode_str(buff, inc_mode, false);
	sprintf(out_str, "MAX %d, %s", max_interval, buff);
}
void set_motor_outputs_str(char out_str[], smc_mode smc_outs){
	switch (smc_outs){
		case SMC_MOTOR_1:;
			strcpy(out_str, "MOTOR 1 [SHORT]");
			break;
		case SMC_MOTOR_2:;
			strcpy(out_str, "MOTOR 2 [LONG]");
			break;
		case SMC_BOTH:;
			strcpy(out_str, "BOTH MOTORS");
			break;
	}
}
void set_motor_step_turns_str(char out_str[], int turns){
	switch (all_step_turns[turns]){
		case STEP_TURN_EIGHTH:;
			strcpy(out_str, "1/8 TURN");
			break;
		case STEP_TURN_QUARTER:;
			strcpy(out_str, "1/4 TURN");
			break;
		case STEP_TURN_HALF:;
			strcpy(out_str, "1/2 TURN");
			break;
		case STEP_TURN_1:;
			strcpy(out_str, "1 TURN");
			break;
		case STEP_TURN_1_HALF:;
			strcpy(out_str, "1.5 TURNS");
			break;
		case STEP_TURN_2:;
			strcpy(out_str, "2 TURNS");
			break;
		case STEP_TURN_3:;
			strcpy(out_str, "3 TURNS");
			break;
		case STEP_TURN_5:;
			strcpy(out_str, "5 TURNS");
			break;
	}
}
int get_next_interval(){
	// function sets output variable to the value of the next increment 
	int local = cur_interval;
    double add = 0;
    
	switch(inc_mode)
	{
		case NONE:;
			break;
		case ADD_ONE:;
			local += 1;
			break;
		case ADD_TWO:;
			local += 2;
			break;
		case ADD_FIVE:;
			local += 5;
			break;
		case ADD_SELF_1_4:;
			add = cur_interval*0.25;
			if (add <1) { add = 1; }
			local += (int)add;
			break;
		case ADD_SELF_1_2:;
			add = cur_interval*0.5;
			if (add <1) { add = 1; }
			local += (int)add;
			break;
		case ADD_SELF_3_4:;
			add = cur_interval*0.75;
			if (add <1) { add = 1; }
			local += (int)add;
			break;
		case DBL:;
			local *= 2;
			break;	
	}
	
	if (local > max_interval)
		local = max_interval;
	
	return local;
} 
int get_prev_interval(){
	unsigned int local = cur_interval;
    double add = 0;
    
	switch(inc_mode)
	{
		case NONE:;
			break;
		case ADD_ONE:;
			local -= 1;
			break;
		case ADD_TWO:;
			local -= 2;
			break;
		case ADD_FIVE:;
			local -= 5;
			break;
		case ADD_SELF_1_4:;
			add = cur_interval*0.25;
			if (add <1) { add = 1; }
			local -= (int)add;
			break;
		case ADD_SELF_1_2:;
			add = cur_interval*0.5;
			if (add <1) { add = 1; }
			local -= (int)add;
			break;
		case ADD_SELF_3_4:;
			add = cur_interval*0.75;
			if (add <1) { add = 1; }
			local -= (int)add;
			break;
		case DBL:;
			local /= 2;
			break;	
	}
	
	if (local < start_interval)
		local = start_interval;
		
	return local;
} 


// BBT FUNCTIONS
// CALL THESE IN IMPLEMENTATION
//
void bbt_log(char msg[]){
	// Output function for logging. This will print to screen and write to file 
	// if specified to do so. Replaces using printf and fprintf separately. Can add more
	
	char fpath[100] = "";
	sprintf(fpath, "%s %s", log_folder_path, log_file_name);
	
	if (WRITE_TO_FILE)
	{
		FILE *fptr = fopen(fpath, "a"); //open with append
		if (fptr == NULL)
		{
			printf("bbt_out: Could not open file %s", fpath);
			return;
		}
		
		fprintf(fptr, "%s\n", msg);
		fclose(fptr);
	}
	
	if (WRITE_TO_CONSOLE)
	{
		strcpy(out_buff, msg);
		strcat(out_buff, "\n");
		printf(out_buff);
	}
}
void bbt_init_pins(){	//	Function that sets pin state back to start of program
	bbt_log("bbt_reset_pins: Setting default pin modes & voltages.");
	
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
	
	bbt_log("bbt_init_pins() Function finished");
}
void bbt_cleanup_pins(){	// We call this function at the end of program to make sure we didn't fuck shit up
	
	bbt_log("bbt_cleanup_pins() function started");
	lcdClear(lcd);		//clear lcd
	bbt_init_pins();	//resets smc pins 
	
	pullUpDnControl(BTN_K1, PUD_DOWN);	// Set pull down resistor to off position so there's no 3.3V on button
	pullUpDnControl(BTN_K2, PUD_DOWN);
	pullUpDnControl(BTN_K3, PUD_DOWN);
	pullUpDnControl(BTN_K4, PUD_DOWN);
	pullUpDnControl(BTN_M1, PUD_DOWN);
	pullUpDnControl(BTN_PWR, PUD_DOWN);
	digitalWrite(LED_10, LOW);
	digitalWrite(LED_20, LOW);
	digitalWrite(LED_30, LOW);
	digitalWrite(LED_40, LOW);
	
	bbt_log("bbt_cleanup_pins() function finished");
}
void bbt_lcd_write(char msg[]){	// Writes to LCD and also to console output 
	lcdClear(lcd);
	lcdPuts(lcd, msg);
	
	strcpy(out_buff, "");
	sprintf(out_buff,"bbt_lcd_write(msg='%s') function finished", msg);
	bbt_log(out_buff);
}
void bbt_toggle_led (unsigned short int LED){
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_toggle_led(LED=%d) function started", LED);
	bbt_log(out_buff);
	
	digitalWrite(LED, !digitalRead(LED));
	
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_toggle_led(LED=%d) function finished", LED);
	bbt_log(out_buff);
}
void bbt_run_screen(screen_def screen_to_run){
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_run_screen(screen_to_run=%d) function started", screen_to_run);
	bbt_log(out_buff);
	
	cur_screen = screen_to_run;
	char top_str[17] = "";
	char bot_str[17] = "";
	char final_str[34] = "";
	bool write_final = true;
	
	switch(cur_screen)
	{
		case S_SPLASH:;
			cur_menu_mode = TIMER_MENU_MODE;	
			bbt_lcd_write("Bio Baring Bear Trainer v0.2");
			delay(2000);
			bbt_run_screen(S_READY_START);
			write_final = false;
			break;
		case S_READY_START:;
			cur_menu_mode = MAIN_MENU_MODE;
			sprintf(top_str, "%s", "HIT K4 TO START:");
			set_timer_display_str(bot_str);
			break;
		case S_OPT_MAX_INT:;
			cur_menu_mode = MAIN_MENU_MODE;
			sprintf(top_str, "%s", "MAX_INTERVAL:   ");
			sprintf(bot_str, "%d", max_interval);
			strcat(bot_str, " SECONDS");
			break;
		case S_OPT_BASE_INT:;
			cur_menu_mode = MAIN_MENU_MODE;
			sprintf(top_str, "%s", "START_INTERVAL: ");
			sprintf(bot_str, "%d", start_interval);
			strcat(bot_str, " SECONDS");
			break;
		case S_OPT_INCREMENT:;
			cur_menu_mode = MAIN_MENU_MODE;
			sprintf(top_str, "%s", "INCREMENT:      ");
			sprintf(bot_str, "%d", increment);
			strcat(bot_str, " SECONDS");
			break;
		case S_OPT_INCREMENT_MODE:;
			cur_menu_mode = MAIN_MENU_MODE;
			sprintf(top_str, "%s", "INC_GROW_MODE:  ");
			set_inc_mode_str(bot_str, inc_mode, 1);
			break;
		case S_OPT_MOTOR_OUTPUTS:;
			cur_menu_mode = MAIN_MENU_MODE;
			sprintf(top_str, "%s", "MOTOR_OUTPUTS:  ");
			set_motor_outputs_str(bot_str, smcm);
			break;
		case S_OPT_MOTOR_STEP_TURNS:;
			cur_menu_mode = MAIN_MENU_MODE;
			sprintf(top_str, "%s", "MOTOR_STEP:     ");
			set_motor_step_turns_str(bot_str, motor_step_turns);
			break;
		case S_OPT_SOUND_ON:;
			cur_menu_mode = MAIN_MENU_MODE;
			sprintf(top_str, "%s", "SOUND_ON:       ");
			if (sound_on == 0)
				sprintf(bot_str, "%s", "FALSE");
			else
				sprintf(bot_str, "%s", "TRUE");
			break;			
		case S_RUNNING:;
			cur_menu_mode = TIMER_MENU_MODE;	
			set_timer_run_top_str(top_str);
			set_timer_run_bot_str(bot_str);
			break;
		case S_DISPENSING:;
			cur_menu_mode = TIMER_MENU_MODE;
			sprintf(top_str, "%s", "DISPENSING:     ");
			set_motor_step_turns_str(bot_str, motor_step_turns);
			break;
	}
	
	if (write_final){
		sprintf(final_str, "%s%s", top_str, bot_str);
		bbt_lcd_write(final_str);
	}
	
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_run_screen(screen_to_run=%d) function finished", screen_to_run);
	bbt_log(out_buff);
}
void bbt_run_stepper_turn(int turn_cycles, int pins[4]) {
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_run_stepper_turn(turn_cycles=%d, pins[4]={%d, %d, %d, %d}) function started", turn_cycles, pins[0], pins[1], pins[2], pins[3]);
	bbt_log(out_buff);
	
	int half_step_counter_clockwise[8][4] = 
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
	
	int half_step_clockwise[8][4] = 
	{
		{1,0,0,1},
		{0,0,0,1},
		{0,0,1,1},
		{0,0,1,0},
		{0,1,1,0},
		{0,1,0,0},
		{1,1,0,0},
		{1,0,0,0}
	};
	
	for (int i = 0; i < turn_cycles; i++)			//Loop through full cycle. 1 revolution = 8 cycles * 32 gear reduction = 256 cycles for full turn
	{
		for(int h = 0; h < 8; h++)					//Loop through each full step
		{
			for (int p = 0; p < 4; p++)				//Through each pin state
			{
				if (smc_direction_clockwise == true)
					digitalWrite(pins[p], half_step_clockwise[h][p]);
				else 
					digitalWrite(pins[p], half_step_counter_clockwise[h][p]);
			}
			delay(1);
		}
	}
	
	//reset pins
	for (int p = 0; p < 4; p++)				//Through each pin state
	{
		digitalWrite(pins[p], LOW);
	}
	
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_run_stepper_turn: turned motor by %d. Function finished", turn_cycles);
	bbt_log(out_buff);
}
void *posix_run_smc1(void* turn_cycles_ptr) {
	int* turn_cycles = (int*)turn_cycles_ptr;
	strcpy(out_buff,"");
	sprintf(out_buff,"posix_run_smc1(void* turn_cycles_ptr=%d) function started", *turn_cycles);
	bbt_log(out_buff);
	
	int smc1_pins[4] = {SMC1_IN1, SMC1_IN2, SMC1_IN3, SMC1_IN4};
	bbt_run_stepper_turn(*turn_cycles, smc1_pins);

	strcpy(out_buff,"");
	sprintf(out_buff, "void *posix_run_smc1(void* turn_cycles_ptr=%d) function completed.)", *turn_cycles);
	bbt_log(out_buff);
	
	return 0;
}
void *posix_run_smc2(void* turn_cycles_ptr) {
	int* turn_cycles = (int*)turn_cycles_ptr;
	strcpy(out_buff,"");
	sprintf(out_buff,"posix_run_smc2(void* turn_cycles_ptr=%d) function started", *turn_cycles);
	bbt_log(out_buff);
	
	int smc2_pins[4] = {SMC2_IN1, SMC2_IN2, SMC2_IN3, SMC2_IN4};
	bbt_run_stepper_turn(*turn_cycles, smc2_pins);
	
	strcpy(out_buff,"");
	sprintf(out_buff,"posix_run_smc2(void* turn_cycles_ptr=%d) function finished", *turn_cycles);
	bbt_log(out_buff);
	
	return 0;
}
void bbt_turn_stepper(int turn_cycles, smc_mode mts, int end_screen) { 	// turns our stepper motor based on params
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_turn_stepper(turn_cycles=%d, mts=%d, end_screen=%d) function started", turn_cycles, mts, end_screen);
	bbt_log(out_buff);
	
	pthread_t threads[2];
	
	if (mts == SMC_MOTOR_1)
	{
		pthread_create(&threads[0], NULL, posix_run_smc1, &turn_cycles);
		pthread_join(threads[0], NULL);
		
	}
	else if (mts == SMC_MOTOR_2)
	{
		pthread_create(&threads[1], NULL, posix_run_smc2, &turn_cycles);
		pthread_join(threads[1], NULL);
	}
	else if (mts == SMC_BOTH){
		pthread_create(&threads[0], NULL, posix_run_smc1, &turn_cycles);
		pthread_create(&threads[1], NULL, posix_run_smc2, &turn_cycles);
		pthread_join(threads[0], NULL);
		pthread_join(threads[1], NULL);
	}
	
	bbt_run_screen(end_screen);
	
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_turn_stepper(turn_cycles=%d, mts=%d, end_screen=%d) function finished", turn_cycles, mts, end_screen);
	bbt_log(out_buff);
}
void bbt_initialize() {	//Resets our application to starting state and initializes important components	
	set_log_file_name(log_file_name);
	strcpy(out_buff, "bbt_initialize() set log_file_name to ");
	strcat(out_buff, log_file_name);
	bbt_log(out_buff);
	
	pid_t pid = getpid();
	strcpy(out_buff, "");
	sprintf(out_buff, "bbt_initialize: Function started with pid: %d", pid);
	bbt_log(out_buff);
	
	putenv("WIRINGPI_CODES=69");	//Enables error return val
	int succ = wiringPiSetup();
	if (succ != 0){
		strcpy(out_buff, "bbt_initialize() FATAL ERROR AFTER CALLING wiringPiSetup(): ");
		strcat(out_buff, strerror(succ));
		exit(1);
	}
	
	bbt_init_pins();
	softPwmCreate(PIB_1, 0, buzz_max_range);
	lcd = lcdInit (2, 16, 4, LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7, 0, 0, 0, 0);
	lcdClear(lcd);	
	bbt_log("bbt_initialize() Function completed.");
}
void* posix_buzz(){
	bbt_log("posix_buzz() function started");
	
	softPwmWrite(PIB_1, buzz_intensity);
	delay(buzz_duration);
	softPwmWrite(PIB_1, 0);
	
	bbt_log("posix_buzz() function finished");
	
	return 0;
}
void bbt_buzz(){
	bbt_log("bbt_buzz function started");
	
	if( sound_on == true){
		pthread_t t1;
		pthread_create(&t1, NULL, &posix_buzz, NULL);
		pthread_detach(t1);
	}else{
		bbt_log("bbt_buzz sound is disabled. buzz skipped");
	}
	bbt_log("bbt_buzz function finished");
}
void* posix_flash_led(void* led_arg){
	int led = *(int*)led_arg;
	strcpy(out_buff,"");
	sprintf(out_buff,"posix_flash_led(led_arg=%d) function started", led);
	bbt_log(out_buff);
	
	digitalWrite(led, HIGH);
	delay(led_flash_delay);
	digitalWrite(led, LOW);
	
	strcpy(out_buff,"");
	sprintf(out_buff,"posix_flash_led(led_arg=%d) function completed", led);
	bbt_log(out_buff);
	
	return 0;
}
void bbt_flash_led(unsigned short int led){
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_flash_led(led=%d) function started", led);
	bbt_log(out_buff);
	
	pthread_t t1;
	int* ledptr = malloc(sizeof(int));
	*ledptr = led;
	pthread_create(&t1, NULL, &posix_flash_led, ledptr);
	pthread_detach(t1);
	
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_flash_led(led=%d) function finished", led);
	bbt_log(out_buff);
}
void* posix_flash_led_loop(void* led_arg){
	int led = *(int*)led_arg;
	strcpy(out_buff,"");
	sprintf(out_buff,"posix_flash_led_loop(led_arg=%d) function started", led);
	bbt_log(out_buff);
	int count = 0;
	while (count < 10){
		digitalWrite(led, HIGH);
		delay(led_flash_delay);
		digitalWrite(led, LOW);
		delay(1000-led_flash_delay);
		count++;
	}
	
	strcpy(out_buff,"");
	sprintf(out_buff,"posix_flash_led_loop(led_arg=%d) function completed", led);
	bbt_log(out_buff);
	
	return 0;
}
void bbt_flash_led_loop(unsigned short int led){
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_flash_led_loop(led=%d) function started", led);
	bbt_log(out_buff);
	
	bbt_enable_flash_loop = true;
	pthread_t t1;
	int* ledptr = malloc(sizeof(int));
	*ledptr = led;
	pthread_create(&t1, NULL, &posix_flash_led_loop, ledptr);
	pthread_detach(t1);
	
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_flash_led_loop(led=%d) function finished", led);
	bbt_log(out_buff);
}
void bbt_dispense_treat(){
	bbt_log("bbt_dispense_treat() function started");
	
	int prev_scr = cur_screen;
	bbt_run_screen(S_DISPENSING);		//Run screen
	bbt_flash_led(LED_10);			//Flash all LEDs because wow
	bbt_flash_led(LED_20);
	bbt_flash_led(LED_30);
	//bbt_flash_led(LED_40);
	delay(150);
	bbt_buzz();
	bbt_turn_stepper(all_step_turns[motor_step_turns], smcm, prev_scr);
	bbt_run_screen(prev_scr);
	
	bbt_log("bbt_dispense_treat() function started");
}
void* posix_run_timer(void* t_arg){	//t_arg is pointer to int val for enum timer_cmd
	int t_cmd = *(int*)t_arg;
	strcpy(out_buff,"");
	sprintf(out_buff,"posix_run_timer(t_arg=%d) function started", t_cmd);
	bbt_log(out_buff);

	cur_timer_state = T_RUNNING;

	//perform timer main loop
	while(cur_timer_state == T_RUNNING)
	{
		bbt_run_screen(S_RUNNING);
		bbt_flash_led(LED_10);
		delay(1000);
		if (cur_sec >= cur_interval-1)
		{
			bbt_dispense_treat();
			cur_sec = 0;
			cur_interval = get_next_interval();
			if (cur_interval > max_interval){
				cur_interval = max_interval;	
			}
		}
		else
		{
			cur_sec++;
		}
	}
	
	strcpy(out_buff,"");
	sprintf(out_buff,"posix_run_timer(t_arg=%d) function finished", t_cmd);
	bbt_log(out_buff);
	
	return 0;
}
void bbt_run_timer(unsigned short int t_cmd){ // Invokes the timer thread based on timer_cmd enum value sent
	
	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_run_timer(t_cmd=%d) function started", t_cmd);
	bbt_log(out_buff);

	switch (t_cmd)
	{
		case T_RUN:;
			
			pthread_t t1;
			int* t_cmdptr = malloc(sizeof(int));
			*t_cmdptr = t_cmd;
			pthread_create(&t1, NULL, &posix_run_timer, t_cmdptr);
			pthread_detach(t1);
			free(t_cmdptr);
			break;
		case T_PAUSE:;
			cur_timer_state = T_PAUSED;
			break;
		case T_RESUME:;
			cur_timer_state = T_RUNNING;
			break;
		case T_NEXT:;
			cur_sec = 0;
			cur_interval = get_next_interval();
			break;
		case T_PREV:;
			cur_sec = 0;
			cur_interval = get_prev_interval();
			break;
		case T_RESET:;
			cur_timer_state = T_STOPPED;
			cur_sec = 0;
			cur_interval = start_interval;
			break;
	}

	strcpy(out_buff,"");
	sprintf(out_buff,"bbt_run_timer(t_cmd=%d) function finished", t_cmd);
	bbt_log(out_buff);
}
void bbt_shutdown(){
	bbt_buzz();
	bbt_log("bbt_shutdown(): SYSTEM SHUTDOWN HAS BEEN DISPATCHED");
	bbt_cleanup_pins();
	system("sudo shutdown -h now");
	kill(getpid(),SIGINT);
}
//	EVENT HANDLERS & RELATED STUFF
//
// Prevent multiple presses from button due to physical contactors
// https://youtu.be/gymfmJIrc3g?t=431
//
void BTN_K1_ONCLICK(){	//	This button will cycle through the menu 
	
	if (btn_is_pressed(BTN_K1)==1)
	{
		
		bbt_log("BTN_K1_ONCLICK() Event handler has been called");
		switch(cur_menu_mode)
		{
			case MAIN_MENU_MODE:;
				bbt_flash_led(LED_10);
				if ((int)cur_screen < SCREEN_ENUM_LENGTH - 3){
					cur_screen++;
				}else{
					cur_screen = 1;
				}
				bbt_run_screen(cur_screen);
				break;
			case TIMER_MENU_MODE:;
				if (cur_timer_state == T_RUNNING || cur_sec > 0)	//Stop timer if running
				{
					bbt_run_timer(T_RESET);	
					bbt_run_screen(cur_screen);
				}else{
					bbt_run_screen(S_READY_START);
				}
				break;
		}
	}
}
void BTN_K2_ONCLICK(){ 	// This button is the select button for the menu
	if (btn_is_pressed(BTN_K2)==1)
	{
		bbt_log("BTN_K2_ONCLICK() Event handler has been called");
		switch(cur_menu_mode)
		{
			case MAIN_MENU_MODE:;
				bbt_flash_led(LED_20);			//Flash led
				if ((int)cur_screen > 1){
					cur_screen--;
				}else{
					cur_screen = SCREEN_ENUM_LENGTH - 3;
				}
				bbt_run_screen(cur_screen);
				break;
			case TIMER_MENU_MODE:;
				if (cur_timer_state == T_RUNNING){
					bbt_run_timer(T_PAUSE);
				}
				else {
					bbt_run_timer(T_RUN);
				}
				break;
		}
		
	}
}
void BTN_K3_ONCLICK(){
	if (btn_is_pressed(BTN_K3)==1)
	{
		switch(cur_menu_mode)
		{
			case MAIN_MENU_MODE:;
				bbt_log("BTN_K3_ONCLICK() Event handler has been called");
				bbt_flash_led(LED_30);
				if (cur_screen == S_OPT_MAX_INT)
				{
					max_interval = max_interval >= max_interval_threshold ? max_interval + max_interval_threshold : max_interval_threshold;
				}
				else if (cur_screen == S_OPT_BASE_INT)
				{
					start_interval = start_interval >= start_interval_threshold ? start_interval + start_interval_threshold : start_interval_threshold;
				}
				else if (cur_screen == S_OPT_INCREMENT)
				{
					increment++;
				}
				else if (cur_screen == S_OPT_INCREMENT_MODE){
					inc_mode = inc_mode < INC_MODE_ENUM_LENGTH-1 ? inc_mode+1 : 0;
				}
				else if (cur_screen == S_OPT_MOTOR_OUTPUTS){
					smcm = smcm < SMC_MODE_LENGTH-1 ? smcm+1 : 0;
				}else if (cur_screen == S_OPT_MOTOR_STEP_TURNS){
					motor_step_turns = motor_step_turns < all_steps_max-1 ? motor_step_turns+1 : 0;
				}else if (cur_screen == S_OPT_SOUND_ON){
					sound_on = !sound_on;
				}
				bbt_run_screen(cur_screen);
				break;
			case TIMER_MENU_MODE:;
				bbt_run_timer(T_NEXT);
				if (cur_timer_state != T_RUNNING){
					bbt_run_timer(T_RUN);
				}
				break;
		}
		
		
	}
}
void BTN_K4_ONCLICK(){
	if (btn_is_pressed(BTN_K4)==1)
	{
		switch(cur_menu_mode)
		{
			case MAIN_MENU_MODE:;
				bbt_log("BTN_K4_ONCLICK() Event handler has been called");
				bbt_flash_led(LED_40);
				
				if (cur_screen == S_READY_START){
					bbt_run_screen(S_RUNNING);		//render screen
					bbt_run_timer(T_RESET);			//Make sure settings are reset
					bbt_run_timer(T_RUN);			//Call timer thread with command to start
				}else{
					if (cur_screen == S_OPT_MAX_INT)
					{
						max_interval = max_interval > max_interval_threshold ? max_interval - max_interval_threshold : max_interval_threshold;
					}
					else if (cur_screen == S_OPT_BASE_INT)
					{
						start_interval = start_interval > start_interval_threshold ? start_interval - start_interval_threshold : 2;
					}
					else if (cur_screen == S_OPT_INCREMENT)
					{
						increment = increment > 0 ? increment - 1 : 0;
					}
					else if (cur_screen == S_OPT_INCREMENT_MODE)
					{
						inc_mode = inc_mode > 0 ? inc_mode-1 : INC_MODE_ENUM_LENGTH-1;
					}
					else if (cur_screen == S_OPT_MOTOR_OUTPUTS){
						smcm = smcm > 0 ? smcm-1 : SMC_MODE_LENGTH-1;
					}else if (cur_screen == S_OPT_MOTOR_STEP_TURNS){
						motor_step_turns = motor_step_turns > 0 ? motor_step_turns-1 : all_steps_max-1;
					}else if (cur_screen == S_OPT_SOUND_ON){
						sound_on = !sound_on;
					}
					bbt_run_screen(cur_screen);
				}
				break;
			case TIMER_MENU_MODE:;
				bbt_run_timer(T_PREV);
				if (cur_timer_state != T_RUNNING){
					bbt_run_timer(T_RUN);
				}
				break;
		}
		
	}
}
void BTN_M1_ONCLICK(){
	if (btn_is_pressed(BTN_M1)==1)
	{
		bbt_log("BTN_M1_ONCLICK() Event handler has been called");
		bbt_dispense_treat();
	}
}
void BTN_PWR_ONCLICK(){
	if (btn_is_pressed(BTN_K2)==1)
	{
		bbt_log("BTN_PWR_ONCLICK() Event handler has been called"); 
		bbt_lcd_write("SYSTEM SHUT DOWN");
		delay(3000);
		bbt_cleanup_pins();
		bbt_lcd_write("");
		//kill(getpid(),SIGINT);
		bbt_shutdown();
	}
}


