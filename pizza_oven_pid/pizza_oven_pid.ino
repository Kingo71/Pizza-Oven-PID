/*
Name:		PizzaOven.ino
Created:	7/14/2018 6:33:24 PM
Author:	Kingo
*/

#include <EEPROMex.h>
#include <EEPROMVar.h>

#include <toneAC.h>

#include <Time.h>

#include <TimeLib.h>

#include <Button.h>

#include <PID_AutoTune_v0.h>

#include <PID_v1.h>

#include <max6675.h>

#include <Wire.h>

#include <LiquidCrystal_I2C.h>

// Menu items

#define MENU_ITEMS 4

#define MENU_TARGET 0

#define MENU_PID 1

#define MENU_CLEANING 2

#define MENU_TIMER1 3

#define MENU_TIMER2 4

#define MENU_Kp 0

#define MENU_Ki 1

#define MENU_Kd 2

#define MENU_EXIT 3

#define MENU_SAVE 4


// Variables declaration

// PID and Autotune
double Output, tmcval, tmcread, targettemp; // PID Parameters
double Kp, Ki, Kd, tKp, tKi, tKd; // PID Tune variables
double aTuneStep = 500, aTuneNoise = 1;
unsigned int aTuneLookBack = 20;
byte ATuneModeRemember = 2;
int long WindowSize = 10000;
bool tuning = false;

// Temperature related variables
int safetemp = 480; // Safe temperature
int cleaningTime = 1500; // Cleaning duration


int menu = -1;
int menupid = -1;
int tmc1;
#define rwlw 0
#define rwhg 146
#define reflw 8
#define refhg 101
int ledstat1 = LOW;
int long interval = 1000;

// Timeout variables 
bool client = false;
bool cleaning = false;


bool timer1Status = false;
bool timerRinging = false;
bool timer2Ringing = false;
bool timer2Status = false;

// Buttons flags
bool button1release = false;
bool button1hold = false;
bool button2hold = false;
bool button3hold = false;

bool blink = false;
bool power = false;
bool warmingup = false;
bool alarm = false;
bool alarmstarted = false;



unsigned long clientlastmillis;
unsigned long lastmillis = 0;
unsigned long windowStartTime;


// Time variables

time_t currentTime1 = 0;
time_t currentTime2 = 0;
time_t startTime1 = 0;
time_t startTime2 = 0;
time_t setupTime1;
time_t setupTime2;
time_t upTime = 0;
time_t safetime = 3600;
time_t warmupstart = 0;
time_t warmup = 40;

// EEPROM addresses for persisted data
const int SpAddress = EEPROM.getAddress(sizeof(targettemp));
const int KpAddress = EEPROM.getAddress(sizeof(Kp));
const int KiAddress = EEPROM.getAddress(sizeof(Ki));
const int KdAddress = EEPROM.getAddress(sizeof(Kd));
const int T1Address = EEPROM.getAddress(sizeof(setupTime1));
const int T2Address = EEPROM.getAddress(sizeof(setupTime2));


// PINS

#define thermoDO 6   // Thermocouple
#define thermoCS 7   // Thermocouple
#define thermoCLK 13 // Thermocouple
#define UpHeaterPin 11   // Upper heater Relay pin
#define DownHeaterPin 12 // Down heater Relay pin
#define btn1 2       // Button 1
#define btn2 4       // Button 2
#define btn3 8       // Button 3

// Buttons

Button button1 = Button(btn1, BUTTON_PULLDOWN);
Button button2 = Button(btn2, BUTTON_PULLDOWN);
Button button3 = Button(btn3, BUTTON_PULLDOWN);

// PID 

PID myPID(&tmcval, &Output, &targettemp, Kp, Ki, Kd, DIRECT);

PID_ATune aTune(&tmcval, &Output);

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
	EEPROM.updateInt(T1Address, setupTime1);
	EEPROM.updateInt(T2Address, setupTime2);

	EEPROM.updateDouble(SpAddress, targettemp);
	EEPROM.updateDouble(KpAddress, Kp);
	EEPROM.updateDouble(KiAddress, Ki);
	EEPROM.updateDouble(KdAddress, Kd);

}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
	// Load from EEPROM
	setupTime1 = EEPROM.readInt(T1Address);
	setupTime2 = EEPROM.readInt(T2Address);
	targettemp = EEPROM.readDouble(SpAddress);
	Kp = EEPROM.readDouble(KpAddress);
	Ki = EEPROM.readDouble(KiAddress);
	Kd = EEPROM.readDouble(KdAddress);

	// Use defaults if EEPROM values are invalid

	if (isnan(setupTime1) || setupTime1 > 3540)
	{
		setupTime1 = 120;
	}

	if (isnan(setupTime2) || setupTime2 > 3540)
	{
		setupTime2 = 120;
	}

	if (isnan(targettemp))
	{
		targettemp = 400;
	}
	if (isnan(Kp))
	{
		Kp = 500;
	}
	if (isnan(Ki))
	{
		Ki = 0.5;
	}
	if (isnan(Kd))
	{
		Kd = 0.1;
	}
}

// Buttons hold 
void onHold(Button& b)
{
	toneAC(450, 10, 50, true);

	if (b.pin == btn1) {
		button1hold = true;

		switch (menu)
		{
		case MENU_TARGET:
		{
			power = !power;
			if (power) {
				upTime = now();

			}
			else break;
		}
		case MENU_PID:
		{
			if (power && menupid == -1) if (tuning || (abs(tmc1 - targettemp) < 5 && !tuning)) {
				changeAutoTune();
				menuPRINT(menu);
				return;
			}

			if (menupid == 3 || menupid == 4) {
				Kp = tKp;
				Ki = tKi;
				Kd = tKd;
				if (menupid == 4) SaveParameters();
				myPID.SetTunings(Kp, Ki, Kd);
				if (myPID.GetMode() == 1) {
					myPID.SetMode(MANUAL);
					myPID.SetMode(AUTOMATIC);
				}

			}
			menupid = -1;
			return;
		}
		case MENU_CLEANING:
		{
			if (power) cleaning = !cleaning;
			if (cleaning) {
				upTime = now();
				targettemp = 380;
			}
			else LoadParameters();
			return;
		}

		case MENU_TIMER1:
		{
			if (power) {
				if (currentTime1 && !timer1Status) {
					currentTime1 = 0;
					menuPRINT(MENU_TIMER1);
					return;
				}

				timer1Status = !timer1Status;
				if (!timer1Status) {
					startTime1 = 0;
					currentTime1 = 0;
					warmingup = false;
				}
				else {
					startTime1 = now();
					warmupstart = now();
					return;
				}
			}
			return;
		}


		case MENU_TIMER2:

			if (power) {
				timer2Status = false;
				if (!timer2Status) {
					startTime2 = 0;
					currentTime2 = 0;
					menu = MENU_TIMER1;
				}


			}

			break;


		}
	}
	if (b.pin == btn2) button2hold = true;

	if (b.pin == btn3) button3hold = true;

}

void onRelease(Button& b) {
	// if (b.pin == btn1) button1hold = false;
	if (b.pin == btn1) {
		if (button1hold) button1release = true;
		button1hold = false;
	}

	if (b.pin == btn2 || b.pin == btn3)
	{
		button2hold = false;
		button3hold = false;
	}
}


// set the LCD address to 0x3f for a 16 chars 2 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol

LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

int refrange = refhg - reflw;
int rwrange = rwhg - rwlw;


// make a cute degree symbol

uint8_t degree[8] = { 140, 146, 146, 140, 128, 128, 128, 128 };

void setup()
{

	pinMode(UpHeaterPin, OUTPUT);
	pinMode(DownHeaterPin, OUTPUT);
	digitalWrite(UpHeaterPin, LOW); // power off on start
	digitalWrite(DownHeaterPin, LOW); // power off on start

									  // Start up the library

									  // HW Serial startup

	Serial.begin(9600);

	// PID Library setup

	LoadParameters(); // load PID and timers parameters

	myPID.SetTunings(Kp, Ki, Kd);
	myPID.SetOutputLimits(0, WindowSize);


	// Imposta il valore di righe e colonne del display LCD

	lcd.begin(16, 2);
	lcd.createChar(0, degree);

	button1.holdHandler(onHold, 1000); // must be held for at least 1000 ms to trigger
	button2.holdHandler(onHold, 1000); // must be held for at least 1000 ms to trigger
	button3.holdHandler(onHold, 1000); // must be held for at least 1000 ms to trigger
	button1.releaseHandler(onRelease);
	button2.releaseHandler(onRelease);
	button3.releaseHandler(onRelease);

	blinking(2, 1);

	windowStartTime = millis();

	// Run timer2 interrupt every 15 ms 
	TCCR2A = 0;
	TCCR2B = 1 << CS22 | 1 << CS21 | 1 << CS20;

	//Timer2 Overflow Interrupt Enable
	TIMSK2 |= 1 << TOIE2;


}

// ************************************************
// Timer Interrupt Handler
// ************************************************

SIGNAL(TIMER2_OVF_vect)
{
	if (!power)
	{
		digitalWrite(UpHeaterPin, LOW);  // make sure relay is off
		digitalWrite(DownHeaterPin, LOW);
		if (myPID.GetMode() == 1) myPID.SetMode(MANUAL);
	}
	else
	{
		if (timer1Status || timerRinging || timer2Status || warmingup) {
			digitalWrite(UpHeaterPin, HIGH); // Upper heater ON during baking stage
			digitalWrite(DownHeaterPin, LOW); // Bottom heater off during baking stage
			if (myPID.GetMode() == 1) myPID.SetMode(MANUAL);
			Output = WindowSize;
			upTime = now();
		}
		if (myPID.GetMode() == 1) DriveOutput();
	}

}

void loop()

{


	if ((now() - upTime) >= safetime && power) {
		power = false;
		blinking(3, 1);
		cleaning = false;
	}

	if (cleaning) if (cleaningTime - (now() - upTime) <= 0) {
		cleaning = false;
		power = false;
		LoadParameters();
		blinking(5, 1);
	}


	if (timer1Status && (now() - warmupstart < warmup)) warmingup = true;


	if (power && (timer1Status || timerRinging && !timer2Ringing)) {
		if (!warmingup) {
			if (!setupTime1) currentTime1 = (now() - startTime1);
			else currentTime1 = setupTime1 - (now() - startTime1);

			if (currentTime1 <= 0 && setupTime1) {
				timerRinging = true;
				timer1Status = false;
				menu = MENU_TIMER2;
				startTime1 = 0;
				currentTime1 = 0;

			}

		}
		else {
			startTime1 = now();
			currentTime1 = warmup - (now() - warmupstart);
			if ((now() - warmupstart) > warmup) currentTime1 = setupTime1;

		}
	}
	else if (power && !timer2Status) {
		if (myPID.GetMode() == 0) myPID.SetMode(AUTOMATIC);
		runPID();
	}

	if (power && timer2Status && !timer2Ringing)
	{
		if (!setupTime2) currentTime2 = (now() - startTime2);
		else currentTime2 = setupTime2 - (now() - startTime2);
		if (currentTime2 <= 0 && setupTime2) {
			timerRinging = true;
			timer2Ringing = true;
			timer2Status = false;
			currentTime1 = 0;
			menu = MENU_TIMER1;
		}
	}

	if ((millis() - lastmillis) >= interval)
	{
		tmcread = thermocouple.readCelsius();

		tmcval = (((tmcread - rwlw) * refrange) / rwrange) + reflw;
		tmc1 = int(tmcval);



		lcd.setCursor(7, 0);

		if (power || ledstat1) {
			if (blink) lcd.print(" <");
			else lcd.print("<<");
			if (tmc1 < 100) lcd.print(" ");
			if (tmc1 < 10) lcd.print(" ");

			if (tmc1 < 600) {
				lcd.print(tmc1);
				lcd.write(byte(0));
				lcd.print("C");
			}
			else lcd.print("Error");
			if (blink) lcd.print("> ");
			else lcd.print(">> ");
		}
		else lcd.print("<<-OFF->>");


		if (tmc1 < targettemp) upTime = now();

		if (tmc1 > safetemp) power = false;

		if ((timerRinging) || (timerRinging)) alarm = true;
		else alarm = false;

		ledstat1 = !ledstat1;

		lastmillis = millis();
		if (timer1Status || timer2Status) blink = !blink;
		else blink = false;
	}

	if (alarm && ledstat1 == HIGH) if (!alarmstarted) {
		blinking(4, 1); // Alarm
		alarmstarted = true;
	}

	buttons(menu);
	menuPRINT(menu);

}


// Functions --------------------------------

//****************************************************
//*      PID Output                                  *
//****************************************************

void DriveOutput()

{
	unsigned long nowmillis = millis();
	if (nowmillis - windowStartTime > WindowSize) { //time to shift the Relay Window
		windowStartTime += WindowSize;
	}
	if ((Output > 200) && (Output > nowmillis - windowStartTime)) {
		digitalWrite(UpHeaterPin, HIGH);
		if (((tmcval * 100) / targettemp < 95) || cleaning) digitalWrite(DownHeaterPin, HIGH);

	}
	else {
		digitalWrite(UpHeaterPin, LOW);
		digitalWrite(DownHeaterPin, LOW);
	}
}

//****************************************************
//*      Backlight blinking                          *
//****************************************************


void blinking(int times, bool sound)

{

	for (int i = 0; i < times; i++)
	{
		lcd.backlight();
		if (sound) toneAC(262, 10, 250);
		lcd.noBacklight();
		if (sound) noToneAC();
		delay(250);

	}
	lcd.backlight(); // finish with backlight on
	noToneAC();

}

//****************************************************
//*      Buttons input                               *
//****************************************************


bool buttons(int menusel)
{
	if (button1.stateChanged() && !button1.isPressed())
	{
		if (menupid == -1) {
			if (button1release == false && button1hold == false && !warmingup && !cleaning)
			{
				++menu;
				if (menu > MENU_ITEMS) menu = 0;
				ledstat1 = false;
				return true;
			}
			else button1release = false;
		}
		else {
			++menupid;
			if (menupid > 4) menupid = 0;
			return true;
		}
	}
	if (button2.stateChanged() && !button2.isPressed())
	{

		switch (menusel)
		{
		case MENU_TARGET:
		{
			if (!tuning) --targettemp;
			if (targettemp < 0) targettemp = 0;
			return true;
		}

		case MENU_PID:
		{
			if (menupid == -1) return true;

			switch (menupid)
			{
			case MENU_Kp:
			{
				tKp = tKp - .01;
				if (tKp <0) tKp = 0;
				return true;
			}
			case MENU_Ki:
			{
				tKi = tKi - 0.01;
				if (tKi <0) tKi = 0;
				return true;
			}
			case MENU_Kd:
			{
				tKd = tKd - 0.01;
				if (tKd < 0.1) tKd = 0;
				return true;
			}
			return true;
			}
			return true;
		}

		case MENU_CLEANING:
		{

			return true;
		}

		case MENU_TIMER1:
		{
			if (timer1Status) break;
			if (setupTime1 >= 15) setupTime1 = setupTime1 - 15;
			else {
				setupTime1 = 0;
				currentTime1 = 0;
				return true;
			}
			return true;
		}

		case MENU_TIMER2:
		{
			if (timer2Status) break;
			if (setupTime2 >= 15) setupTime2 = setupTime2 - 15;

			if (setupTime2 <= 15) {
				setupTime2 = 0;
				currentTime2 = 0;
				return true;
			}

			return true;
		}
		}
	}
	else if (button2.isPressed() && button2hold)
	{
		switch (menusel)
		{
		case MENU_TARGET:
		{
			if (!tuning) targettemp = targettemp - .5;
			if (targettemp < 0) targettemp = 0;
			return true;
		}

		case MENU_PID:
		{
			if (menupid == -1) return true;

			switch (menupid)
			{
			case MENU_Kp:
			{
				tKp = tKp - .1;
				if (tKp <0) tKp = 0;
				return true;
			}
			case MENU_Ki:
			{
				tKi = tKi - 0.01;
				if (tKi <0) tKi = 0;
				return true;
			}
			case MENU_Kd:
			{
				tKd = tKd - 0.01;
				if (tKd < 0.1) tKd = 0;
				return true;
			}
			return true;
			}
			return true;
		}

		case MENU_CLEANING:
		{

			return true;
		}

		case MENU_TIMER1:
		{
			if (timer1Status) break;
			if (setupTime1 >= 15) setupTime1 = setupTime1 - 15;


			if (setupTime1 <= 0) {
				setupTime1 = 0;
				currentTime1 = 0;
				return true;
			}
			return true;
		}

		case MENU_TIMER2:
		{
			if (timer2Status) break;
			if (setupTime2 >= 15) setupTime2 = setupTime2 - 15;

			if (setupTime2 <= 0) {
				setupTime2 = 0;
				currentTime2 = 0;
				return true;
			}

			return true;
		}
		}
	}

	if (button3.stateChanged() && !button3.isPressed())
	{
		if (timerRinging) {
			ledstat1 = false;
			timerRinging = false;
			alarmstarted = false;
			if (!timer2Ringing) {
				timer2Status = true;
				startTime2 = now();
				menu = MENU_TIMER2;
				return true;
			}
			timer2Ringing = false;
			menu = MENU_TIMER1;
			return true;
		}

		if ((now() - warmupstart >= warmup) && warmingup) {
			warmingup = false;
			ledstat1 = false;
			menu = MENU_TIMER1;
			return true;
		}
		switch (menusel)
		{
		case MENU_TARGET:
		{
			if (!tuning) ++targettemp;
			return true;
		}
		case MENU_PID:
		{
			if (menupid == -1) {
				menupid = 0;
				tKp = Kp;
				tKi = Ki;
				tKd = Kd;
				return true;
			}
			switch (menupid)
			{
			case MENU_Kp:
			{
				tKp = tKp + .01;
				if (tKp > 1000) tKp = 1000;
				return true;
			}
			case MENU_Ki:
			{
				tKi = tKi + 0.01;
				if (tKi > 1000) tKi = 1000;
				return true;
			}
			case MENU_Kd:
			{
				tKd = tKd + 0.01;
				if (tKd > 1000) tKd = 1000;
				return true;
			}
			}
			return true;
		}

		case MENU_CLEANING:
		{
			return true;
		}

		case MENU_TIMER1:
		{
			if (timer1Status) break;

			setupTime1 = setupTime1 + 15;
			if (setupTime1 >3540) setupTime1 = 3540;
			return true;
		}

		case MENU_TIMER2:
		{
			if (timer2Status) break;
			setupTime2 = setupTime2 + 15;
			if (setupTime2 >3540) setupTime2 = 3540;
			return true;
		}

		}
	}
	else if (button3.isPressed() && button3hold)
	{
		switch (menusel)
		{
		case MENU_TARGET:
		{
			if (!tuning) targettemp = targettemp + .5;
			return true;
		}

		case MENU_PID:
		{

			switch (menupid)
			{
			case MENU_Kp:
			{
				tKp = tKp + .1;
				if (tKp > 1000) tKp = 1000;
				return true;
			}
			case MENU_Ki:
			{
				tKi = tKi + 0.01;
				if (tKi > 1000) tKi = 1000;
				return true;
			}
			case MENU_Kd:
			{
				tKd = tKd + 0.01;
				if (tKd > 1000) tKd = 1000;
				return true;
			}
			}
			return true;
		}

		case MENU_CLEANING:
		{
			return true;
		}

		case MENU_TIMER1:
		{
			if (timer1Status) break;

			setupTime1 = setupTime1 + 30;
			if (setupTime1 >3540) setupTime1 = 3540;
			return true;
		}

		case MENU_TIMER2:
		{
			if (timer2Status) break;
			if (timerRinging) {
				timerRinging = false;
				break;
			}
			setupTime2 = setupTime2 + 30;
			if (setupTime2 >3540) setupTime2 = 3540;
			return true;
		}

		}

	}

	return false;
}

//****************************************************
//*         Menu displaying                          *
//****************************************************


void menuPRINT(int menusel)


{
	lcd.setCursor(0, 0); // bottom left
	switch (menusel)
	{
	case MENU_TARGET:
	{
		lcd.print("Target|");
		break;
	}
	case MENU_PID:
	{
		lcd.print("PID");
		if (myPID.GetMode() == 0) lcd.print(" --|");
		else lcd.print(" on|");
		break;
	}
	case MENU_CLEANING:
	{
		lcd.print("Clean |");
		break;
	}
	case MENU_TIMER1:
	{
		lcd.print("Timer |");
		break;
	}
	case MENU_TIMER2:
	{
		lcd.print("Timer |");
		break;
	}
	}

	lcd.setCursor(0, 1);

	// blinking Messages 

	if (ledstat1) {
		if ((now() - warmupstart >= warmup) && timer1Status && warmingup) {
			lcd.print(" -  INFORNARE  -");
			return;
		}

		if (timer1Status && warmingup) {
			lcd.print("WARMING UP");

		}

		if (timerRinging && !timer2Ringing) {
			lcd.print("-RUOTA LA PIZZA-");
			return;
		}

		if (timer2Ringing) {
			lcd.print("-PIZZA  PRONTA!-");
			return;
		}

		if (cleaning) {
			lcd.print("--- CLEANING ---");
			return;
		}


	}



	switch (menusel)
	{
	case MENU_TARGET: // Home menu
	{
		lcd.print("Target Tmp.:");
		lcd.print(int(targettemp));
		lcd.write(byte(0));
		lcd.print(" ");
		break;
	}

	case MENU_PID: // PID menu
	{
		if (menupid == -1) {
			lcd.print("Atune:");
			if (tuning) lcd.print("On");
			else lcd.print("Off");
			lcd.print("  :");
			lcd.print(int(((Output) * 100) / WindowSize));
			lcd.print("%  ");
			return;
		}
		switch (menupid)
		{
		case MENU_Kp:
		{
			lcd.print("Kp: ");
			lcd.print(tKp);
			lcd.print("              ");
			return;
		}
		case MENU_Ki:
		{
			lcd.print("Ki: ");
			lcd.print(tKi);
			lcd.print("              ");
			return;
		}
		case MENU_Kd:
		{
			lcd.print("Kd: ");
			lcd.print(tKd);
			lcd.print("              ");
			return;
		}
		case MENU_EXIT:
		{
			lcd.print("APPLY & EXIT");
			lcd.print("      ");
			return;
		}
		case MENU_SAVE:
		{
			lcd.print("SAVE & EXIT");
			lcd.print("     ");
			return;
		}
		}
		return;
	}

	case MENU_CLEANING: // Cleaning menu
	{
		lcd.print("---   ");
		if (cleaning && !ledstat1) {
			lcd.setCursor(6, 1);
			if (minute(cleaningTime - (now() - upTime)) < 10) lcd.print("0");
			lcd.print(minute(cleaningTime - (now() - upTime)));
			lcd.print(":");
			if (second(cleaningTime - (now() - upTime)) < 10) lcd.print("0");
			lcd.print(second(cleaningTime - (now() - upTime)));
		}
		else lcd.print("00:00");
		lcd.print("  ---");

		break;

	case MENU_TIMER1: // Timer 1 menu
	{
		if (setupTime1 && !timer1Status) {
			if (!ledstat1) lcd.print("Step 1 -   ");
			lcd.setCursor(11, 1);
			if (minute(setupTime1) < 10) lcd.print("0");
			lcd.print(minute(setupTime1));
			lcd.print(":");
			if (second(setupTime1) < 10) lcd.print("0");
			lcd.print(second(setupTime1));
		}
		else {
			if (!ledstat1) lcd.print("Step 1 -   ");
			lcd.setCursor(11, 1);
			if (minute(currentTime1) < 10) lcd.print("0");
			lcd.print(minute(currentTime1));
			lcd.print(":");
			if (second(currentTime1) < 10) lcd.print("0");
			lcd.print(second(currentTime1));
		}
		break;
	}

	case MENU_TIMER2: // Timer 2 menu
	{
		if (setupTime2 && !timer2Status) {
			lcd.print("Step 2 -   ");
			if (minute(setupTime2) < 10) lcd.print("0");
			lcd.print(minute(setupTime2));
			lcd.print(":");
			if (second(setupTime2) < 10) lcd.print("0");
			lcd.print(second(setupTime2));
		}
		else {
			lcd.print("Step 2 -   ");
			if (minute(currentTime2) < 10) lcd.print("0");
			lcd.print(minute(currentTime2));
			lcd.print(":");
			if (second(currentTime2) < 10) lcd.print("0");
			lcd.print(second(currentTime2));
		}
		break;
	}
	}
	}
}


//****************************************************
//*      PID e Autotune                              *
//****************************************************


bool runPID()

{

	if (tuning)
	{
		upTime = now();
		byte val = (aTune.Runtime());
		if (val != 0)
		{
			tuning = false;
		}
		if (!tuning)
		{ //we're done, set the tuning parameters
			Kp = aTune.GetKp();
			Ki = aTune.GetKi();
			Kd = aTune.GetKd();
			myPID.SetTunings(Kp, Ki, Kd);
			menupid = 3;
			AutoTuneHelper(false);
			blinking(5, 1);
		}

	}
	else {
		myPID.Compute();
	}



}

void changeAutoTune()
{
	if (!tuning)
	{
		aTune.SetNoiseBand(aTuneNoise);
		aTune.SetOutputStep(aTuneStep);
		aTune.SetLookbackSec((int)aTuneLookBack);
		AutoTuneHelper(true);
		tuning = true;
	}
	else
	{ //cancel autotune
		aTune.Cancel();
		tuning = false;
		AutoTuneHelper(false);
	}
}

void AutoTuneHelper(boolean start)
{
	if (start)
		ATuneModeRemember = myPID.GetMode();
	else
		myPID.SetMode(ATuneModeRemember);
}
