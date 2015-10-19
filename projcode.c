

#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include "keypad.h"
#include "mpu6050.c"
#include "twimastertimeout.c"


typedef struct task {
	int state;
	unsigned long period;
	unsigned long elapsedTime;
	int (*TickFct)(int);
} task;

task tasks[3];

const unsigned char tasksNum = 3;
const unsigned long tasksPeriodGCD = 1;
const unsigned long periodLCDSM =300;
const unsigned long periodGy = 10;
const unsigned long periodStepperMotorSM = 2;

const unsigned char DIRECTION_CW = 0;
const unsigned char DIRECTION_CCW = 1;

unsigned char direction = -1;

int deltaAngle = 0;

unsigned char motorCW[8] = { 0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09 };
unsigned char motorCCW[8] = { 0x01, 0x09, 0x08, 0x0c, 0x04, 0x06, 0x02, 0x03 };
int TickFct_Stepper_Motor(int state);

int TickFct_lcd(int state);

void TimerISR() {
	unsigned char i;
	for (i = 0; i < tasksNum; ++i) { // Heart of the scheduler code
		if ( tasks[i].elapsedTime >= tasks[i].period ) { // Ready
			tasks[i].state = tasks[i].TickFct(tasks[i].state);
			tasks[i].elapsedTime = 0;
		}
		tasks[i].elapsedTime += tasksPeriodGCD;
	}	
}

void TimerOn() {
	TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS11); // (ClearTimerOnCompare mode). // Prescaler=?
	TIMSK1 = (1 << OCIE1A); // Enables compare match interrupt
	SREG |= 0x80; // Enable global interrupts
}

void TimerSet(int ms) {
	TCNT1 = 0; // Clear the timer counter
	OCR1A = ms * 90; // Set the match compare value
}

ISR(TIMER1_COMPA_vect) {
	//RIOS
	TimerISR();
}

unsigned char setBit(unsigned char port, unsigned char pin, unsigned char val) {
	return (val ? port | (0x01 << pin) : port & ~(0x01 << pin));
}

unsigned char getBit(unsigned char port, unsigned char pin) {
	return ((port & (0x01 << pin)) !=0);
}
unsigned char stepspeed = 1;// 0 = fast 1 = norm 2 = slow
int rolltoangle( int roll)
{
	 if (roll > 26)
	 {
		if(stepspeed == 4)
			return 90;
		else 
			return 115;
	 }		
	else if (roll > 24)
		return 85;
	else if (roll > 22)
		return 65;
	else if (roll > 20)
		return 40;
	else if (roll > 17)
		return 25;
	else if (roll > 13)
		return 0;
	else if (roll > 11)
		return -25;
	else if (roll > 9)
		return -40;
	else if (roll > 7)
		return -60;
	else if (roll > 3)
		return -85;
	else if (roll <= 3)
		{
		if(stepspeed == 4)
			return -90;
		else 
			return -115;
		}
}	

void digitalWrite(unsigned char pin, unsigned char value){
	unsigned char writeValue = 0;
	if(value != 0){
		writeValue = 1;
	}

	switch(pin){
		case 1:
		PORTB = (PORTB & ~(1 << 0)) | (writeValue << 0);
		break;
		case 2:
		PORTB = (PORTB & ~(1 << 1)) | (writeValue << 1);
		break;
		case 3:
		PORTB = (PORTB & ~(1 << 2)) | (writeValue << 2);
		break;
		case 4:
		PORTB = (PORTB & ~(1 << 3)) | (writeValue << 3);
		break;
		case 5:
		PORTB = (PORTB & ~(1 << 4)) | (writeValue << 4);
		break;
		case 6:
		PORTB = (PORTB & ~(1 << 5)) | (writeValue << 5);
		break;
		default:
		/* Bad pin number, insert error message here. */
		break;
	}
}
void dataIn(unsigned char datapin , unsigned char clockpin, unsigned char value) // put data into pin
{
	short num;
	for (unsigned char i = 0; i < 8; i++)
	{
		num = (value >> i) & (0x01) ;
		dataIn1 ( datapin, clockpin, num);
		
	}

}
void dataIn1(unsigned char datapin , unsigned char clockpin, unsigned char value)
{

	digitalWrite(datapin,  value);
	digitalWrite(clockpin, 1);
	digitalWrite(clockpin, 0);
}
void draw(unsigned char num)
{
	
	
	digitalWrite(6, 1);
	digitalWrite(4, 0);
	digitalWrite(5, 0);
	dataIn(3,5,255);//columns
	if(num > 26){
		dataIn(3,5,255);//columns
		dataIn(3,5,255);//green
		dataIn(3,5,255);//blue 0 = all on
		dataIn(3,5,15);// red
	}
	else if(num > 24){
		dataIn(3,5,255);//columns
		dataIn(3,5,143);//green
		dataIn(3,5,255);//blue 0 = all on
		dataIn(3,5,143);// red
	}
	else if(num > 20){
		dataIn(3,5,255);//columns
		dataIn(3,5,207);//green
		dataIn(3,5,255);//blue 0 = all on
		dataIn(3,5,255);// red
	}
	else if(num > 17){
		dataIn(3,5,255);//columns
		dataIn(3,5,239);//green
		dataIn(3,5,255);//blue 0 = all on
		dataIn(3,5,255);// red
	}
	else if(num > 13){
		dataIn(3,5,255);//columns
		dataIn(3,5,255);//green
		dataIn(3,5,231);//blue 0 = all on
		dataIn(3,5,231);// red
	}
	else if(num > 10){
		dataIn(3,5,255);//columns
		dataIn(3,5,247);//green
		dataIn(3,5,255);//blue 0 = all on
		dataIn(3,5,255);// red
	}
	else if(num > 7){
		dataIn(3,5,255);//columns
		dataIn(3,5,243);//green
		dataIn(3,5,255);//blue 0 = all on
		dataIn(3,5,255);// red
	}
	else if(num > 3){
		dataIn(3,5,255);//columns
		dataIn(3,5,241);//green
		dataIn(3,5,255);//blue 0 = all on
		dataIn(3,5,241);// red
	}
	else if(num <= 3){
		dataIn(3,5,255);//columns
		dataIn(3,5,255);//green
		dataIn(3,5,255);//blue 0 = all on
		dataIn(3,5,240);// red
	}
	digitalWrite(4, 3);
	
}
unsigned char returnmenu = 0;
unsigned char modeflag = 1; // 0 = auto 1 = manual
unsigned char z = 'a';
unsigned char resetflag= 0;

enum lcd_SM_States {start,display, wait, mode, modewait1,modewait2, speed, speedwait1,speedwait2,reset,resetwait };
int TickFct_lcd(int state) {
	unsigned char x = GetKeypadKey();
	
	switch(state){
		case -1:
			state = start;
		break;
		case start:
			if(x == 'D')
			state = display;
		break;
		case display:
			state = wait;
		break;
	
		case wait:
			if (x =='A')
			state = mode;
			else if(x == 'B')
			state = speed;
			else if(x == 'C')
			state = reset;
		break;
		case mode:
			
			state = modewait1;
		break;
		case modewait1:
				if(x != 'A');
				state = modewait2;
		break;
		case modewait2:
			if(returnmenu == 1 )
			state = start;
		break;
		
		case speed:
			state = speedwait1;
		break;
		case speedwait1:
				if(x != 'B');
					state = speedwait2;
				break;
		case speedwait2:
			if(returnmenu == 1 )
			state = start;
		break;
		case reset:
			state = resetwait;
		break;
		case resetwait:
			if(returnmenu == 1 )
			state = start;
		break;
	}
	switch(state){
		case start:
			returnmenu = 0;
		break;
		case display:
			LCD_DisplayString( 1, "A: Mode B: SpeedC: Original Pos");
			
		break;
		case wait:
		break;
		case mode:
			LCD_DisplayString( 1, "Press A: Auto   Press B: Manual");
		break;
		case modewait1:
			
		break;
		case modewait2:
			if (x == 'A')
			{
				LCD_DisplayString( 1, "Auto is set.    D: Main Menu");
				modeflag = 0;
				returnmenu = 1;
			}			
			else if(x == 'B'){
				LCD_DisplayString( 1, "Manual is set.  D: Main Menu");
				modeflag = 1;
				returnmenu = 1;
			}			
		break;
		
		case speed:
			LCD_DisplayString( 1, "A: Fast B: Slow C: Normal");
		break;
		
		case speedwait2:
			if (x == 'A'){
				LCD_DisplayString( 1, "Speed is Fast   D: Main Menu");
				stepspeed = 0;
				returnmenu = 1;}
			else if(x == 'B'){
				LCD_DisplayString( 1, "Speed is Slow   D: Main Menu");
				stepspeed = 4;
				returnmenu = 1;}
			else if(x == 'C'){
				LCD_DisplayString( 1, "Speed is Normal D: Main Menu");
				stepspeed = 2;
				returnmenu = 1;}
		break;	
		case reset:
		LCD_DisplayString(1, "Are you sure?   A:Yes B:No");
		break;
		case resetwait:
			if (x == 'A'){
				if(modeflag ==1){
					LCD_DisplayString( 1, "Resetting Pos.  D: Main Menu");
					resetflag = 1;
				}
				if(modeflag ==0){
					LCD_DisplayString( 1, "Must be Manual  D: Main Menu");
					resetflag = 1;
				}

			returnmenu = 1;}
			else if(x == 'B'){
				LCD_DisplayString( 1, "Canceled        D: Main Menu");
				resetflag = 0;
			returnmenu = 1;}

		break;
		
	}
	return state;
}	


//variables for the gyro
double qw = 1.0f;
double qx = 0.0f;
double qy = 0.0f;
double qz = 0.0f;
double roll = 0.0f;
double pitch = 0.0f;
double yaw = 0.0f;
unsigned char x = 0;
unsigned char rollnum = 0;
int targetangle = 0;
int angle = 0;
unsigned char turningflag = 0;
int TickFct_Gy(int state);
enum Gy_States { Wait_gy,Turning };
int TickFct_Gy(int state) 
	{
			unsigned char b0 = !getBit(PINB, 0);
			unsigned char b1 = !getBit(PINB, 1);
		switch (state) {
			case -1:
				state = Wait_gy;
			break;
			case Wait_gy:
				if(turningflag == 1)
				state = Turning;
			break;
			case Turning:
				if (turningflag == 0)
				{
					state = Wait_gy;
				}
			break;
			

		}	
		switch (state) {
			case -1:
			break;
			case Wait_gy:
				mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
				mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
				if(roll < 0)
				{
					rollnum = 16 - (roll * -1 * 10);
					if (rollnum < 1)
						rollnum = 1;
				}
				else if(roll >0)
				{
					rollnum = 16 + (roll * 10);
				}
				else if (roll == 0  )
				{
					rollnum = 16;
				}
				draw(rollnum);
				if (resetflag == 1)
				{
					resetflag = 0;
					targetangle = 0;
					if ( targetangle > deltaAngle)
					{
						direction = DIRECTION_CW;
						angle = (targetangle - deltaAngle);
					}
					else if (targetangle < deltaAngle)
					{
						direction = DIRECTION_CCW;
						angle = ((targetangle - deltaAngle) * (-1));
					}
				}
				else if (modeflag == 0)
				{
					targetangle = rolltoangle( rollnum);
					if ( targetangle > deltaAngle)
					{
						direction = DIRECTION_CW;
						angle = (targetangle - deltaAngle);
					}
					else if (targetangle < deltaAngle)
					{
						direction = DIRECTION_CCW;
						angle = ((targetangle - deltaAngle) * (-1));
					}
				}
				else if(modeflag == 1)
				{
					if (b1 ) {
						if(deltaAngle < 115)
						{
							direction = DIRECTION_CW;
							angle = 15;
						}


					} else if ( b0) {
						if (deltaAngle > -115 )
						{
							direction = DIRECTION_CCW;
							angle = 15;
						}

					}
				
				}

				
				
			break;
			case Turning:
				mpu6050_getQuaternion(&qw, &qx, &qy, &qz);
				mpu6050_getRollPitchYaw(&roll, &pitch, &yaw);
				if(roll < 0)
				{
					rollnum = 16 - (roll * -1 * 10);
					if (rollnum < 1)
						rollnum = 1;
				}
				else if(roll >0)
				{
					rollnum = 16 + (roll * 10);
				}
				else if (roll == 0  )
				{
					rollnum = 16;
				}
				draw(rollnum);
			break;
				}
	return state;	
	}

unsigned char tick = 0;
enum Stepper_Motor_SM_States { Motor_Wait, Motor_CW, Motor_CCW };
int TickFct_Stepper_Motor(int state) {
	static char count = 0;
	static int phaseCount = 0;
	static int numPhases = 0;
	
	switch (state) {
		case -1:
			state = Motor_Wait;
			break;
		case Motor_Wait:
			if (angle > 0) {
				state = direction == DIRECTION_CW ? Motor_CW : Motor_CCW;
				numPhases = (angle / 5.625) * 64;
			}
			break;	
		case Motor_CW:
			if (phaseCount >= numPhases) {
				deltaAngle += angle;
				angle = 0;
				tick = 0;
				state = Motor_Wait;
			}
			break;
		case Motor_CCW:
			if (phaseCount >= numPhases) {
				deltaAngle -= angle;
				angle = 0;
				tick = 0;
				state = Motor_Wait;
			}
			break;
		default:
			state = -1;
			break;
	}
	
	switch (state) {
		case -1:
			break;
		case Motor_Wait:
			PORTA = 0x00;
			count = 0;
			phaseCount = 0;
			turningflag = 0;
			break;
		case Motor_CW:
			turningflag = 1;
			if(stepspeed <= tick)
			{
				tick = 0;
				PORTA= motorCW[count++];
				phaseCount++;
				if (count >= sizeof(motorCW)) {
					count = 0;
				}
			}
			tick ++;

			break;
		case Motor_CCW:
			turningflag = 1;
			if(stepspeed <= tick)
			{
				tick = 0;
				PORTA= motorCCW[count++];
				phaseCount++;
				if (count >= sizeof(motorCCW)) {
					count = 0;
				}
			}			
			tick ++;
			break;
		default:
			break;
	}
	
	return state;
}
int main(void)
{
	DDRA = 0x0F; PORTA = 0xF0;
	DDRB = 0xFC; PORTB = 0x03;
	DDRD = 0xFF; PORTD = 0x00;
	DDRC = 0x80; PORTC = 0x7F;

	mpu6050_init();
	_delay_ms(50);
	
	LCD_init();
	
	unsigned char i = 0;
	tasks[i].state = -1;
	tasks[i].period = periodLCDSM;
	tasks[i].elapsedTime = tasks[i].period;
	tasks[i].TickFct = &TickFct_lcd;
	i++;
	tasks[i].state = -1;
	tasks[i].period = periodGy;
	tasks[i].elapsedTime = tasks[i].period;
	tasks[i].TickFct = &TickFct_Gy;
	i++;
	tasks[i].state = -1;
	tasks[i].period = periodStepperMotorSM;
	tasks[i].elapsedTime = tasks[i].period;
	tasks[i].TickFct = &TickFct_Stepper_Motor;
	TimerSet(tasksPeriodGCD);
	TimerOn();
    while(1)
    {
        //TODO:: Please write your application code 
    }
}