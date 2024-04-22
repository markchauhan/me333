#include "nu32dip.h"
#include "encoder.h"
#include "utilities.h"
#include "ina219.h"
#include "current.h"
#include "position.h"


float Kp, Ki, Kd;

volatile int pwm; 
volatile int current_count;
volatile int position_count;
volatile float input_position; 
volatile float input_current;
float Kp, Ki, Kd;
volatile float Ji, Jp;
volatile float pwm_value, pwm_int;
volatile float volt;
volatile float current;
volatile float e, eprev, eint, edot;

float current_array[100];
float reference_array[100];

float trajectory_array[3000];
float position_array[3000];
float temp[100];

int trajectory_length;
float trajectory;

volatile float error; 


//200Hz ISR
void __ISR(_TIMER_4_VECTOR, IPL4SOFT) PositionControl(void) {
	switch(mode) {
		case HOLD:
		{positionPID();
		position_count = 0; 
		break;
		}
		case TRACK:{
			input_position = trajectory_array[position_count];
			positionPID();	
			//end
			if (position_count > (trajectory_length - 1)) {
				position_count = trajectory_length - 1;
				input_position = trajectory_array[position_count];
				set_mode(HOLD);
				break;}
			break;}
		default:{
			break;}		
	}

	IFS0bits.T4IF = 0; //clear interrupt flag
}

void positionPID(void) {
	
	//read encoder 
	WriteUART2("a");
	
    //delay
	while (!get_encoder_flag()) {
	}
	set_encoder_flag(0); 
	float position = get_encoder_count();
		
	//PID control
	e = -(input_position * 1400 / 360) + position;

	edot =  (e - eprev)/ 0.005;

	if (eint > 150){
		eint = 150;
	} else if (eint < 150) {
		eint = -150;
	}

	error = Kp * e + Ki * eint + Kd * edot;
    position_array[position_count] = position * 360 / 1400;			
    //new value to current
	input_current = error;
	
	//increment
	eint += e;
	eprev = e;
	position_count++;
}


void position_info(void) {
	//send datapints
	sprintf(temp, "%d\r\n", trajectory_length);
	NU32DIP_WriteUART1(temp);
	
	for (int i = 0; i < trajectory_length; ++i) {
		sprintf(temp, "%f %f\r\n", trajectory_array[i], position_array[i]);
		NU32DIP_WriteUART1(temp);	
	}	
}

void read_position(void) {	
	//length of the trajectory array
	NU32DIP_ReadUART1(temp, BUF_SIZE);
	sscanf(temp, "%d", &trajectory_length);
	
	//read the commands
	for (int j = 0; j < trajectory_length; j++) {
		NU32DIP_ReadUART1(temp, BUF_SIZE);
		sscanf(temp, "%f", trajectory_array + j); 
	}
}
