#include "current.h"
#include "NU32DIP.h"
#include "utilities.h"
#include "ina219.h"
#include "encoder.h"



volatile int pwm; 
volatile int current_count;
volatile float input_current;
volatile float Ji, Jp;
volatile float pwm_value, pwm_int;
volatile float volt;
volatile float current;
volatile float e, eprev, eint, edot;

//arrays
float current_array[100];
float reference_array[100];
char temp[BUF_SIZE];


void current_initialize(void){

    TRISAbits.TRISA1 = 0; 
    LATAbits.LATA1 = 0;

    // Timer 2
    T2CONbits.TCKPS = 0b001; // 2,2000
    T2CONbits.T32 = 0;
    PR2 = 2000;

    // Timer 3 5kHz
    T3CONbits.TCKPS = 0b011; // 8, 2000
    PR3 = 2000;

    // Timer 4 200Hz
    T4CONbits.TCKPS = 0b100; // 16, 25000
    T4CONbits.T32 = 0;
    PR4 = 25000;

    OC1CONbits.ON = 0;      // Turn off OC1 
    OC1CONbits.OCM = 0b110; // Configure for PWM 
    OC1CONbits.OCTSEL = 0;  // Timer 2
    OC1CONbits.SIDL = 1;    // Stop for IDLE
    OC1CONbits.OC32 = 0;    
    OC1R = PR2 / 4; // Initialize for duty cycle
    OC1RS = PR2;
	RPA0Rbits.RPA0R=0b0101;
    
	IPC3bits.T3IP = 5;
    IPC3bits.T3IS = 0;
    IPC4bits.T4IP = 4;
    IPC4bits.T4IS = 0;

    // On
    OC1CONbits.ON = 1;
    T2CONbits.ON = 1; // PWM
    T3CONbits.ON = 1; // current
    T4CONbits.ON = 1; // position

    IEC0bits.T3IE = 1;
    IEC0bits.T4IE = 1;
}


//time for current
void __ISR(_TIMER_3_VECTOR, IPL5SOFT) CurrentControl(void) {
    switch (get_mode()) {
        case IDLE:
        {   OC1RS = 0;
            break;
        }
        case PWM:{
            if (pwm>0){   
                DIRECTION = 0; 
                OC1RS = (float) pwm/100.0 * 2400;
                OC1RS = (int) (OC1RS);
                }
            else{               
                DIRECTION = 1;
                OC1RS = (float) pwm/-100.0 * 2400;
                OC1RS = (int)(OC1RS);
            break; }
        break;}
        case ITEST:
        {if (current_count < 25) {
				input_current = 200;
			} else if (current_count < 50) {
				input_current = -200;
			} else if (current_count < 75) {
				input_current = 200;
			} else if (current_count < 100) {
				input_current = -200;
			} else {
				set_mode(IDLE);
				break;
			}	
			currentPID();
			break;}
		case HOLD:
		{currentPID();
		 current_count--;
         break;}
		case TRACK:
		{//for position PID
		currentPID();	
		current_count--;
		break;} 
    }
    IFS0bits.T3IF = 0; //clear interrupt flag 
}


void currentPID() {
	//ref varies
	current = INA219_read_current();
	
    //store currents
	current_array[current_count] = current;
	reference_array[current_count] = input_current;

	//PI for current
	pwm_value = input_current - current;
	volt = Jp * pwm_value + Ji * pwm_int;

	//PI bounds
	if (volt > 100.0) {
		volt = 100.0; 
	} else if (volt < -100.0) {
		volt = -100.0;
	}
	
	//PI to PWM
    //positive and negative voltages
	if (volt < 0) {	
		LATAbits.LATA1 = 1;
		OC1RS =  -volt * PR2/100;
        OC1RS =  -volt * -1;
	} else {
		LATAbits.LATA1 = 0;
		OC1RS =  volt * PR2/100;		
	}
	//increment
	pwm_int += pwm_value;
	current_count++;
}

void current_info() {
	for (int i = 0; i < 100; ++i) {
		sprintf(temp, "%f %f\r\n", reference_array[i], current_array[i]);
		NU32DIP_WriteUART1(temp);	
	}
}
