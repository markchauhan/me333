#include "nu32dip.h"
#include "encoder.h"
#include "utilities.h"
#include "ina219.h"
#include "current.h"
#include "position.h"

#define BUF_SIZE 200

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

int trajectory_length;
float trajectory;

volatile float error; 


int main(void){
    char buffer[BUF_SIZE];
    NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
    NU32DIP_GREEN = 1; // turn off the LEDs
    NU32DIP_YELLOW = 1;

     __builtin_disable_interrupts();
    // in future, initialize modules or peripherals here
    UART2_Startup();
    current_initialize();
    INA219_Startup();

     __builtin_enable_interrupts();
     set_mode(0); //IDLE
     

    while(1){
        NU32DIP_ReadUART1(buffer,BUF_SIZE); // we expect the next character to be a menu command
        NU32DIP_YELLOW = 1; // clear the error LED
        switch (buffer[0]) {
            case 'a': //read current
            {   char current_val[50];
                sprintf(current_val, "\nCurrent(ma): %f \r\n", INA219_read_current());
                NU32DIP_WriteUART1(current_val);
                break;
            }
            case 'c': //read encoder
            {   WriteUART2("a");
                while(!get_encoder_flag()){
                }
                set_encoder_flag(0);
                char encoder_value[50];
                int encoder_count = get_encoder_count();
                sprintf(encoder_value,"%d\r\n", encoder_count);
                NU32DIP_WriteUART1(encoder_count);
                break;
            }
            case 'd': //read encoder in degrees
            {   WriteUART2("a");
                while(!get_encoder_flag()){
                }
                set_encoder_flag(0);
                char encoder_value[50];
                int encoder_count = get_encoder_count();
                float degrees = encoder_count *360.0/(1400.0);
                sprintf(encoder_count,"%f\r\n", degrees);
                NU32DIP_WriteUART1(encoder_count);
                break;
            }
            case 'e':
            {   WriteUART2("b");
                break;
            }
            case 'f':{ // Set PWM (-100 to 100)
                set_mode(1);
                char pwm_value[20];
                NU32DIP_ReadUART1(pwm_value, BUF_SIZE);
                sscanf(pwm_value, "%d", &pwm);
                break;
            }
            case 'g':{
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &Jp);
            
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &Ji);			
                break;                
            }
            case 'h':
		    {  sprintf(buffer, "%f\r\n", Jp);
                NU32DIP_WriteUART1(buffer);
                
                sprintf(buffer, "%f\r\n", Ji);
                NU32DIP_WriteUART1(buffer);			
                break;
		    }   
            case 'i':{
                //Kp position
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &Kp);

                //Ki position
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &Ki);

                //Kd position
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &Kd);
                break; 
            }
            case 'j':{
                //Kp
                sprintf(buffer, "%f\r\n", Kp);
                NU32DIP_WriteUART1(buffer);
                //Ki
			    sprintf(buffer, "%f\r\n", Ki);
			    NU32DIP_WriteUART1(buffer);			
                //Kd
			    sprintf(buffer, "%f\r\n", Kd);
			    NU32DIP_WriteUART1(buffer);			
			    break;
            }
            case 'k':
		    {   //ITEST mode
               //initialize
                eint = 0;
                eprev = 0;
                edot = 0;
                pwm_int = 0;
                current_count = 0;
                set_mode(2);
                
                //cc ISR at 5KHz
                //pause till it reaches
                while (get_mode() == 2) {}
                sprintf(buffer, "%d\r\n", 100); 
                NU32DIP_WriteUART1(buffer);
                current_info();		
                break;
		    } 
            case 'l':
		    {//move to a position
                position_count = 0;
                input_position = 0;
                eint = 0;
                eprev = 0;
                edot = 0;
                pwm_int = 0;
                current_count = 0;

                NU32DIP_WriteUART1("\nEnter Value: \r\n");
                NU32DIP_ReadUART1(buffer, BUF_SIZE);
                sscanf(buffer, "%f", &input_position);
                sprintf(buffer, "Value: %f\r\n", input_position);
			    NU32DIP_WriteUART1(buffer);	
                set_mode(HOLD); //wait till p command		
                break;
		    }  
            case 'm':
            {//step trajectory
            read_position();
            break;}
            case 'n':
            {//cubic trajectory
            read_position();	
            break;}
            case 'o':
            {//m or n traj set
            //initialize
                eint = 0;
                eprev = 0;
                edot = 0;
                pwm_int = 0;
                current_count = 0;
                input_position = 0;
                position_count = 0;
                set_mode(TRACK);
                //wait
                while (get_mode() == TRACK) {
                }
                //send data
                position_info();
                //clear array
                for (int k = 0; k < trajectory_length; k++) {
                    position_array[k] = 0;}
                break;
            }
            case 'p':{//unpower
                NU32DIP_WriteUART1("\nMotor unpowered.\r\n");
                set_mode(0);
                break;
            }
            case 'q':
            {
                // handle q for quit. Later you may want to return to IDLE mode here.
                mode = IDLE;
                break;
            }
            case 'r': //read the mode of the motor
            {
                char motor_condition[20];
                sprintf(motor_condition, "\nMode: %d \r\n", get_mode());
                NU32DIP_WriteUART1(motor_condition);
                break;
            }
            case 'x': // dummy command for demonstration purposes
            {
                int n = 0;
                NU32DIP_ReadUART1(buffer,BUF_SIZE);
                sscanf(buffer, "%d", &n);
                sprintf(buffer,"%d\r\n", n + 1); //return the number + 1
                NU32DIP_WriteUART1(buffer);
                break;
            }
            default:
            {
                NU32DIP_YELLOW = 0; //Turn on Yellow LED for error
                break;
            }
        }
    }
    return 0;
}