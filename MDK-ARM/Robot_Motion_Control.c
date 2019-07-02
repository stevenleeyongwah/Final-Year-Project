#include "Robot_Motion_Control.h"
#include "PID_Motor_Control.h"
#include "NRF24L01.h"

extern TIM_HandleTypeDef htim2;

extern uint8_t balancing, standing, sleeping, rise_arm, received_byte, received_com_byte;
extern uint16_t throttle_left_motor,throttle_right_motor;
extern float angle_gyro, pid_i_mem, pid_output, self_balance_pid_setpoint;

uint8_t servo_pos,servo=0,arm_rising_time=100,servo_delay=0;

void Raise_Or_Lower_Down_Robot()
{
			if(standing == false && sleeping == false)
			{
					// NRF24L01 Remote Control
					check_NRF24L01_payload();
					
					if(received_byte == 0x05 || received_com_byte == 0x05)
					{
							if(balancing == false)
							{
								standing = true;
								arm_rising_time = 0;
							}
							else if(balancing == true)
							{
								sleeping = true;
							}
							HAL_GPIO_WritePin(CE_PORT, CE_Pin, GPIO_PIN_RESET); // Disable transmitter
							NRF24_FlushRx();
							servo_pos = 25;
							servo_delay = 0;
							received_byte = 0;
							received_com_byte = 0;
					}
			}
}

void Change_Robot_Position()
{
			if(balancing == false && standing == true) // Raises the robot up from sleeping position
			{		
					if(servo_delay == 20)
					{
							if(angle_gyro > 0)
							{
								servo = 1;
								htim2.Instance->CCR1 = servo_pos;
							}
							else if(angle_gyro < 0)
							{
								servo = 2;
								htim2.Instance->CCR2 = servo_pos;
							}
							if(servo_pos < 120)++servo_pos;
							servo_delay = 0;
					}
					else
					{
							++servo_delay;
					}
			}
			else if(balancing == true && sleeping == true) // Rotate servo arm until it almost touches the floor and lean robot forward
			{		
					if(servo_delay == 20)
					{
							if(servo_pos < 93)
							{
								servo_pos = servo_pos + 1;
								htim2.Instance->CCR1 = servo_pos;
							}
							servo_delay = 0;
					}
					
					if(servo_pos == 93)
					{
							received_byte = 0x08;
							if(angle_gyro > 2)
							{
								 balancing = false;
								 servo_delay = 0;
							}
					}
					else
					{
							++servo_delay;
					}
			}
			else if(balancing == false && sleeping == true) // Bring the robot to sleeping position
			{
					if(servo_delay == 30)
					{
							if(angle_gyro <= 82)
							{
								--servo_pos;
								htim2.Instance->CCR1 = servo_pos;
							}
							servo_delay = 0;
					}
					
					if(angle_gyro > 82)
					{
							sleeping = false;
							htim2.Instance->CCR1 = 0;
							HAL_GPIO_WritePin(CE_PORT, CE_Pin, GPIO_PIN_SET); // Enable transmitter
					}
					else
					{
							++servo_delay;
					}
			}
}

void Check_If_Robot_Enters_Vertical_Position()
{
			if(balancing == false && angle_gyro > -0.5 && angle_gyro < 0.5)		// Check if robot enters vertical position
			{  
					standing = false;
					balancing = true;                                       // Enable balancing mode
					HAL_GPIO_WritePin(CE_PORT, CE_Pin, GPIO_PIN_SET);				// Enable NRF24L01 receiver to control movement of robot
			}	
}
// Rotates servo arm to vertical position if servo arm is not in vertical position
void Rotate_Servo_Arm_To_Vertical_Position()
{
				if(balancing == true && arm_rising_time < 100)
				{
						if(servo == 1)
						{
							htim2.Instance->CCR1 = 25;
						}
						else if(servo == 2)
						{
							htim2.Instance->CCR2 = 25;
						}

						if(arm_rising_time == 100)
						{
							if(servo == 1)
							{
								htim2.Instance->CCR1 = 0;
							}
							else if(servo == 2)
							{
								htim2.Instance->CCR2 = 0;
							}
						}
						else
						{
							++arm_rising_time;
						}
				}
}

void Check_If_Robot_Tips_Over()
{
			if(angle_gyro > 30 || angle_gyro < -30 || balancing == false)     	//If the robot tips over or the balancing variable is zero or the battery is empty
			{
					pid_output = 0;                                                 //Set the PID controller output to 0 so the motors stop moving
					pid_i_mem = 0;                                                  //Reset the I-controller memory
					throttle_left_motor = 0;
					throttle_right_motor = 0;
					self_balance_pid_setpoint = 0;                                  //Reset the self_balance_pid_setpoint variable
					balancing = false;
			}
}

