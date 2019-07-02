#include "MPU6050.h"

extern I2C_HandleTypeDef hi2c2;
extern float angle_gyro;

uint8_t buffer[6];
int16_t gyro_pitch_calibration_value;
int16_t acc_calibration_value=1525; // Enter your own accelerometer calibration value when your robot is balance
int16_t gyro_pitch_data_raw, accelerometer_data_raw;
float angle_acc;
	
void MPU6050_config()
{
			buffer[0] = 0x6B;
			buffer[1] = 0x00;
			HAL_I2C_Master_Transmit(&hi2c2,0x68<<1,buffer,2,100);
			buffer[0] = 0x1B;
			buffer[1] = 0x00;
			HAL_I2C_Master_Transmit(&hi2c2,0x68<<1,buffer,2,100);
			buffer[0] = 0x1C; // Accelerometer config register
			buffer[1] = 0x08; // AFS_SEL = 4g
			HAL_I2C_Master_Transmit(&hi2c2,0x68<<1,buffer,2,100);
			buffer[0] = 0x1A;
			buffer[1] = 0x03;
			HAL_I2C_Master_Transmit(&hi2c2,0x68<<1,buffer,2,100);
}

void MPU6050_pitch_calibration()
{
			for(uint16_t i=0; i<500; ++i)
			{
					if(i%20 == 0)
					{
						HAL_GPIO_TogglePin(GPIOC,LED_Pin);
					}
					
					buffer[0] = 0x45;
					HAL_I2C_Master_Transmit(&hi2c2,0x68<<1,buffer,1,200);
					HAL_I2C_Master_Receive(&hi2c2,0x68<<1,buffer,2,500);

					gyro_pitch_calibration_value += buffer[0]<<8 | buffer[1];
					
					HAL_Delay(4);		
			}
			
			gyro_pitch_calibration_value /= 500; 
}

void MPU6050_detect_initial_angle()
{
			buffer[0] = 0x3F; // Accelerometer Z axis address
			HAL_I2C_Master_Transmit(&hi2c2,0x68<<1,buffer,1,100);
			HAL_I2C_Master_Receive(&hi2c2,0x68<<1,buffer,2,100);
			accelerometer_data_raw = buffer[0]<<8 | buffer[1];
			accelerometer_data_raw += acc_calibration_value;
			if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;
			if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;
			
			angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296; 
			angle_gyro = angle_acc;
}

void MPU6050_calculate_angle_gyro()
{
			// Calculate the angle of accelerometer
			buffer[0] = 0x3F; // Accelerometer Z axis address
			HAL_I2C_Master_Transmit(&hi2c2,0x68<<1,buffer,1,100);
			HAL_I2C_Master_Receive(&hi2c2,0x68<<1,buffer,2,100);
			accelerometer_data_raw = buffer[0]<<8 | buffer[1];
			accelerometer_data_raw += acc_calibration_value;
			if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;
			if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;
			
			angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296; 
			
			buffer[0] = 0x45;
			HAL_I2C_Master_Transmit(&hi2c2,0x68<<1,buffer,1,100);
			HAL_I2C_Master_Receive(&hi2c2,0x68<<1,buffer,2,100);
			gyro_pitch_data_raw = buffer[0]<<8 | buffer[1];

			gyro_pitch_data_raw -= gyro_pitch_calibration_value;

			angle_gyro += gyro_pitch_data_raw*0.000031;
						
			angle_gyro = angle_gyro*0.9996 + angle_acc*0.0004;
}
