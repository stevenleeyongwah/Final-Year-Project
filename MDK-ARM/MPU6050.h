#include "stm32f1xx_hal.h"
#include "math.h"

void MPU6050_config(void);
void MPU6050_pitch_calibration(void);
void MPU6050_detect_initial_angle(void);
void MPU6050_calculate_angle_gyro(void);
