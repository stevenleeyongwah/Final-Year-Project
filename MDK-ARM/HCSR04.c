#include "HCSR04.h"

extern uint32_t micro(void);

extern uint32_t echo1_pulse_width,echo3_pulse_width;
float echo1_rotating_mem[20],median1_distance,echo3_rotating_mem[20],median3_distance,echo_rotating_mem_temp[20];
uint8_t echo1_rotating_mem_location=0,echo3_rotating_mem_location=0;

void HCSR04()
{
		HAL_GPIO_WritePin(GPIOB,TRIGGER1_Pin,GPIO_PIN_RESET);
		delay_micro(5);
		
		HAL_GPIO_WritePin(GPIOB,TRIGGER1_Pin,GPIO_PIN_SET);
		delay_micro(15);
		HAL_GPIO_WritePin(GPIOB,TRIGGER1_Pin,GPIO_PIN_RESET);
		
		if(echo1_pulse_width >= 117 && echo1_pulse_width <= 23323)
		{
			echo1_rotating_mem[echo1_rotating_mem_location] = 0.0343*(float)echo1_pulse_width/2;
			++echo1_rotating_mem_location;
			if (echo1_rotating_mem_location == 20)echo1_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
			for(uint8_t i=0; i<20; ++i)
			{
					echo_rotating_mem_temp[i] = echo1_rotating_mem[i];
			}
			sort(echo_rotating_mem_temp,20);
			
			median1_distance = (echo_rotating_mem_temp[9] + echo_rotating_mem_temp[10])/2;
		}
		else
		{
			median1_distance = 2;
		}
		
		HAL_GPIO_WritePin(GPIOB,TRIGGER3_Pin,GPIO_PIN_RESET);
		delay_micro(5);
		
		HAL_GPIO_WritePin(GPIOB,TRIGGER3_Pin,GPIO_PIN_SET);
		delay_micro(15);
		HAL_GPIO_WritePin(GPIOB,TRIGGER3_Pin,GPIO_PIN_RESET);
		
		if(echo3_pulse_width >= 117 && echo3_pulse_width <= 23323)
		{
			echo3_rotating_mem[echo3_rotating_mem_location] = 0.0343*(float)echo3_pulse_width/2;
			++echo3_rotating_mem_location;
			if (echo3_rotating_mem_location == 20)echo3_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
			for(uint8_t i=0; i<20; ++i)
			{
					echo_rotating_mem_temp[i] = echo3_rotating_mem[i];
			}
			sort(echo_rotating_mem_temp,20);
			
			median3_distance = (echo_rotating_mem_temp[9] + echo_rotating_mem_temp[10])/2;
		}
		else{
			median3_distance = 2;
		}

}

void swap(float *p,float *q) {
   float t;
   
   t=*p; 
   *p=*q; 
   *q=t;
}

void sort(float a[],int n) { 
   int i,j;

   for(i = 0;i < n-1;i++) 
	 {
      for(j = 0;j < n-i-1;j++)
			{
         if(a[j] > a[j+1])
         {
					 swap(&a[j],&a[j+1]);
				 }
      }
   }
}

void delay_micro(uint32_t time)
{
	uint32_t temp = micro();
	while((micro() - temp) < time); 
}
