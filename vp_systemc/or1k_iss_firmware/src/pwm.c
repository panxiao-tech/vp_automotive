/**
 * @file pwm.c
 * @author Xiao Pan <pan@cs.uni-kl.de>,
 *         AG Design of Cyber-Physical Systems
 * @date  04.05.2018
 * @brief PWM Driver in or1k Firmware
 *
 * @section LICENSE License
 *
 *
 * Copyright 2012-2018 Workgroup Design of Cyber-Physical Systems.
 * University of Kaiserslautern.
 *
 * @section DESCRIPTION Description
 *
 */


#include "../../hw_def/config_hw.h"
#include "orsocdef.h"

#include "pwm.h"
#include <math.h>
#include "stdio.h"


const unsigned int pwm_steps = 255;


void pwm_init()
{
//	printf( "PWM initialization. \n");

	// Reserved
	// TBD
	return;
}


// -----------------------------------------------------------------------------
//! @brief Send duty cycle data to PWM, data length = 1byte

//! @param duty  duty cycle, unit = 1/1000
// -----------------------------------------------------------------------------
void pwm_send_duty(double duty)
{
	//	printf ("updaed pwm_duty=%d\n", duty);

	uint16  pwm_duty = fabs(duty) * pwm_steps;

	// saturation
	if(pwm_duty > pwm_steps) pwm_duty = pwm_steps;


	// assign duty low and high bytes to the relevant registers
	uint8 duty_h, duty_l, ctrl;
	duty_h = (pwm_duty>>8) & 0xFF;
	duty_l = pwm_duty & 0xFF;

	// set the first bit to 1 to indicate the negative voltage output
	if(duty < 0){
		duty_h |= 0x80;
	}


	REG8(PWM_BASE+PWMDUTYL) = duty_l & 0xFF; 	// write date to duty low byte
	REG8(PWM_BASE+PWMDUTYH) = duty_h & 0xFF;	// write date to duty high byte

	ctrl = REG8(PWM_BASE+PWMCTRL);
	ctrl |= PWM_CTRL_UDC;

	REG8(PWM_BASE+PWMCTRL)  = ctrl & 0xFF;
	return;
}


unsigned int pwm_get_duty()
{
	// Reserved
	// TBD
	return 0;
}




