/**
 * @file main.c
 * @author Xiao Pan <pan@cs.uni-kl.de>,
 *         AG Design of Cyber-Physical Systems
 * @date  04.05.2018
 * @brief OR1K Embedded Software of project "ECU_ISS"
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
#include "adc.h"
#include "pwm.h"


//// ----------------------------------------------------------------------------
//// ADC(pedal position sensor) parameters
//// ----------------------------------------------------------------------------
//const double vref_app = 3.0; // adc reference voltage
//const int res_app = 12; // resolution
//const int rate_app = 1;
//
//// ----------------------------------------------------------------------------
//// ADC(throttle position sensor) parameters
//// ----------------------------------------------------------------------------
//const double vref_throttle = 3.0; // adc reference voltage
//const int res_throttle = 12; // resolution
//const int rate_throttle = 1;

// ----------------------------------------------------------------------------
//  controller
// ----------------------------------------------------------------------------
const double gc_t_reg = 0.5e-3; 	// regulation time in second
const double gc_pid_p = 0.8; 		// PID controller proportional gain
const double gc_pid_i = 0.4; 		// PID controller integral
const double gc_pid_comp = 0.11; 	// compensator factor of pre-loaded torque


// ----------------------------------------------------------------------------
// ADC(pedal position sensor) parameters
// ----------------------------------------------------------------------------
const double gc_adc_app_vref = 3.0; // adc reference voltage
const int    gc_adc_app_res  = 12;      // resolution
const int    gc_adc_app_rate = 1;

// ----------------------------------------------------------------------------
// ADC(throttle position sensor) parameters
// ----------------------------------------------------------------------------
const double gc_adc_throttle_vref = 3.0; // adc reference voltage
const int    gc_adc_throttle_res  = 12; // resolution
const int    gc_adc_throttle_rate = 1;




// ----------------------------------------------------------------------------
// global static variables for pid controller algrithm
// ----------------------------------------------------------------------------
static double       gsLastErr   = 0.0;
static double       gsCurrErr   = 0.0;
static double       gsLastCtrl  = 0.0;
static double       gsCurrCtrl 	= 0.0;


// -----------------------------------------------------------------------------
//! @brief PID controller
//! @param _currVal readout value

//! @return  updated pwm duty cycle value
// -----------------------------------------------------------------------------
double pid_calc(double tarVal,  double currVal, double pid_za, double pid_zb )
{
    gsLastErr  = gsCurrErr;
    gsLastCtrl = gsCurrCtrl;
    gsCurrErr  = tarVal - currVal;
//    gsCurrCtrl = gsLastCtrl  - 0.7998 * gsLastErr + 0.8 * gsCurrError;
    gsCurrCtrl = gsLastCtrl  - pid_zb * gsLastErr + pid_za * gsCurrErr;
    double ctrl_out = gsCurrCtrl + gc_pid_comp;

    // saturation
    if(ctrl_out > 1)
    {
        ctrl_out = 1;
        gsCurrCtrl = 1- gc_pid_comp;
    }
    if(ctrl_out < -1) {
        ctrl_out = -1;
        gsCurrCtrl = -1 - gc_pid_comp;
    }

//	printf( "tarVal = %f, ", tarVal);
//	printf( "currVal = %f, ", currVal);
//	printf( "gsLastError = %f, ", gsLastErr);
//	printf( "curr_error = %f, ", gsCurrError);
//	printf( "gsCurrCtrl = %f, ", gsCurrCtrl);
//	printf( "ctrl_out = %f\n"
//			"----------------------------\n", ctrl_out);

    return  ctrl_out ;
}


// -----------------------------------------------------------------------------
//! @brief delay for a given nanoseconds
//! @param  _ns nano seconds
// -----------------------------------------------------------------------------
void delay (uint32 _ns)
{
	// 120ns for NOP and ADD
	uint16 end = _ns/120;
	uint16 it = 1;
	do{
		it++;
	} while(it < end);
}

int
main()
{
//	printf( "main started \n");


	// initialize adc and pwm
	adc_init(adc_throttle);
	adc_init(adc_pedal);

	pwm_init();

	double pid_za = gc_pid_p;
	double pid_zb = gc_pid_p - gc_pid_i*gc_t_reg;

	while(1)
	{
		delay(5e5); // regulation time 0.5ms

		// measure pedal position
		double pedal_pos =
				adc_convt(adc_get_data(adc_pedal),gc_adc_app_res, gc_adc_app_vref);
		// measure throttle plate position
		double throttle_pos =
			    adc_convt(adc_get_data(adc_throttle), gc_adc_throttle_res, gc_adc_throttle_vref);


		double pwm_duty = pid_calc(pedal_pos, throttle_pos, pid_za, pid_zb);
		pwm_send_duty(pwm_duty);
//		pwm_send_duty (pid_ctrl(adc_analog_read()));
	}

  // simexit( 0 );

  return  0;
}	/* main() */





