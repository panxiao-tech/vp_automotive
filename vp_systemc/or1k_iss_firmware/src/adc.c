/**
 * @file adc.c
 * @author Xiao Pan <pan@cs.uni-kl.de>,
 *         AG Design of Cyber-Physical Systems
 * @date  04.05.2018
 * @brief ADC Driver in or1k Firmware
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


#include <math.h>
#include <stdio.h>

#include "../../hw_def/config_hw.h"
#include "orsocdef.h"
#include "adc.h"


void adc_init(int adc_module)
{
	// debug print
//	printf( "ADC initilaziation. \n");

	switch (adc_module){
	case adc_throttle:
		REG8(ADC_THROTTLE_BASE + ADCCTRL) = 0x01;
		break;
	case adc_pedal:
		REG8(ADC_PEDAL_BASE + ADCCTRL) = 0x01;
		break;

	default:
		break;
	}
	return;
}

//
// read data (2bytes/16bits) from adc data registers
//
int16  adc_get_data(int adc_module)
{
	uint8 data_l = 0, data_h = 0;

	switch (adc_module) {
	        case adc_throttle:
	        	data_h = REG8(ADC_THROTTLE_BASE + ADCDATAH);
	        	data_l = REG8(ADC_THROTTLE_BASE + ADCDATAL);
	            break;

	        case adc_pedal:
	        	data_h = REG8(ADC_PEDAL_BASE + ADCDATAH);
	        	data_l = REG8(ADC_PEDAL_BASE + ADCDATAL);
	            break;

	        default:
	            break;
	    }


	int16 readout =0 ;
	readout = (data_h & 0xFF )| readout;
	readout = (data_l & 0xFF )| (readout<<8);

	// debug print
//	printf( "ADC readout digital = %x\n", readout);

	return readout;
}

// convert the readout to analog data
 double  adc_convt(int16 readout,
			unsigned int res,
			double refvol)
 {
	 double analog = readout * refvol/ pow(2, res);
//	printf( "ADC readout analog = %f\n", analog);
	 return analog;
 }
