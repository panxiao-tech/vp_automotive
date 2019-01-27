/**
 * @file adc.h
 * @author Xiao Pan <pan@cs.uni-kl.de>,
 *         AG Design of Cyber-Physical Systems
 * @date  04.05.2018
 * @brief ADC Driver head file
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


#ifndef _prog32_adc_h_
#define _prog32_adc_h_


// ADC unitily functions
// initialize adc
extern void adc_init(int adc_module);

// read data (2bytes/16bits) from adc data registers
extern int16  adc_get_data(int adc_module);

// convert the readout to analog data
extern double    adc_convt(int16 readout,
		unsigned int res,
		double refvol);


// seletion of ADC module
enum adc_module{
    adc_throttle,
    adc_pedal
};

#endif
