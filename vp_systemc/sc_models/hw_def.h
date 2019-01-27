/**
 *
 * @file    config_hw.h
 * @author  Xiao Pan (pan@cs.uni-kl.de)
 * @date    05.05.2018
 * @section LICENSE License (ADD YOUR LICENSE HERE)
 *
 * @section DESCRIPTION Macros defining the address map of memories and peripherals, 
 *                      offsets for peripheral registers, and bits of interestes. 
 */




#ifndef _config_hw_h_
#define _config_hw_h_


// -----------------------------------------------------------------------------
// Base address of memories and peripherals.
// -----------------------------------------------------------------------------
#define ADC_THROTTLE_BASE   0x91001000   //!< throttle ADC base address
#define PWM_BASE            0x91002000   //!< PWM base address
#define ADC_PEDAL_BASE      0x91003000   //!< pedal ADC base address


// -----------------------------------------------------------------------------
// ADC hardware 
// -----------------------------------------------------------------------------
//! Mask for addresses (6 bit bus)
#define ADC_ADDR_MASK  63
// Offsets for the ADC registers
#define	ADCCTRL		10 //!< R/W: ADC Control Register
#define ADCDATAH	14  //!< R: 	 ADC Data register High Byte
#define ADCDATAL	15  //!< R: 	 ADC Data register Low Byte


// -----------------------------------------------------------------------------
// PWM hardware 
// -----------------------------------------------------------------------------
//! Mask for addresses (6 bit bus)
#define PWM_ADDR_MASK  63
// Offsets for the PWM registers
#define	PWMCTRL		10 //!< R/W: PWM Control Register
#define	PWMDUTYH	11 //!< R/W: PWM Data register High Byte of duty cycle
#define	PWMDUTYL	12 //!< R/W: PMW Data register High Byte of duty cycle
// PWM Control/configuration Register bits of interest
#define PWM_CTRL_UDC 0x20  	//!< update duty cycle bit in ctrl register




#endif
