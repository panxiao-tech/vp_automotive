/**
 * @file pwm.h
 * @author Xiao Pan <pan@cs.uni-kl.de>,
 *         AG Design of Cyber-Physical Systems
 * @date  04.05.2018
 * @brief PWM Driver head file
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


#ifndef _prog32_pwm_h_
#define _prog32_pwm_h_


// PWM unitily functions
extern   void pwm_init();
extern   void pwm_send_duty(double duty);
extern   unsigned int pwm_get_duty();

#endif
