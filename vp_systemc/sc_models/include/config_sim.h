/**
 * @file    etc_demo/include/comfig_sims.h
 * @brief   Simulation configuration file
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    01.02.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */

#ifndef ETCDEMO_CONFIG_SIM_H
#define ETCDEMO_CONFIG_SIM_H

#include <iostream>
#include <iomanip>
#include <systemc-ams>
#include "../../or1k_iss_firmware/include/hw_def.h"


namespace scvp
{
    // -----------------------------------------------------------------------------
    // Debug and logging configurations
    // -----------------------------------------------------------------------------
#define DEBUG
#define LOG_LEVEL     VERBOSE_LEVEL // option levels: NO_LOG, ERROR_LEVEL, INFO_LEVEL,  DEBUG_LEVEL, VERBOSE_LEVEL
#define LOG_FILE_PATH "log.txt"
    
    // -----------------------------------------------------------------------------
    // Model and MoC configurations
    // -----------------------------------------------------------------------------
    // #define ENABLE_ISS    // Apply ECU model with Or1ksim ISS
#define ENABLE_DTDF   // Enable SCA dynamic-tdf simulation
    
    // -----------------------------------------------------------------------------
    // Timming and Settings in the models and simulation
    // -----------------------------------------------------------------------------
    const std::string  k_trace_file = "vcd";  // set trace file format either "vcd" or "csv"
    const double k_tsim_in_sec     = 3;       // total simulation time in seconds
    const double k_tstep_in_sec    = 3e-6;    // TDF module time step in seconds
    const double k_quantum_in_us   = 500;     // global quantum of temporal decoupling in us
    const double k_ins_delay_in_ns = 1;       // instruction delay in ns
    const double k_bus_delay_in_ns = 1;       // bus transaction delay in ns
    
    // -----------------------------------------------------------------------------
    // Accelerator Pedal Position Sensor positions
    // 1st column is time point, 2nd is value
    // -----------------------------------------------------------------------------
    const std::vector<double> k_pedalpos_ref
    {
        0.0, 0.3,
        0.3, 0.7,
        0.6, 1.2,
        0.9, 0.3,
        1.2, 0.7,
        1.5, 0.3,
        1.8, 0.6,
        2.1, 0.0
    };
    
    // -----------------------------------------------------------------------------
    //  PID controller parameters
    // -----------------------------------------------------------------------------
    const double k_t_regu_in_sec = 0.5e-3; 	// regulation time in second
    const double k_pid_p    = 0.8;          // PID controller proportional gain
    const double k_pid_i    = 0.4;          // PID controller integral gain
    const double k_pid_comp = 0.11;         // compensator factor of pre-loaded torque
    
    // -----------------------------------------------------------------------------
    // PWM parameters
    // -----------------------------------------------------------------------------
    const double k_pwm_v0 = 0.0;
    const double k_pwm_v1 = 12.0;
    const unsigned int k_pwm_steps = 255;
    const double k_pwm_t_step   = k_tstep_in_sec;            // PWM step time
    const double k_pwm_t_period = k_pwm_steps * k_pwm_t_step;// PWM period in second
    const double k_pwm_t_ramp   = 1 * k_pwm_t_step;          // PWM ramp time in second
    
    // -----------------------------------------------------------------------------
    // ADC(pedal position sensor) parameters
    // -----------------------------------------------------------------------------
    const double k_adc_app_vref = 3.0; // adc reference voltage
    const int    k_adc_app_res  = 12;  // resolution
    const int    k_adc_app_rate = 1;   // TDF port rate
    
    // -----------------------------------------------------------------------------
    // ADC(throttle position sensor) parametersk_t_regu_in_sec
    // -----------------------------------------------------------------------------
    const double k_adc_throttle_vref = 3.0; // adc reference voltage
    const int    k_adc_throttle_res  = 12;  // resolution
    const int    k_adc_throttle_rate = 1;   // TDF port rate
    
    // -----------------------------------------------------------------------------
    // Throttle body model parameters
    // -----------------------------------------------------------------------------
    const double k_dcmotor_Jeq = 0.0021;
    const double k_dcmotor_Beq = 0.0088;
    const double k_dcmotor_K   = 0.383;
    const double k_dcmotor_R   = 1.5;
    const double k_dcmotor_L   = 1.5e-3;
    const double k_dcmotor_Ks  = 0.087;
    const double k_dcmotor_Tpl = 0.396;
    const double k_dcmotor_Tf  = 0.284;
    
   
} //namspace scvp

#endif  //ETCDEMO_CONFIG_SIM_H
// eof
