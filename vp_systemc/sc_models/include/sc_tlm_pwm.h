/**
 * @file    etc_demo/include/sc_tlm_pwm.h
 * @brief   PWM unit model incl. analog part implemented in SCA
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date  05.05.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */


#ifndef ETCDEMO_TLM_PWM_H 
#define ETCDEMO_TLM_PWM_H



#include <iostream>
#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"
#include "config_sim.h"


#include "sca_pwm.h"
#include "sc_logger.h"


namespace scvp
{
    // ----------------------------------------------------------------------------
    //! @brief Class of sc_tlm_pwm digital part (SystemC-TLM)
    
    // ----------------------------------------------------------------------------
    class sc_tlm_pwm
    : public sc_core::sc_module
    {
    public:
        
        // sc_tlm_pwm interface
        sca_tdf::sca_out<double> *pout_pwm;
        
        //! @brief Custom constructor for sc_tlm_pwm digital part
        //! @param name             The SystemC_AMS module name
        //! @param v0_              Initial value
        //! @param v1_              Plateau value
        //! @param t_period_        Pulse period
        //! @param t_ramp_          Ramp time
        sc_tlm_pwm( sc_core::sc_module_name  nm,
                   double v0_ = 0.0,
                   double v1_ = 1.0,
                   const sca_core::sca_time& t_period_ = sca_core::sca_time(255.0 * 0.05, sc_core::SC_MS),
                   const sca_core::sca_time& t_ramp_ = sca_core::sca_time(0.05, sc_core::SC_MS)) ;
        
        //! The TLM target socket to receive bus traffic.
        tlm_utils::simple_target_socket<sc_tlm_pwm>  i_bus;
        
    private:
        // The blocking transport routine for the port
        void  bus_read_write( tlm::tlm_generic_payload &trans,
                             sc_core::sc_time         &delay );
        unsigned char bus_read (unsigned char addr);
        void bus_write (unsigned char addr, unsigned char wdata);
        
        // internal registers
        struct
        {
            unsigned char    ctrl;   // sc_tlm_pwm control register
            unsigned char    dutyh;   // register stores duty cycle data
            unsigned char    dutyl;   // register stores duty cycle data
        } regs;
        
        sca_pwm i_pwm_ana;
        sc_core::sc_signal<double> sig_pwm_ana_in;
        
        void update_dutycycle();
    };
}  // namespace scvp

#endif // ETCDEMO_TLM_PWM_H

//eof
