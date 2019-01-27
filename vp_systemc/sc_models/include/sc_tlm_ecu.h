/**
 * @file    etc_demo/include/sc_tlm_ecu.h
 * @brief   ECU behavior model in SystemC-TLM
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    01.02.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */



#ifndef ETCDEMO_TLM_ECU_H
#define ETCDEMO_TLM_ECU_H

#include <cstdint>
#include "tlm_utils/simple_initiator_socket.h"
#include "config_sim.h"
#include "sc_logger.h"

namespace scvp
{
    // ----------------------------------------------------------------------------
    //! @brief SystemC module class for the Microcontroller (BASE)
    // ----------------------------------------------------------------------------
    
    class sc_tlm_ecu
    : public  sc_core::sc_module
    {
    public:
        
#ifdef DEBUG
        sc_core::sc_out<double>   pout_pid_ctrl;
        sc_core::sc_out<double>   pout_pwm_duty;
#endif
        
        //! TLM initiator for data access
        tlm_utils::simple_initiator_socket<sc_tlm_ecu>  data_bus;
        
        
        //! Constructor
        sc_tlm_ecu(sc_core::sc_module_name  nm);
        
        //! Destructor
        ~sc_tlm_ecu();
        
    protected:
        
        sc_core::sc_time    instrDelay;		//!< instruction delay
        sc_core::sc_time    transDelay;		//!< TLM trans delay
        
        // this will be reimplement later
        virtual void program_main();
        
        //! The common thread to make the transport calls
        virtual void  do_trans (tlm::tlm_generic_payload &trans);
        
        
        // The transactional trans on the bus.
        tlm::tlm_generic_payload  trans;
        
        // functions support read and write to the bus
        std::uint8_t   bus_read (unsigned long int  addr);
        void bus_write (unsigned long int  addr, std::uint8_t wdata);
        
        
        // ADC unitily functions
        void adc_init(int adc_module);
        int16_t  adc_get_data(int adc_module);
        double  adc_convt(int16_t readout, unsigned int res, double refvol);
        
        // PWM unitily functions
        void pwm_init();
        void pwm_send_duty(double &duty);
        unsigned int pwm_get_duty();
        
        // PID control function
        double pid_calc(const double &val_target,
                        const double &val_curr,
                        const double &pid_za,
                        const double &pid_zb);
        
        // seletion of ADC module
        enum adc_module
        {
            adc_throttle,
            adc_pedal
        };        
    };	/* sc_tlm_ecu() */
    
} // namespace scvp
#endif //ETCDEMO_TLM_ECU_H

//eof
