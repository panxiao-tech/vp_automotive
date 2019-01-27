/**
 * @file    etc_demo/include/sc_tlm_adc.h
 * @brief   SystemC-TLM model of ADC incl. ADC analog part impl. in SCA
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    05.05.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */


#ifndef ETCDEMO_TLM_ADC_H 
#define ETCDEMO_TLM_ADC_H

#include <iostream>
#include "tlm.h"
#include "tlm_utils/simple_target_socket.h"
#include "config_sim.h"

#include "sca_adc.h"
#include "sc_logger.h"


namespace scvp
{
    
    // ----------------------------------------------------------------------------
    /// @brief sc_tlm_adc digitial interface (SystemC-TLM module)
    
    /// @note For simplicity, this sc_tlm_adc always updates its measurement result and
    /// send the latest data to ECU by reading the internal register 'datareg'
    ///
    // ----------------------------------------------------------------------------
    class sc_tlm_adc : public sc_core::sc_module
    {
    public:
        // sc_tlm_adc interface
        sca_tdf::sca_in<double>  *pin_pos;
        sca_tdf::sca_in<double>  *pin_neg;
        
        /// @brief Custom constructor for sc_tlm_adc digital part
        /// @param name             The SystemC_AMS module name
        /// @param _res             sc_tlm_adc resolution
        sc_tlm_adc( sc_core::sc_module_name  nm,
                   double _vref = 16.0,
                   int _res = 12,
                   int _rate = 1,
                   sca_core::sca_time _t_step =sca_core::sca_time(0.05,sc_core::SC_MS)) ;
        ~sc_tlm_adc();
        
        /// The TLM target socket to receive bus traffic.
        tlm_utils::simple_target_socket<sc_tlm_adc>  i_bus;
        
    private:
        
        
        // The blocking transport routine for the port
        void  bus_read_write( tlm::tlm_generic_payload &trans,
                             sc_core::sc_time         &delay );
        unsigned char bus_read (unsigned char addr);
        void bus_write (unsigned char addr, unsigned char wdata);
        
        // sc_tlm_adc internal registers
        struct
        {
            unsigned char	ctrl;   // sc_tlm_adc control register
            unsigned char	datal;  // sc_tlm_adc data register low byte
            unsigned char	datah;  // sc_tlm_adc data register high byte
        } regs;
        
        
        // analog ADCs
        sca_adc i_adc_ana;
        
    
        void method_meas_update();
        
        /// SystemC signal to get adc_ana (analogue part) sampled output.
        sc_core::sc_signal<unsigned int> sig_adc_ana_out;
        
    };
} //namespace scvp

#endif

