/**
 * @file    etc_demo/include/sca_adc.h
 * @brief   ADC analog part implemented in SystemC-AMS
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    16-04-2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */


#ifndef ETCDEMO_SCA_ADC_H 
#define ETCDEMO_SCA_ADC_H


#include <systemc-ams>
#include "config_sim.h"
#include "sc_logger.h"


namespace scvp
{
    
    SCA_TDF_MODULE(sca_adc)
    {
    public:
        /*
         * in/out ports
         */
        sca_tdf::sca_in<double>   pin_pos;          // positive voltage input
        sca_tdf::sca_in<double>   pin_neg;          // negative voltage input
        sca_tdf::sc_out<unsigned int> pout_conv;    // converted digital output
        
        /// @brief Custom constructor of ADC (analog part)
        /// @para vref   ADC reference voltage, default is set to 1 [volt]
        /// @para res    ADC resolution, default is set to 12 [bits]
        /// @para fs     Sampling rate, default is set to 100M [Hz]
        /// @para _rate  Output port rate, default is set to 1
        /// @para _t_step  Time step, default is set to 50us
        sca_adc(sc_core::sc_module_name nm,
                double _vref = 1.0,
                unsigned int _res = 12,
                int _rate = 1,
                sca_core::sca_time _t_step =sca_core::sca_time(0.05,sc_core::SC_MS)
                );
        
        /// @brief Public interface to set/change ADC resolution, acceptable values: 9/10/11/12 (bits)
        /// @para _res  ADC resolution in bits
        /// @return true if set resolution successfully (current version always true)
        bool set_res(unsigned int _res);
        
        /// @brief Public interface to set/change ADC sampling rate
        /// @para _fs Updated ADC resolution in bits
        /// @return true if set resolution successfully (current version always true)
        bool set_fs(double _fs);
        
        
    private:
        void initialize();
        void processing();
        void set_attributes();
        
        /// @brief return maximum converted value of ADC
        /// @para _value
        /// @return
        int16_t max_value(int16_t _value);
        
        // variables for internal calculation
        long    adc_max;
        double  adc_lsb;
        double  analog;
        int16_t erg;
        int16_t bv_erg;
        
        // module parameters
        double       p_vref;
        unsigned int p_res;
        int          p_rate;
        sca_core::sca_time t_step;
    };
} // namespace scvp

#endif // ETCDEMO_SCA_ADC_H

//eof
