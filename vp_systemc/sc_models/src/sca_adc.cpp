/**
 * @file    etc_demo/src/sc_tlm_adc.cpp
 * @brief   SystemC-TLM model of ADC incl. ADC analog part impl. in SCA
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    05.05.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */

#include "sca_adc.h"


namespace scvp
{
    /*
     * constructor
     */
    sca_adc::sca_adc(sc_core::sc_module_name nm,
                     double _vref,
                     unsigned int _res,
                     int _rate,
                     sca_core::sca_time _t_step)
    {
        p_vref = _vref;
        p_res  = _res;
        p_rate = _rate;
        t_step = _t_step;
    }

    /*
     * initiatlize()
     */
    void  sca_adc::initialize()
    {
        adc_max = (long)(pow(2.0, p_res ));
        adc_lsb = p_vref / (adc_max);
    }
    
    
    /*
     * set_attributes()
     */
    void sca_adc::set_attributes()
    {
        accept_attribute_changes();
        pin_pos.set_timestep(t_step);
    }
    
    
    /*
     * processing()
     */
    void sca_adc::processing()
    {
        analog = pin_pos.read() - pin_neg.read();  //  read analog input
        erg = lround(analog / adc_lsb); // calculate digital value and rounding
        erg = max_value (erg);     		// output limitation
        bv_erg = erg; 					// save as bitvector
        pout_conv.write(bv_erg);
        
        LOG_VERBOSE("processing... analog readin = %f, converted value = 0x%04X. ",analog, bv_erg );
    }
    
    
    bool sca_adc::set_res(unsigned int _res)
    {
        LOG_INFO("Set ADC's resolution Rate to %d bits. ", _res);

        p_res = _res;
        adc_max = (long)(pow(2.0, p_res ));
        adc_lsb = p_vref / (adc_max);
        return true;
    }
    
    
    bool sca_adc::set_fs(double _fs)
    {
        /* not implemented */
        LOG_INFO("Set ADC's Sampling Rate to %d. ", _fs);
        return true;
    }
    
    
    int16_t sca_adc::max_value(int16_t _value)
    {
        if(std::fabs(_value) > adc_max-1)
            _value = _value> 0 ? adc_max-1 : -adc_max;
        return _value;
    }
    
} // namespace scvp
// eof
