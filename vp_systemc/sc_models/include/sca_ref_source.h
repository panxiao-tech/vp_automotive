/**
 * @file    etc_demo/include/sca_ref_source.h
 * @brief   provides reference source models such as step/gnd
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    05-05-2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */



#ifndef ETCDEMO_SCA_REF_SOURCE_H 
#define ETCDEMO_SCA_REF_SOURCE_H

#include <systemc-ams>
#include "config_sim.h"
#include "sc_logger.h"

namespace scvp
{

SCA_TDF_MODULE(sca_ref_step)
{
public:
    sca_tdf::sc_out<double> pout;
    
private:
    void set_attributes()
    {
        pout.set_timestep(t_step);
    }
    
    void processing()
    {
        pout.write(value);
    }
    
    double value;
    sca_core::sca_time t_step;
    
public:
    sca_ref_step( sc_core::sc_module_name nm,
            double value_= 1,
            sca_core::sca_time t_step_ =sca_core::sca_time(0.05,sc_core::SC_MS))
    : pout("pout"),
    value(value_),
    t_step(t_step_)
    {}
};


// ---------------------------------------------------------------------
/// Analogue source GND
// ---------------------------------------------------------------------
SCA_TDF_MODULE(sca_ref_gnd) {
public:
	/// in/output ports
	 sca_tdf::sca_out<double>  pout_gnd;

	/// @brief Custom constructor ofsca_ref_gnd
	sca_ref_gnd(sc_core::sc_module_name nm,
			sca_core::sca_time t_step_ =sca_core::sca_time(0.05,sc_core::SC_MS))
	 :pout_gnd("pout_gnd"), t_step(t_step_){}

private:
	sca_core::sca_time t_step;

    void set_attributes()
    {
    	accept_attribute_changes();
    	pout_gnd.set_timestep(t_step);
    }

    void processing()
    {
    	pout_gnd.write(0.0);
    }
};


// ---------------------------------------------------------------------
/// Analogue source
// ---------------------------------------------------------------------
SCA_TDF_MODULE(sca_ref_sawtooth) {
    
public:
    sca_tdf::sc_out<double> pout;

    sca_ref_sawtooth(sc_core::sc_module_name nm, double  _vref=0.5, int _count =100)
    {
        step = _vref/_count;
        count = _count;
        out_temp = 0;
        i =0;
        // cout<< "sca_ref_sawtooth : step = " << step <<endl;
    }
    
    double step;
    double out_temp;
    int count;
    int i;
    
    void processing()
    {
        out_temp += step;
        i++;
        if (i == count)
        {
            i=0;
            out_temp =0.0;
        }
        pout.write(out_temp);
    }
    
    
};
} // namespace scvp
#endif // ETCDEMO_SCA_REF_SOURCE_H

// EOF
