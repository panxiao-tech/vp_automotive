/**
 * @file    etc_demo/include/sca_adc.h
 * @brief   PWM analog part implemented in SystemC-AMS
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    05-05-2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */



#ifndef ETCDEMO_SCA_PWM_H 
#define ETCDEMO_SCA_PWM_H


#include <systemc-ams>
#include "config_sim.h"
#include "sc_logger.h"

namespace  scvp
{
    
    // ----------------------------------------------------------------------------
    /// @brief analog part of PWM module implemented in SCA-TDF
    // ----------------------------------------------------------------------------
    SCA_TDF_MODULE(sca_pwm)
    {
        sca_tdf::sc_in<double>   pin_duty;
        sca_tdf::sca_out<double> pout_pwm;
        
        sca_pwm(sc_core::sc_module_name nm, double v0_ = 0.0, double v1_ = 1.0,
                const sca_core::sca_time& t_period_ = sca_core::sca_time(255.0 * 0.05, sc_core::SC_MS),
                const sca_core::sca_time& t_ramp_ = sca_core::sca_time(0.05, sc_core::SC_MS),
                const sca_core::sca_time& _t_step =sca_core::sca_time(0.05,sc_core::SC_MS)       )
        : pin_duty("pin_duty"),
        pout_pwm("pout_pwm"), v0(v0_), v1(v1_), v0_set(v0_), v1_set(v1_),
        t_period(t_period_.to_seconds()),
        t_ramp(t_ramp_.to_seconds()),
        t_duty_max(t_period - 2.0 * t_ramp),
        t_duty(t_duty_max),
        t_step(_t_step)
        {
            
        }
        
        void set_attributes()
        {
            does_attribute_changes();
            accept_attribute_changes();
        }
        
        void initialize()
        {

            ts_last  = get_timestep();
            ts_cnt   = 0;
            tsch_cnt = 0;
            t_res    = sc_core::sc_get_time_resolution().to_seconds();
        }
        
        void change_attributes()
        {
            double t = this->get_time().to_seconds(); // current time
            double t_pos = std::fmod(t, t_period);    // time position inside pulse period
            
            double td_req;
            
            if (t_pos < t_ramp) 
            {
                // rising edge
                td_req=t_ramp - t_pos;
            } 
            else if (t_pos < t_ramp + t_duty ) 
            {
                // plateau
                td_req= (t_ramp + t_duty) - t_pos;
            } 
            else if (t_pos < t_ramp + t_duty + t_ramp) 
            {
                // falling edge
                td_req=(t_ramp + t_duty + t_ramp) - t_pos;
            } 
            else 
            {
                // return to initial value
                td_req = t_period - t_pos;
            }
            //        std::cout << get_time() << " request new activation: " << td_req << std::endl;
            //        std::cout << get_time() << " sc_core::sc_get_time_resolution().to_seconds(): " << sc_core::sc_get_time_resolution().to_seconds() << std::endl;
#if defined(ENABLE_DTDF) 
            if(td_req > t_res)
            {
                request_next_activation(td_req, sc_core::SC_SEC);
            }
            else
            {
                request_next_activation(t_step);
            }
#else
            request_next_activation(t_step);
#endif
        }
        
        void processing()
        {
            double t = this->get_time().to_seconds(); // current time
            double t_pos = fmod(t, t_period);         // time position inside pulse period
            
            
            // calculate and clamp duty time
            t_duty = pin_duty.read();
            if(t_duty < 0) // negative voltage output
            {
                v0 = -v0_set;
                v1 = -v1_set;
                t_duty = fabs(t_duty);
            }
            else
            {
                v0 = v0_set;
                v1 = v1_set;
            }
            
            t_duty = t_duty * t_duty_max;
            if (t_duty < 0.0) t_duty = 0.0;
            if (t_duty > t_duty_max) t_duty = t_duty_max;
            
            double val = v0; // initial value
            if (t_pos < t_ramp)
            {
                // rising edge
                val = ((v1 - v0) / t_ramp) * t_pos + v0;
            }
            else if (t_pos < t_ramp + t_duty )
            {
                // plateau
                val = v1;
            }
            else if (t_pos < t_ramp + t_duty + t_ramp)
            {
                // falling edge
                val = ((v0 - v1) / t_ramp) * (t_pos - t_ramp - t_duty) + v1;
            }
            else
            {
                // return to initial value
            }
            pout_pwm.write(val);
            
            ts_cnt++;
            if(ts_last!=get_timestep())
            {
                tsch_cnt++;
                ts_last=get_timestep();
            }
        }
        
        void end_of_simulation()
        {
            std::ostringstream str;
            str << "Number of timestep changes: " << tsch_cnt << " of " << ts_cnt << " timesteps";
            SC_REPORT_INFO("statistic",str.str().c_str());
        }
        
    private:
        double v0, v1;            // initial and plateau values
        double v0_set, v1_set;    // set-up values
        double t_period, t_ramp;  // pulse period and ramp time
        double t_duty_max;        // maximum duty time
        double t_duty;            // current duty time
        double t_res; // system time resolution
        
        sca_core::sca_time ts_last;
        sca_core::sca_time t_step;
        
        unsigned long tsch_cnt;
        unsigned long ts_cnt;
        
    };
    
} // namespace scvp
#endif // ETCDEMO_SCA_PWM_H
// EOF
