/**
 * @file    etc_demo/include/sc_top.h
 * @brief   top level model of ETC demo
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    16-04-2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */



#ifndef ETCDEMO_SC_TOP_H 
#define ETCDEMO_SC_TOP_H

#include<memory>
#include "sca_ref_source.h"
#include "sc_tlm_ecu.h"
#include "sc_tlm_adc.h"
#include "sc_tlm_bus.h"
#include "sc_tlm_pwm.h"
#include "sca_throttle_motor.h"
#include "sca_pedalpos_sensor.h"
#ifdef ENABLE_ISS
#include "sc_tlm_ecu_iss.h"
#endif //ENABLE_ISS
#include "sc_logger.h"


namespace scvp
{
    // -----------------------------------------------------------------------------
    /// @brief drain  for binding SCA signals
    // -----------------------------------------------------------------------------
    SCA_TDF_MODULE(sca_tdf_drain)
    {
        sca_tdf::sca_in<double> pin_i;
        
        sca_tdf_drain(sc_core::sc_module_name nm){}
        
        void set_attributes()
        {
            accept_attribute_changes();
        }
        
        void processing(){}
    };
    
    
    // -----------------------------------------------------------------------------
    /// @brief top level module
    // -----------------------------------------------------------------------------
    SC_MODULE(sc_top)
    {
    private:
        /*
         * model instances
         */
        std::shared_ptr<sca_tdf_drain>       i_drain;
        std::shared_ptr<sca_pedalpos_sensor> i_pedal_sensor;
        std::shared_ptr<sca_ref_gnd>         i_ref_gnd;
        std::shared_ptr<sc_tlm_adc>          i_adc_throttle_pos;
        std::shared_ptr<sc_tlm_adc>          i_adc_pedal_pos;
        std::shared_ptr<sc_tlm_pwm>          i_pwm;
        std::shared_ptr<sc_tlm_bus<3>>       i_bus;
        std::shared_ptr<sca_throttle_body>   i_throttle_body;
#ifdef ENABLE_ISS
        std::shared_ptr<sc_tlm_ecu_iss>      i_ecu;
#else 
        std::shared_ptr<sc_tlm_ecu>          i_ecu;
#endif // ENABLE_ISS
        
        /*
         * signals
         */
        sca_tdf::sca_signal<double> sig_pwm_pout;
        sca_tdf::sca_signal<double> sig_throttle_body_pout_a;
        sca_tdf::sca_signal<double> sig_throttle_body_pout_i;
        sca_tdf::sca_signal<double> sig_gnd_throttle;
        sca_tdf::sca_signal<double> sig_gnd_padel;
        sca_tdf::sca_signal<double> sig_pedal_sensor;
        sc_core::sc_signal<double>  sig_ref_pout;
#ifdef DEBUG
        sc_core::sc_signal<double>  sig_ecu_pid_ctrl;
        sc_core::sc_signal<double>  sig_ecu_pwm_duty;
#endif
        
        /*
         * trace file
         */
        sca_util::sca_trace_file* trace_file;
        
    public:
        SC_CTOR(sc_top)
        {
            /*
             * drain for DC motor current sig output
             */
            i_drain = std::make_shared<sca_tdf_drain>("drain");
            i_drain ->pin_i(sig_throttle_body_pout_i);
            
            /*
             * Accelerator pedal sensor position source
             */
            i_pedal_sensor = std::make_shared<sca_pedalpos_sensor>("app_sensor", k_pedalpos_ref);
            i_pedal_sensor ->pout(sig_pedal_sensor);
            
            /*
             * GND
             */
            i_ref_gnd = std::make_shared<sca_ref_gnd>("gnd", sc_core::sc_time(k_tstep_in_sec, sc_core::SC_SEC ));
            i_ref_gnd ->pout_gnd(sig_gnd_throttle);
            
            
            /*
             * PWM
             */
            i_pwm = std::make_shared<sc_tlm_pwm>("pwm", k_pwm_v0, k_pwm_v1, sc_core::sc_time(k_pwm_t_period, sc_core::SC_SEC ), sc_core::sc_time(k_pwm_t_ramp, sc_core::SC_SEC ));
            i_pwm ->pout_pwm->bind(sig_pwm_pout);
            
            
            /*
             * ADC module  for the pedal position sensor
             */
            i_adc_pedal_pos = std::make_shared<sc_tlm_adc>("adc_pedal_pos", k_adc_app_vref, k_adc_app_res, k_adc_app_rate, sc_core::sc_time(k_tstep_in_sec, sc_core::SC_SEC ));
            i_adc_pedal_pos->pin_pos->bind(sig_pedal_sensor);
            i_adc_pedal_pos->pin_neg->bind(sig_gnd_throttle);
            
            /*
             * ADC for the throttle position
             */
            i_adc_throttle_pos = std::make_shared<sc_tlm_adc>("adc_throttle_pos",  k_adc_throttle_vref, k_adc_throttle_res, k_adc_throttle_rate, sc_core::sc_time(k_tstep_in_sec, sc_core::SC_SEC ));
            i_adc_throttle_pos ->pin_pos->bind(sig_throttle_body_pout_a);
            i_adc_throttle_pos ->pin_neg->bind(sig_gnd_throttle);
            
            /*
             * BUS module  module
             */
            i_bus = std::make_shared<sc_tlm_bus<3>>("bus");
            i_bus ->initiator_socket[0]->bind( i_adc_throttle_pos->i_bus );
            i_bus ->initiator_socket[1]->bind( i_pwm->i_bus );
            i_bus ->initiator_socket[2]->bind( i_adc_pedal_pos->i_bus );
            
            /*
             * throttle body module
             */
            i_throttle_body = std::make_shared<sca_throttle_body>("throttle_motor");
            i_throttle_body ->pin_v(sig_pwm_pout);
            i_throttle_body ->pout_a(sig_throttle_body_pout_a);
            i_throttle_body ->pout_i(sig_throttle_body_pout_i);
            
            /*
             * ECU (ISS or Behavior model)
             */
#ifdef ENABLE_ISS
            i_ecu = std::make_shared<sc_tlm_ecu_iss>("ecu");
#else // ENABLE_ behavior
            i_ecu = std::make_shared<sc_tlm_ecu>("ecu");
#ifdef DEBUG
            i_ecu ->pout_pid_ctrl(sig_ecu_pid_ctrl);
            i_ecu ->pout_pwm_duty(sig_ecu_pwm_duty);
#endif // EBUG
#endif // ENABLE_ISS
            i_ecu ->data_bus.bind(i_bus->data_bus);
            
            /*
             * trace file
             */
            if(!k_trace_file.compare("vcd"))
            {
                trace_file = sca_util::sca_create_vcd_trace_file("tr_ecu_demo");
            }
            else
            {
                trace_file = sca_util::sca_create_tabular_trace_file("tr_ecu_demo");
            }
#ifdef DEBUG
            sca_util::sca_trace(trace_file, sig_pedal_sensor, "ref_pout");
            sca_util::sca_trace(trace_file, sig_throttle_body_pout_a, "dcmotor_pout_a");
            sca_util::sca_trace(trace_file, sig_throttle_body_pout_i, "dcmotor_pout_i");
            sca_util::sca_trace(trace_file, sig_ecu_pid_ctrl, "ecu_pid_ctrl");
            sca_util::sca_trace(trace_file, sig_pwm_pout,     "pwm_pout");
            sca_util::sca_trace(trace_file, sig_ecu_pwm_duty, "ecu_pwm_duty");
#else
            sca_util::sca_trace(trace_file, sig_pedal_sensor,         "ref_pout");
            sca_util::sca_trace(trace_file, sig_throttle_body_pout_a, "motor_pout_a");
            sca_util::sca_trace(trace_file, sig_throttle_body_pout_i, "motor_pout_i");
#endif
        };
        
        ~sc_top()
        {
            if(!k_trace_file.compare("vcd"))
            {
                sca_util::sca_close_vcd_trace_file(trace_file);
            }
            else
            {
                sca_util::sca_close_tabular_trace_file(trace_file);
            }
        }
    };
    
} // namespace scvp

#endif  // ETCDEMO_SC_TOP_H
//eof 
