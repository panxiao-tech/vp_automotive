/**
 * @file    etc_demo/include/sca_pedalpos_sensor.h
 * @brief   Accelerator Pedal Position Sensor implemented in SystemC-AMS
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    16-06-2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */



#ifndef ETCDEMO_PEDALPOS_SENSOR_H
#define ETCDEMO_PEDALPOS_SENSOR_H 


#include <systemc-ams>
#include "config_sim.h"
#include "sc_logger.h"

namespace  scvp
{
    
    // ----------------------------------------------------------------------------
    /// @brief Accelerator Pedal Position Sensor module that generate out signals
    /// to simulate the behavior of the senosr.
    // ----------------------------------------------------------------------------
    SCA_TDF_MODULE(sca_pedalpos_sensor)
    {
    public:
        // Accelerator Pedal Position to Throttle Position (angle in rad)
        sca_tdf::sca_out<double>     pout;
        
        /// @brief Custom constructor ofsca_pedalpos_sensor
        /// @param nm module name
        /// @param _sig_vec stimuli output signal format. The vect must not be ZERO
        /// in size and size must be EVEN. The 1st element is the initial time, the
        /// 2nd element is the first output valute. 3rd is the next change time, and
        /// so on. The module is implemented using Dynamic TDF.
        ///
        sca_pedalpos_sensor(sc_core::sc_module_name nm, std::vector<double> _sig_vec)
        :pout("pout"), sig_vec(_sig_vec), sig_pos(0)
        {
            sig_size = sig_vec.size();
            if (sig_size == 0 )
            {
                SC_REPORT_ERROR( "ERROR","Input vector size cannot be ZERO!");
            }
            else if(sig_size % 2 != 0 )
            {
                SC_REPORT_ERROR( "ERROR","ERROR: Input vector size must be  EVEN!");
            }
        }
        
        
    private:
        // ---------------------------------------------------------------------
        // set_attributes()
        // ---------------------------------------------------------------------
        void set_attributes()
        {
            does_attribute_changes();
            accept_attribute_changes();
        }
        
        // ---------------------------------------------------------------------
        // change_attributes()
        // ---------------------------------------------------------------------
        void initialize()
        {
            t_resolution = sc_core::sc_get_time_resolution().to_seconds();
        }
        
        
        // ---------------------------------------------------------------------
        // change_attributes()
        // ---------------------------------------------------------------------
        void change_attributes()
        {
            double  t_now = this->get_time().to_seconds(); 
            // overflow the maximum time generation of signal
            if(t_now >= sig_vec[sig_size - 2])
            {
                return;
            }
            
            double t_sig_next = sig_vec[sig_pos+2];
            double t_rest = t_sig_next - t_now;
            if(t_rest <= 0 )
            {
                sig_pos += 2;
                t_sig_next = sig_vec[sig_pos+2];
                t_rest = t_sig_next - t_now;
            }
            
            if(t_rest > t_resolution)
            {
                request_next_activation(t_rest, sc_core::SC_SEC);
            }
            else
            {
                request_next_activation(t_resolution, sc_core::SC_SEC);
            }
        }
        
        
        // ---------------------------------------------------------------------
        // processing()
        // ---------------------------------------------------------------------
        void processing()
        {
            pout.write(sig_vec[sig_pos+1]);
        }
        
        std::vector<double> sig_vec;
        unsigned int sig_size;
        unsigned int sig_pos;
        double t_resolution; // system time resolution
    };
    
} // namespace scvp
#endif // ETCDEMO_PEDALPOS_SENSOR_H 
