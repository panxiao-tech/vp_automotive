/**
 * @file    etc_demo/src/sc_tlm_ecu.h
 * @brief   ECU behavior model in SystemC-TLM
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    01.02.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */


#include "sc_tlm_ecu.h"

namespace scvp
{
    SC_HAS_PROCESS( sc_tlm_ecu );
    
    // ----------------------------------------------------------------------------
    //! Constructor
    //! @param name             The SystemC module name
    // ----------------------------------------------------------------------------
    sc_tlm_ecu::sc_tlm_ecu( sc_core::sc_module_name  nm) :
    sc_module( nm ),
    data_bus ("bus_initiator" )
    {
        // NO delay in the basic module
        instrDelay = sc_core::sc_time(1, sc_core::SC_NS );
        transDelay = sc_core::sc_time(1, sc_core::SC_NS );
        
        // thread to run demo example
        SC_THREAD( program_main );
        
    }// sc_tlm_ecu()
    
    // ----------------------------------------------------------------------------
    //! Destructor
    // ----------------------------------------------------------------------------
    sc_tlm_ecu::~sc_tlm_ecu()
    {
    }
    
    // ----------------------------------------------------------------------------
    //! @brief Process a read on the bus
    
    //! @param addr   The address for the read
    //! @return  The read data
    // ----------------------------------------------------------------------------
    std::uint8_t  sc_tlm_ecu::bus_read (unsigned long int  addr)
    {
        int wdata;
        trans.set_read ();
        trans.set_address ((sc_dt::uint64) addr);
        trans.set_data_length( 4 );
        trans.set_streaming_width( 4 ); // = data_length to indicate no streaming
        trans.set_data_ptr( reinterpret_cast<std::uint8_t *>(&wdata) );
        trans.set_byte_enable_length (2);
        trans.set_byte_enable_ptr (0);
        
        trans.set_dmi_allowed( false ); // Mandatory initial value
        trans.set_response_status( tlm::TLM_INCOMPLETE_RESPONSE );
        
        // Transport.
        do_trans( trans );
        
        /* For now just simple non-zero return code on error */
        if (trans.is_response_ok ()==0)
        {
            LOG_ERROR("TLM Error: BUS read failed. ");
        }
        
        return wdata;
    }	/* bus_read() */
    
    
    // ----------------------------------------------------------------------------
    //! @brief Process a write on the bus
    
    //! @param addr   The address for the write
    //! @param wdata  Data to write
    // ----------------------------------------------------------------------------
    void sc_tlm_ecu::bus_write (unsigned long int  addr, std::uint8_t  wdata)
    {
        trans.set_write ();
        trans.set_address ((sc_dt::uint64) addr);
        trans.set_data_length( 4 );
        trans.set_streaming_width( 4 ); // = data_length to indicate no streaming
        trans.set_data_ptr( reinterpret_cast<std::uint8_t *>(&wdata) );
        trans.set_byte_enable_length (2);
        trans.set_byte_enable_ptr (0);
        
        trans.set_dmi_allowed( false ); // Mandatory initial value
        trans.set_response_status( tlm::TLM_INCOMPLETE_RESPONSE );
        
        
        // Transport.
        do_trans( trans );
        
        /* For now just simple non-zero return code on error */
        if (trans.is_response_ok ()==0)
        {
            LOG_ERROR("TLM Error: BUS write failed. ");
        }
    }	/* bus_write() */
    
    
    
    // ----------------------------------------------------------------------------
    //! @brief TLM transport to the target
    
    //! Calls the blocking transport routine for the initiator socket (@see
    //! ::dataBus). Passes in a dummy time delay of zero.
    
    //! @param trans  The transaction trans
    // ----------------------------------------------------------------------------
    void
    sc_tlm_ecu::do_trans( tlm::tlm_generic_payload &trans )
    {
        data_bus->b_transport( trans , transDelay );
    }	/* do_trans() */
    
    
    
    // ----------------------------------------------------------------------------
    //! @brief application program main PID controller 
    // ----------------------------------------------------------------------------
    void sc_tlm_ecu::program_main()
    {
        LOG_INFO("ECU program started.");

        // init ADC
        adc_init(adc_throttle);
        adc_init(adc_pedal);
        
        double pid_za = k_pid_p;
        double pid_zb = k_pid_p - k_pid_i * k_t_regu_in_sec;
        
        while (true)
        {
            // regulating rate
            wait(sc_core::sc_time(k_t_regu_in_sec, sc_core::SC_SEC ));
            
            double pedal_pos =
            adc_convt(adc_get_data(adc_pedal),k_adc_app_res, k_adc_app_vref);
            
            double throttle_pos =
            adc_convt(adc_get_data(adc_throttle), k_adc_throttle_res,
                      k_adc_throttle_vref);
    
            LOG_INFO("Current pedal postion = %f, throttle postion = %f",pedal_pos, throttle_pos);
            
            double pwm_duty_d = pid_calc(pedal_pos, throttle_pos, pid_za, pid_zb);

            LOG_INFO("Calauted new PWM dutycycle = %f", pwm_duty_d);

            pwm_send_duty(pwm_duty_d);
        }
        return;
    }/* demo_thread() */
    
    
    
    // ----------------------------------------------------------------------------
    //! @brief program to calcute PID output
    // ----------------------------------------------------------------------------
    static double       gsLastErr  = 0.0;
    static double       gsCurrErr  = 0.0;
    static double       gsLastCtrl = 0.0;
    static double       gsCurrCtrl = 0.0;
    //static unsigned int cnt_ctrl  = 0;
    
    
    double sc_tlm_ecu::pid_calc(const double &tarVal,
                                const double &currVal,
                                const double &pid_za,
                                const double &pid_zb)
    {
        gsLastErr  = gsCurrErr;
        gsLastCtrl = gsCurrCtrl;
        gsCurrErr  = tarVal - currVal;
        gsCurrCtrl = gsLastCtrl  - pid_zb * gsLastErr + pid_za * gsCurrErr;
        
        double ctrl_out = gsCurrCtrl + k_pid_comp;
        
        // saturation
        if(ctrl_out > 1)
        {
            ctrl_out = 1;
            gsCurrCtrl = 1- k_pid_comp;
        }
        if(ctrl_out < -1) {
            ctrl_out = -1;
            gsCurrCtrl = -1 - k_pid_comp;
        }
        
        
#ifdef DEBUG
        pout_pid_ctrl.write(ctrl_out *12);
#endif
        
        return  ctrl_out ;
    }
    
    
    // ====================  ADC unitily software  =================================
    // -----------------------------------------------------------------------------
    //! @brief Initialize ADC component. (ADC Driver)
    
    //! - Enable ADC.
    //! - Enable intrrupt (not implemented).
    // -----------------------------------------------------------------------------
    void sc_tlm_ecu::adc_init(int adc_module)
    {
        wait(instrDelay);	// instruction delay
        /* enable ADC,  set ctrl register 0x01 */
        switch (adc_module) {
            case adc_throttle:
                bus_write((sc_dt::uint64)(ADC_THROTTLE_BASE+ADCCTRL), 0x01);
                break;
            case adc_pedal:
                bus_write((sc_dt::uint64)(ADC_PEDAL_BASE+ADCCTRL), 0x01);
                break;
                
            default:
                break;
        }
        
        LOG_INFO("Initializing ADC module %d... done. ", adc_module);

        return;
    }/* adc_init () */
    
    
    // -----------------------------------------------------------------------------
    //! @brief Read conversion result from ADC
    
    //! @return  The converted digital data
    // -----------------------------------------------------------------------------
    int16_t sc_tlm_ecu::adc_get_data(int adc_module)
    {
        uint8_t data_l = 0, data_h = 0;
        
        switch (adc_module) {
            case adc_throttle:
                LOG_VERBOSE("Fetching measurement data from throttle postion ADC ...");
                wait(instrDelay);
                data_l = bus_read((sc_dt::uint64)(ADC_THROTTLE_BASE+ADCDATAL));
                wait(instrDelay);
                data_h = bus_read((sc_dt::uint64)(ADC_THROTTLE_BASE+ADCDATAH));
                break;
                
            case adc_pedal:
                LOG_VERBOSE("Fetching measurement data from pedal postion ADC ...");
                wait(instrDelay);
                data_l = bus_read((sc_dt::uint64)(ADC_PEDAL_BASE+ADCDATAL));
                wait(instrDelay);
                data_h = bus_read((sc_dt::uint64)(ADC_PEDAL_BASE+ADCDATAH));
                break;
                
            default:
                break;
        }
        
        int16_t readout =0 ;
        readout = (data_h & 0xFF )| readout;
        readout = (data_l & 0xFF )| (readout<<8);

        LOG_VERBOSE("Readout ADC data = 0x%04X", readout);

        return readout;
    } /* adc_get() */
    
    
    
    // -----------------------------------------------------------------------------
    //! @brief adc analog rea
    //! @param readout  readout data from ADC, unigned int
    //! @param res resultion of ADC
    //! @param refvol reference voltage
    
    //! @return  The actural analog valute of adc readout
    // -----------------------------------------------------------------------------
    double sc_tlm_ecu::adc_convt( int16_t readout,
                                 unsigned int res,
                                 double refvol)
    {

        double analog = readout * refvol/ pow(2, res);
        
        LOG_VERBOSE("Converting ADC readout value \'0x%04X\' to voltage ... %f ", readout, analog);
        
        return analog;
    } /* adc_get() */
    
    
    // ====================  PWM unitily software  =================================
    // -----------------------------------------------------------------------------
    //! @brief Initialize PWM component. (ADC Driver)
    
    //! - Enable ADC.
    // -----------------------------------------------------------------------------
    void sc_tlm_ecu::pwm_init()
    {
        // wait(instrDelay);	// instruction delay
        /* enable PWM,  set ctrl register 0x01 */
        // bus_write((sc_dt::uint64)(PWM_BASE+PWMCTRL), 0x01);
        
        LOG_INFO("Initializing PWM module ... done.");

        return;
    }/* pwm_init () */
    
    
    // -----------------------------------------------------------------------------
    //! @brief Send duty cycle data to PWM, data length = 1byte
    
    //! @param duty  duty cycle, unit = 1/1000
    // -----------------------------------------------------------------------------
    void sc_tlm_ecu::pwm_send_duty(double &duty)
    {
        
        unsigned long  pwm_duty = std::fabs(duty) * k_pwm_steps;
        
        // saturation
        if(pwm_duty > k_pwm_steps) pwm_duty = k_pwm_steps;
        
#ifdef DEBUG
        pout_pwm_duty.write(duty * k_pwm_v1 );
#endif
        
        // assign duty low and high bytes to the relevant registers
        unsigned char duty_h, duty_l, ctrl;
        duty_h = (pwm_duty>>8) & 0xFF;
        duty_l = pwm_duty & 0xFF;
        
        // set the first bit to 1 to indicate the negative voltage output
        if(duty < 0)
        {
            duty_h |= 0x80;
        }
        
        LOG_VERBOSE("Sending new duty cycle %f (0x%04X) to PWM registers...", duty, pwm_duty );

        
        wait(instrDelay);	// instruction delay
        bus_write((sc_dt::uint64)(PWM_BASE+PWMDUTYH), (unsigned char) (duty_h));
        
        wait(instrDelay);	// instruction delay
        bus_write((sc_dt::uint64)(PWM_BASE+PWMDUTYL), (unsigned char) (duty_l));
        
        LOG_VERBOSE("Send complete. " );

        
        wait(instrDelay);	// instruction delay
        ctrl = bus_read((sc_dt::uint64)(PWM_BASE+PWMCTRL));
        ctrl |= PWM_CTRL_UDC;
        
        
        LOG_VERBOSE("Updating PWM new duty cycle... " );
        wait(instrDelay);	// instruction delay
        bus_write((sc_dt::uint64)(PWM_BASE+PWMCTRL), (unsigned char) (ctrl));
        LOG_VERBOSE("Update complete. " );

        return ;
    } /* pwm_send_duty() */
    
    
    
    // -----------------------------------------------------------------------------
    //! @brief get duty cycle data from PWM
    
    // -----------------------------------------------------------------------------
    unsigned int sc_tlm_ecu::pwm_get_duty()
    {
        wait(instrDelay);	// instruction delay
        
        //TODO. implement readout of current duty cycle of PWM
        
        return 0;
    } /* pwm_get_duty() */
    
    
    
} // namespace scvp
