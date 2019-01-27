/**
 * @file    etc_demo/src/sc_tlm_pwm.cpp
 * @brief   PWM unit model incl. analog part implemented in SCA
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date  05.05.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */

#include "sc_tlm_pwm.h"

namespace scvp
{
    
    SC_HAS_PROCESS( sc_tlm_pwm );
    
    
    // ----------------------------------------------------------------------------
    //! @brief Custom Constructor for sc_tlm_pwm module.
    
    //! @param name         The SystemC module name
    // ----------------------------------------------------------------------------
    sc_tlm_pwm::sc_tlm_pwm( sc_core::sc_module_name  nm,
                           double v0_,
                           double v1_,
                           const sca_core::sca_time& t_period_,
                           const sca_core::sca_time& t_ramp_)
    : sc_module( nm ),
    i_bus("i_bus"),
    i_pwm_ana("i_pwm_ana", v0_, v1_, t_period_, t_ramp_)
    {
        
        i_bus.register_b_transport( this, &sc_tlm_pwm::bus_read_write );
        
        i_pwm_ana.pin_duty(sig_pwm_ana_in);
        pout_pwm = &i_pwm_ana.pout_pwm;
        
        sig_pwm_ana_in.write(0.5);
    }	//sc_tlm_pwm()
    
    
    
    // ----------------------------------------------------------------------------
    //! @brief TLM2.0 blocking transport routine for the sc_tlm_pwm bus socket
    
    //! - Receives transport requests on the target socket.
    //! - Break out the command, address, data.
    //!     - if address = bropwmast, assgin chip
    //!     - Switches on the command and calls sc_tlm_pwm::bus_read() or
    //!         sc_tlm_pwm::bus_write() routines to do the behavior.
    //! - Increases the delay as appropriate and sets a success response.
    
    //! @param trans  The transaction payload
    //! @param delay  How far the initiator is beyond baseline SystemC time. For
    //!              use with temporal decoupling
    // ----------------------------------------------------------------------------
    void
    sc_tlm_pwm::bus_read_write( tlm::tlm_generic_payload &trans,
                               sc_core::sc_time         &delay )
    {
        
        //    LOG_MSG_LN("bus ReadWrite");
        // Break out the address, mask and data pointer.
        tlm::tlm_command    comm    = trans.get_command();
        sc_dt::uint64       addr    = trans.get_address();
        unsigned char     *maskPtr = trans.get_byte_enable_ptr();
        unsigned char       *dataPtr = trans.get_data_ptr();
        int                 offset = 0;
        unsigned char		pwmaddr;		//sc_tlm_pwm addresses
        
        if(!maskPtr )
        {
            offset = 0;
        }
        else
        {
            // Deduce the byte address (endianness independent)
            if (0xff == maskPtr[0])
            {
                offset = 0;
            }
            else if (0xff == maskPtr[1])
            {
                offset = 1;
            }
            else if (0xff == maskPtr[2])
            {
                offset = 2;
            }
            else if (0xff == maskPtr[3])
            {
                offset = 3;
            }
            else
            {
                trans.set_response_status( tlm::TLM_GENERIC_ERROR_RESPONSE );
                return;
            }
        }
        
        // Mask off the address to its range. This ought to have been done already
        // by an arbiter/decoder.
        pwmaddr = (unsigned char)((addr + offset) & PWM_ADDR_MASK);
        
        switch( comm )
        {
            case tlm::TLM_READ_COMMAND:
                dataPtr[offset] = bus_read( pwmaddr );
                LOG_VERBOSE("TLM bus Read from address \'0x%04X\'.", pwmaddr);
                break;
            case tlm::TLM_WRITE_COMMAND:
                bus_write( pwmaddr, dataPtr[offset] );
                LOG_VERBOSE("TLM bus Write \'0x%02X\' from address \'0x%04X\'.", *((uint32_t *)dataPtr), pwmaddr);
                break;
            case tlm::TLM_IGNORE_COMMAND:
                LOG_ERROR("TLM bus command not supported.");
                trans.set_response_status( tlm::TLM_GENERIC_ERROR_RESPONSE );
                return;
        }
        
        trans.set_response_status( tlm::TLM_OK_RESPONSE );  // Always OK
        
    }	/* bus_read_write() */
    
    
    // ----------------------------------------------------------------------------
    // @brief  excute bus write function
    // ----------------------------------------------------------------------------
    void sc_tlm_pwm::bus_write( unsigned char  addr,
                               unsigned char  wdata )
    {
        // update control regitser. For simplicity, this function is not
        // implemented here.
        switch (addr) {
            case PWMCTRL : regs.ctrl  = wdata;
                LOG_VERBOSE("Write \'0x%02X\' to PWM CTRL regisger.", wdata);
                // update duty cycle if UDC bit in ctrl register is set
                if(PWM_CTRL_UDC == (regs.ctrl & PWM_CTRL_UDC))
                {
                    update_dutycycle();
                }
                break;
            case PWMDUTYH :
                regs.dutyh = wdata;
                LOG_VERBOSE("Write \'0x%02X\' to PWM DUTYH regisger.", wdata);
                break;
            case PWMDUTYL :
                regs.dutyl = wdata;
                LOG_VERBOSE("Write \'0x%02X\' to PWM DUTYL regisger.", wdata);
                break;
        }
        return;
    }
    
    
    // ----------------------------------------------------------------------------
    // @brief  excute bus read function
    // ----------------------------------------------------------------------------
    unsigned char sc_tlm_pwm::bus_read( unsigned char  addr )
    {
        unsigned char res;
        switch (addr) {
            case PWMCTRL  :
                res = regs.ctrl;
                LOG_VERBOSE("Readout \'0x%02X\' from PWM CTRL regisger.", res);
                break;
            case PWMDUTYH :
                res = regs.dutyh;
                LOG_VERBOSE("Readout \'0x%02X\' from PWM DUTYH regisger.", res);
                break;
            case PWMDUTYL :
                res = regs.dutyl;
                LOG_VERBOSE("Readout \'0x%02X\' from PWM DUTYL regisger.", res);
                break;
        }
        return res;
    }
    
    
    // ----------------------------------------------------------------------------
    //! @brief update sc_tlm_pwm dutycyle
    // ----------------------------------------------------------------------------
    void sc_tlm_pwm::update_dutycycle()
    {
        unsigned int dutycycle =0 ;
        dutycycle = (regs.dutyh & 0x7F )| dutycycle;
        dutycycle = (regs.dutyl & 0xFF )| (dutycycle<<8 );
        
        double duty_per = dutycycle/(double)k_pwm_steps;
        
        if(regs.dutyh & 0x80)
        {
            duty_per = -duty_per;
        }
        
        sig_pwm_ana_in.write(duty_per);
        
        LOG_VERBOSE("Set PWM duty cycle to %f (dutyreg = 0x%04X).", duty_per, dutycycle);
        return;
    }
    
    
} // namespace scvp

