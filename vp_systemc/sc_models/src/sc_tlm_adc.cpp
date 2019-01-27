/**
 * @file    etc_demo/include/sc_tlm_adc.cpp
 * @brief   SystemC-TLM model of ADC incl. ADC analog part impl. in SCA
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    05.05.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */


#include "sc_tlm_adc.h"

namespace scvp
{
    SC_HAS_PROCESS( sc_tlm_adc );
    
    // ----------------------------------------------------------------------------
    //! @brief Custom Constructor for sc_tlm_adc module.
    
    //! @param name         The SystemC module name
    // ----------------------------------------------------------------------------
    sc_tlm_adc::sc_tlm_adc(sc_core::sc_module_name  nm,
                           double _vref,
                           int _res,
                           int _rate,
                           sca_core::sca_time _t_step)
    : sc_module( nm ),
    i_bus("bus"),
    i_adc_ana("adc_ana",_vref, _res,_rate, _t_step)
    {
        i_bus.register_b_transport( this, &sc_tlm_adc::bus_read_write );
        
        i_adc_ana.pout_conv(sig_adc_ana_out);
        pin_pos = &i_adc_ana.pin_pos;
        pin_neg = &i_adc_ana.pin_neg;
        
        // Set up the method for the sc_tlm_adc (statically sensitive to sig_adc_ana_out)
        SC_METHOD( method_meas_update );
        sensitive << sig_adc_ana_out;
        dont_initialize();
        
    }	//sc_tlm_adc()
    
    
    // Destructor
    sc_tlm_adc:: ~sc_tlm_adc()
    {
    }
    
    
    // ----------------------------------------------------------------------------
    //! @brief SystemC method sensitive to data on the sc_tlm_adc analog output.
    //! For simplicity, this sc_tlm_adc always updates its measurement result and
    //! send the latest data to ECU by reading the internal register 'datareg'
    // ----------------------------------------------------------------------------
    void sc_tlm_adc::method_meas_update()
    {
        unsigned int adc_conv = sig_adc_ana_out.read();
        regs.datah = (adc_conv>>8) & 0xFF;
        regs.datal = adc_conv & 0xFF;
        
        LOG_INFO("Conversion complete, measurement = %f ", (double) (adc_conv * 2.0/ pow(2, 12)));
        
    }/* method_intr_spi_tx() */
    
    
    
    // ----------------------------------------------------------------------------
    //! @brief TLM2.0 blocking transport routine for the sc_tlm_adc bus socket
    
    //! - Receives transport requests on the target socket.
    //! - Break out the command, address, data.
    //!     - if address = broadcast, assgin chip
    //!     - Switches on the command and calls sc_tlm_adc::bus_read() or
    //!         sc_tlm_adc::bus_write() routines to do the behavior.
    //! - Increases the delay as appropriate and sets a success response.
    
    //! @param trans  The transaction payload
    //! @param delay  How far the initiator is beyond baseline SystemC time. For
    //!              use with temporal decoupling
    // ----------------------------------------------------------------------------
    void
    sc_tlm_adc::bus_read_write( tlm::tlm_generic_payload &trans,
                               sc_core::sc_time         &delay )
    {
        // Break out the address, mask and data pointer.
        tlm::tlm_command    comm    = trans.get_command();
        sc_dt::uint64       addr    = trans.get_address();
        unsigned char     	*maskPtr = trans.get_byte_enable_ptr();
        unsigned char       *dataPtr = trans.get_data_ptr();
        int                 offset = 0;
        unsigned char		adcaddr;		//sc_tlm_adc addresses
        
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
        adcaddr = (unsigned char)((addr + offset) & ADC_ADDR_MASK);
        
        switch( comm ) {
            case tlm::TLM_READ_COMMAND:
                LOG_VERBOSE("TLM bus Read from address \'0x%04X\'.", adcaddr);
                dataPtr[offset] = bus_read( adcaddr );
                break;
            case tlm::TLM_WRITE_COMMAND:
                LOG_VERBOSE("TLM bus Write \'0x%02X\' from address \'0x%04X\'.", *((uint32_t *)dataPtr), adcaddr);
                bus_write( adcaddr, dataPtr[offset] );
                break;
            case tlm::TLM_IGNORE_COMMAND:
                LOG_ERROR("TLM bus command not supported.");
                trans.set_response_status( tlm::TLM_GENERIC_ERROR_RESPONSE );
                return;
        }
        
        trans.set_response_status( tlm::TLM_OK_RESPONSE );  // Always OK
        
    }	/* bus_read_write() */
    
    
    // ----------------------------------------------------------------------------
    //! @brief write bus
    // ----------------------------------------------------------------------------
    void sc_tlm_adc::bus_write( unsigned char  addr,
                               unsigned char  wdata )
    {
        // update control regitser. For simplicity, this function is not
        // implemented here.
        switch (addr){
            case ADCCTRL :
                regs.ctrl = wdata;
                LOG_VERBOSE("Write \'0x%02X\' to ADC CTRL regisger.", wdata);
                break;
        }
        return;
    }
    
    // ----------------------------------------------------------------------------
    //! @brief read bus
    // ----------------------------------------------------------------------------
    unsigned char sc_tlm_adc::bus_read( unsigned char  addr )
    {
        unsigned char res = 0;
        switch (addr){
            case ADCCTRL:
                res = regs.ctrl;
                LOG_VERBOSE("Readout \'0x%02X\' from ADC CTRL regisger.", res);
                break;
            case ADCDATAH:
                res = regs.datah;
                LOG_VERBOSE("Readout \'0x%02X\' from ADC datah regisger.", res);
                break;
            case ADCDATAL:
                res = regs.datal;
                LOG_VERBOSE("Readout \'0x%02X\' from ADC datal regisger.", res);
                break;
        }
        return res;
    }
    
    
} //namespace scvp
