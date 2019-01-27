/**
 * @file    etc_demo/include/sc_tlm_bus.h
 * @brief   Abstracted behavior model of bus (e.g. CAN) transaction using SystemC-TLM.
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    01.02.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */


#ifndef ETCDEMO_TLM_BUS_H
#define ETCDEMO_TLM_BUS_H 


#include <systemc>
#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "config_sim.h"
#include "sc_logger.h"

namespace scvp
{
    
    // ----------------------------------------------------------------------------
    //! System sc_tlm_bus module in ecu.
    
    //! @tparam N_TARGETS   Number of targets on the sc_tlm_bus. The router contains an
    //!                     array of simple initiator sockets.
    //!
    // ----------------------------------------------------------------------------
    template<unsigned int N_TARGETS>
    class sc_tlm_bus : public sc_core::sc_module
    {
    public:
        //! @brief The TLM target socket to receive sc_tlm_bus traffic.
        tlm_utils::simple_target_socket<sc_tlm_bus>  data_bus;
        
        //! @brief The TLM initiator socket to route generic payload transactions
        //! @note   Tagged sockets to be able to distinguish incoming backward path
        //!         calls, because there are multiple initiator sockets.
        tlm_utils::simple_initiator_socket_tagged<sc_tlm_bus>* initiator_socket[N_TARGETS];
        
        // -------------------------------------------------------------------------
        //! Custom Constructor for System sc_tlm_bus.
        
        //! Startup chipID = 0
        //! Register callbacks for incoming interface method calls.
        //! Instantiation of sc_tlm_bus targets.
        
        //! @param name             The SystemC module name
        // -------------------------------------------------------------------------
        SC_CTOR(sc_tlm_bus) : data_bus("data_bus")
        {
            // Register callbacks for incoming interface method calls
            data_bus.register_b_transport(  this, &sc_tlm_bus::bus_read_write);
            
            for (unsigned int i = 0; i < N_TARGETS; i++)
            {
                initiator_socket[i] = new
                tlm_utils::simple_initiator_socket_tagged<sc_tlm_bus>
                (sc_core::sc_gen_unique_name("socket"));
            }
        }
        
    private:
        
        // ----------------------------------------------------------------------------
        //！TLM2.0 blocking transport routine for the sc_tlm_bus socket
        
        //! Receives transport requests and route the transaction on the target socket
        //! by decoding the address.
        
        //! @param trans  The transaction payload
        //! @param delay  How far the initiator is beyond baseline SystemC time. For
        //!              use with temporal decoupling
        // ----------------------------------------------------------------------------
        void bus_read_write( tlm::tlm_generic_payload& trans, sc_core::sc_time& delay )
        {
            sc_dt::uint64 address = trans.get_address();
            sc_dt::uint64 masked_address;
            
            // target number starts from 1 instead of 0
            unsigned int target_nr = decode_address( address, masked_address)  ;
            
            trans.set_response_status( tlm::TLM_OK_RESPONSE );  // Always OK
            
            // Modify address within transaction
            // trans.set_address( masked_address );
            // Forward transaction to appropriate target
            ( *initiator_socket[target_nr] ) -> b_transport( trans, delay );
        }
        
        // ----------------------------------------------------------------------------
        //! Process simple fixed address decoding. Inspect the address attribute to
        //! determine which sockect to send the transaction out through, which it
        //! puts into the variable N_TARGETS.
        
        //! @param address          The address breaked out from generic payload.
        //! @param masked_address   masked address.
        
        //! @return  target number on the sc_tlm_bus
        // ----------------------------------------------------------------------------
        inline unsigned int decode_address( sc_dt::uint64 address,
                                           sc_dt::uint64& masked_address )
        {
            unsigned int target_nr = 0;
            
            unsigned long addr = static_cast<unsigned long>(address & 0xFFFFFF00);
            switch (addr) {
                case ADC_THROTTLE_BASE :
                    target_nr = 0;
                    masked_address = address & ADC_ADDR_MASK;
                    break;
                case PWM_BASE :
                    target_nr = 1;
                    masked_address = address & PWM_ADDR_MASK;
                    break;
                case ADC_PEDAL_BASE :
                    target_nr = 2;
                    masked_address = address & ADC_ADDR_MASK;
                    break;
                default :
                    LOG_ERROR( "TLM ERROR : Given address can not be found.");
            }
            
            LOG_VERBOSE("Decoding address 0x%4X to masked address = 0x%04X, target = %d ", address, masked_address, target_nr);
            
            return target_nr;
        }
        
    };
    
} //namespace scvp

#endif //ETCDEMO_TLM_BUS_H
//eof
