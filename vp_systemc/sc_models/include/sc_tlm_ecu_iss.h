/**
 * @file    etc_demo/include/sc_tlm_ecu_iss.h
 * @brief   ECU model by wrapping  Or1ksim(ISS)
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    01.02.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */



#ifndef ETCDEMO_TLM_ECU_ISS_H 
#define ETCDEMO_TLM_ECU_ISS_H




#include <or1ksim.h>

#include <stdint.h>

#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/tlm_quantumkeeper.h"

#include "config_sim.h"
#include "sc_logger.h"

namespace  scvp
{
    
    // ----------------------------------------------------------------------------
    //! SystemC-TLM  module  wrapping Or1ksim ISS
    
    // ----------------------------------------------------------------------------
    class sc_tlm_ecu_iss : public sc_core::sc_module
    {
    public:
        
        //! Initiator port for data accesses
        tlm_utils::simple_initiator_socket<sc_tlm_ecu_iss>  data_bus;
        
        sc_tlm_ecu_iss (sc_core::sc_module_name  name,
                        const char              *configFile,
                        const char              *imageFile);
        
        sc_tlm_ecu_iss (sc_core::sc_module_name  name);
        
        
    protected:
        
        //global quantum.
        tlm::tlm_global_quantum *g_quantum;
        
        // Quantum keeper for the ISS model thread.
        tlm_utils::tlm_quantumkeeper  iss_qkeeper;
        
        virtual void  run ();
        virtual void  do_trans (tlm::tlm_generic_payload &trans);
        
    private:
        
        // initializtaion of ECU ISS model
        void sc_tlm_ecu_iss_init(sc_core::sc_module_name  nm);
        
        tlm::tlm_generic_payload  trans;
        
        static int  static_read_upcall (void              *instancePtr,
                                        unsigned long int  addr,
                                        unsigned char      mask[],
                                        unsigned char      rdata[],
                                        int                dataLen);
        
        static int  static_write_upcall (void              *instancePtr,
                                         unsigned long int  addr,
                                         unsigned char      mask[],
                                         unsigned char      wdata[],
                                         int                dataLen);
        
        int  bus_read (unsigned long int  addr,
                       unsigned char      mask[],
                       unsigned char      rdata[],
                       int                dataLen);
        
        int bus_write (unsigned long int  addr,
                       unsigned char      mask[],
                       unsigned char      wdata[],
                       int                dataLen);
    };
    
} //namespace scvp
#endif //ETCDEMO_TLM_ECU_ISS_H

//eof
