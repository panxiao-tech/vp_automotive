/**
 * @file    etc_demo/src/sc_tlm_ecu_iss.cpp
 * @brief   ECU model by wrapping  Or1ksim(ISS)
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    01.02.2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */


#include "sc_tlm_ecu_iss.h"



SC_HAS_PROCESS( sc_tlm_ecu_iss );

// ----------------------------------------------------------------------------
//! Custom constructor
//! @param name        module name
//! @param configFile  Config file for the  ISS
//! @param imageFile   Binary image to run on the ISS
// ----------------------------------------------------------------------------
sc_tlm_ecu_iss::sc_tlm_ecu_iss (sc_core::sc_module_name  name,
                                const char              *configFile,
                                const char              *imageFile) :

sc_module (name),
data_bus ("data_initiator" )
{
    or1ksim_init (configFile, imageFile, this, static_read_upcall, static_write_upcall);
    sc_tlm_ecu_iss_init(name);
}


// ----------------------------------------------------------------------------
//! Overloading  constructor use static path of configuration  and binary files
//! @param name        module name
// ----------------------------------------------------------------------------
sc_tlm_ecu_iss::sc_tlm_ecu_iss (sc_core::sc_module_name  name) :
sc_module (name),
data_bus ("data_initiator" )
{
    const char* configFile = "/home/cps/git/iss_sca/ecuSC_iss/iss_cfg/or1k_simple.cfg";
    const char* imageFile = "/home/cps/git/iss_sca/ecuSC_iss/ecu_firmware/bin/ecu_firmware" ;
    or1ksim_init (configFile, imageFile, this, static_read_upcall, static_write_upcall);
    
    sc_tlm_ecu_iss_init(name);
    
}


// ----------------------------------------------------------------------------
//! ISS ECU model  Initialization
// ----------------------------------------------------------------------------
void sc_tlm_ecu_iss::sc_tlm_ecu_iss_init(sc_core::sc_module_name  nm)
{
    g_quantum = &(tlm::tlm_global_quantum::instance());
    iss_qkeeper.set_global_quantum( g_quantum->get() );
    iss_qkeeper.reset();	// Zero local time offset
    
    SC_THREAD (run);		  // Thread to run the ISS
    //syn-soc
    or1ksim_set_time_point();		// Mark the start time
    
}


// ----------------------------------------------------------------------------
//! SystemC thread running the underlying ISS
// ----------------------------------------------------------------------------
void
sc_tlm_ecu_iss::run ()
{
    while (true)
    {
        sc_core::sc_time time_left =
        g_quantum->compute_local_quantum() - iss_qkeeper.get_local_time();
        
        or1ksim_set_time_point();
        (void)or1ksim_run(time_left.to_seconds());
        iss_qkeeper.inc( sc_core::sc_time( or1ksim_get_time_period(), sc_core::SC_SEC ));
        
        //Sync if needed
        if(iss_qkeeper.need_sync())
        {
            iss_qkeeper.sync();
        }
    }
}


// ----------------------------------------------------------------------------
//! Static upcall for read from the underlying Or1ksim ISS ()
//!
//! @param[in]  instancePtr  The pointer to the class member associated with
//!                          this upcall (originally passed to or1ksim_init in
//!                          the constructor, ::()).
//! @param[in]  addr         The address for the read
//! @param[in]  mask         The byte enable mask for the read
//! @param[out] rdata        Vector for the read data
//! @param[in]  dataLen      The number of bytes to read
//!
//! @return  Zero on success. A return code otherwise.
// ----------------------------------------------------------------------------
int sc_tlm_ecu_iss::static_read_upcall (void  *instancePtr,
                                        unsigned long int  addr,
                                        unsigned char      mask[],
                                        unsigned char      rdata[],
                                        int                dataLen)
{
    sc_tlm_ecu_iss *classPtr = (sc_tlm_ecu_iss *) instancePtr;
    return  classPtr->bus_read (addr, mask, rdata, dataLen);
}


// ----------------------------------------------------------------------------
//! Static upcall for write to the underlying Or1ksim ISS
//!
//! @param[in] instancePtr  The pointer to the class member associated with
//!                         this upcall (originally passed to or1ksim_init in
//!                         the constructor, ::()).
//! @param[in] addr         The address for the write
//! @param[in] mask         The byte enable mask for the write
//! @param[in] wdata        Vector of data to write
//! @param[in] dataLen      The number of bytes to write
//!
//! @return  Zero on success. A return code otherwise.
// ----------------------------------------------------------------------------
int sc_tlm_ecu_iss::static_write_upcall (void  *instancePtr,
                                         unsigned long int  addr,
                                         unsigned char      mask[],
                                         unsigned char      wdata[],
                                         int                dataLen)
{
    sc_tlm_ecu_iss *classPtr = (sc_tlm_ecu_iss *) instancePtr;
    return  classPtr->bus_write (addr, mask, wdata, dataLen);
}


// ----------------------------------------------------------------------------
//! Member function to handle read upcall from the underlying Or1ksim ISS
//!
//! @param[in]  addr         The address for the read
//! @param[in]  mask         The byte enable mask for the read
//! @param[out] rdata        Vector for the read data
//! @param[in]  dataLen      The number of bytes to read
//!
//! @return  Zero on success. A return code otherwise.
// ----------------------------------------------------------------------------
int sc_tlm_ecu_iss::bus_read (unsigned long int  addr,
                              unsigned char      mask[],
                              unsigned char      rdata[],
                              int                dataLen)
{
    // Set up the payload fields. Assume everything is 4 bytes.
    trans.set_read ();
    trans.set_address ((sc_dt::uint64) addr);
    
    trans.set_byte_enable_length ((const unsigned int) dataLen);
    trans.set_byte_enable_ptr ((unsigned char *) mask);
    
    trans.set_data_length ((const unsigned int) dataLen);
    trans.set_data_ptr ((unsigned char *) rdata);
    
    // Transport.
    do_trans (trans);
    
    /* For now just simple non-zero return code on error */
    return  trans.is_response_ok () ? 0 : -1;
    
}


// ----------------------------------------------------------------------------
//! Member function to handle write upcall from the underlying Or1ksim ISS
//!
//! @param[in] addr         The address for the write
//! @param[in] mask         The byte enable mask for the write
//! @param[in] wdata        Vector of data to write
//! @param[in] dataLen      The number of bytes to write
//!
//! @return  Zero on success. A return code otherwise.
// ----------------------------------------------------------------------------
int sc_tlm_ecu_iss::bus_write (unsigned long int  addr,
                               unsigned char      mask[],
                               unsigned char      wdata[],
                               int                dataLen)
{
    // Set up the payload fields. Assume everything is 4 bytes.
    trans.set_write ();
    trans.set_address ((sc_dt::uint64) addr);
    
    trans.set_byte_enable_length ((const unsigned int) dataLen);
    trans.set_byte_enable_ptr ((unsigned char *) mask);
    
    trans.set_data_length ((const unsigned int) dataLen);
    trans.set_data_ptr ((unsigned char *) wdata);
    
    // Transport.
    do_trans( trans );
    
    /* For now just simple non-zero return code on error */
    return  trans.is_response_ok () ? 0 : -1;
    
}


// ----------------------------------------------------------------------------
//! TLM transport to the target

//! Calls the blocking transport routine for the initiator socket (@see
//! ::data_bus). Passes in a dummy time delay of zero.

//! @param trans  The transaction payload
// ----------------------------------------------------------------------------
void sc_tlm_ecu_iss::do_trans( tlm::tlm_generic_payload &trans )
{
    iss_qkeeper.inc( sc_core::sc_time( or1ksim_get_time_period(), sc_core::SC_SEC ));
    or1ksim_set_time_point();
    
    sc_core::sc_time  delay = iss_qkeeper.get_local_time();
    
    data_bus->b_transport( trans, delay );
    iss_qkeeper.set( delay );			// Updated
    
    if( iss_qkeeper.need_sync() )
    {
        iss_qkeeper.sync();
    }
    
    sc_core::sc_time  time_left      =
    g_quantum->compute_local_quantum() - iss_qkeeper.get_local_time();
    or1ksim_reset_duration ( time_left.to_seconds() );
}




