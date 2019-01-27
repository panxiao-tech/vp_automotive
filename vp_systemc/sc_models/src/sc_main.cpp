/**
 * @file     etc_demo/src/sc_main.h
 * @brief    main file of ecu demo application 
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    01-02-2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */


#include <ctime>
#include <sys/time.h>

#include "sc_top.h"

using namespace scvp;


// -----------------------------------------------------------------------------
/// @brief sc_main
// -----------------------------------------------------------------------------
int sc_main(int argc, char* argv[])
{
	// Define and set the global time quantum
	tlm::tlm_global_quantum &global_quatum = tlm::tlm_global_quantum::instance();
	global_quatum.set(sc_core::sc_time(k_quantum_in_us, sc_core::SC_US));

    sc_top top("top");
    
    double sim_time = k_tsim_in_sec; //unit SEC
    std::cout<< ">> simulation start.\n";

    clock_t t_start = clock();
    sc_core::sc_start(sim_time, sc_core::SC_SEC);
    clock_t t_end = clock();
    
    sc_core::sc_stop();

    
    std::cout << ">> simulation complete.\n";
    std::cout << ">> simulation of " << sim_time <<"s complete(elapsed time = ";
    std::cout << static_cast<double>(t_end - t_start) * 1e3 / CLOCKS_PER_SEC << " ms.)" << std::endl;

    
//    string message1 = "logg message 1 ...";
//    string message2 = "logg message 2 ...";
//    int    nNum = 10;
//    CLogger::GetLogger()->Log("message to be logged");
//    CLogger::GetLogger()->Log(message1);
//    LOGGER->Log(" Message is:%s Number is:%d", message2.c_str(), nNum);
//     *CLogger::GetLogger() << "log2";
    
   return 0;
}
