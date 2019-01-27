/**
 * This file is generated using [XML2SCA]
 * Contact : panxiao.tech@gmail.com
 * Web     : http://panxiao.tech/tools/xml2sca/
 *
 * @file    main.cpp
 * @author
 * @date    Apr 16 2018
 * @section LICENSE License (ADD YOUR LICENSE HERE)
 *
 * @section DESCRIPTION Description (ADD YOUR DESCRIPTION HERE)
 *
 */





#include <sys/time.h>

#include "sc_top_eln_e.h"
#include "sc_top_lsf_m.h"
#include "sc_top_motor_em.h"

using namespace std;



// ----------------------------------------------------------------------------
//! @brief main
// ----------------------------------------------------------------------------
int sc_main(int argc, char* argv[])
{
    using sca_core::sca_time;
    
    double tsim = 2;  // simulation durtaion in s
    
    
//    sc_set_time_resolution(1, sc_core::SC_NS);
    
//    sc_top_eln top("top");
//    sc_top_lsf top("top");
    sc_top_motor top("top");
//    sc_top_motor2 top("top");

    // Print some useful information:
    cout << "Info: Simulation options:" << endl;
    cout << "         Simulation time:     " << tsim<< endl;
    cout << "Info: Simulation start. "<< endl;
    
    
    int start_s=clock();
    sc_core::sc_start(tsim, sc_core::SC_SEC);
//    sc_core::sc_stop();
    int stop_s=clock();
        
    // print simulation performance 
    std::cout<< "Info: simulation of "<< tsim <<" SEC complete. (CPU time: "
    << (stop_s-start_s)/double(CLOCKS_PER_SEC) <<" sec )"<< std::endl;
    
    
    return 0;
}
