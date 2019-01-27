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

#include "sc_top_motor_lsf.h"

using namespace std;



// ----------------------------------------------------------------------------
//! @brief main
// ----------------------------------------------------------------------------
int sc_main(int argc, char* argv[])
{
    using sca_core::sca_time;
    
    double tsim = 1;  // simulation durtaion in s
    
    
    
    sc_top_motor top("top");

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
