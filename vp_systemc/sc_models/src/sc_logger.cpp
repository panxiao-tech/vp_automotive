/**
 * @file    sc_logger.h
 * @brief   A simplified logging tool for SystemC (incl.-AMS, -TLM) based modelling and simulation.
 *
 * @author  Xiao Pan <panxiao.tech@gmail.com>
 * @date    09.11.2018
 * @section LICENSE See file LICENSE
 *
 */


#include "sc_logger.h"

using namespace std;

namespace scvp
{
    
#ifdef log_file_path_
    const string sc_logger::log_file_path_ = log_file_path_;
#else
    const string sc_logger::log_file_path_ = "scvp_log.txt";
#endif
    
    sc_logger* sc_logger::logger_instance_ = NULL;
    ofstream sc_logger::log_file_ ;
    
    sc_logger::sc_logger()
    {
    }
    
    // Get logger
    sc_logger* sc_logger::get_logger()
    {
        if (logger_instance_ == NULL)
        {
            logger_instance_ = new sc_logger();
            log_file_ .open(log_file_path_.c_str(), ios::out | ios::trunc);
        }
        return logger_instance_;
    }
    
    
    // Logging function
    void sc_logger::log(const char* log_cat, const char* moddel_name, const char* format, ...)
    {
        va_list ap;
        char buffer[1024];
        va_start(ap, format);
        vsnprintf(buffer, 1024, format, ap);
        
        log_file_  << left << setw(7) << setfill(' ') << log_cat <<" | "
        << right << setw(15) << setfill(' ')
        <<  sc_core::sc_time_stamp().value()/1000<<" ns"
        << " | " << left << setw(30) << setfill(' ') << moddel_name
        << " | " << buffer << endl ;
        log_file_ .flush();
        va_end(ap);
    }
    
    
    // operator <<
    sc_logger& sc_logger::operator << (const string& message)
    {
#if LOG_LEVEL >= INFO_LEVEL
        log_file_  << left << setw(7) << setfill(' ') << "LOG_INFO" <<" | "
        << right << setw(15) << setfill(' ')
        <<  sc_core::sc_time_stamp().value()/1000<<" ns"
        << " | "  << message << "\n";
        return *this;
#endif
    }
    
} //namespace scvp
