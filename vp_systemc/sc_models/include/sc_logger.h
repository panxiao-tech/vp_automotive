/**
 * @file    sc_logger.h
 * @brief   A simplified logging tool for SystemC (incl.-AMS, -TLM) based modelling and simulation.
 *
 * @author  Xiao Pan <panxiao.tech@gmail.com>
 * @date    09.11.2018
 * @section LICENSE See file LICENSE
 *
 */

#ifndef ETCDEMO_SC_LOGGER_H
#define ETCDEMO_SC_LOGGER_H
#include <fstream>
#include <iostream>
#include <cstdarg>
#include <string>
#include <systemc-ams>
#include "config_sim.h"


namespace scvp
{
    
#define NO_LOG          0x00
#define ERROR_LEVEL     0x01
#define INFO_LEVEL      0x02
#define DEBUG_LEVEL     0x03
#define VERBOSE_LEVEL   0x04
    
    
#ifndef LOG_LEVEL
#define LOG_LEVEL   DEBUG_LEVEL
#endif
    
#if LOG_LEVEL >= VERBOSE_LEVEL
#define LOG_VERBOSE(format, args...)  do{ sc_logger::get_logger()->log("VERBOSE", name(), format, ##args);} while(0)
#else
#define LOG_VERBOSE(format, args...)
#endif
    
#if LOG_LEVEL >= DEBUG_LEVEL
#define LOG_DEBUG(format, args...)  do{ sc_logger::get_logger()->log("DEBUG", name(), format, ##args);} while(0)
#else
#define LOG_DEBUG(format, args...)
#endif
    
#if LOG_LEVEL >= INFO_LEVEL
#define LOG_INFO(format, args...)  do{ sc_logger::get_logger()->log("INFO", name(), format, ##args);} while(0)
#else
#define LOG_INFO(format, args...)
#endif
    
#if LOG_LEVEL >= ERROR_LEVEL
#define LOG_ERROR(format, args...)  do{ sc_logger::get_logger()->log("ERROR", name(), format, ##args);} while(0)
#else
#define LOG_ERROR(format, args...)
#endif
    
    // ----------------------------------------------------------------------------
    //! @brief Singleton sc_logger Class.
    // ----------------------------------------------------------------------------
    class sc_logger
    {
    public:
        
        // ----------------------------------------------------------------------------
        //! public log interface
        // ----------------------------------------------------------------------------
        void log(const char* log_cat, const char* moddel_name, const char* format, ...);
        
        // ----------------------------------------------------------------------------
        //! operator overloading
        // ----------------------------------------------------------------------------
        sc_logger& operator<<(const std::string& message);
        
        
        // ----------------------------------------------------------------------------
        //! Funtion to create the instance of sc_logger class.
        //! @return singleton object of sc_logger class..
        // ----------------------------------------------------------------------------
        static sc_logger* get_logger();
        
    private:
        // ----------------------------------------------------------------------------
        //! Default constructor for the sc_logger class.
        // ----------------------------------------------------------------------------
        sc_logger();
        
        // ----------------------------------------------------------------------------
        //! Copy constructor for the sc_logger class.
        // ----------------------------------------------------------------------------
        sc_logger(const sc_logger&){};// copy constructor is private
        
        // ----------------------------------------------------------------------------
        //! Assignment operator for the sc_logger class.
        // ----------------------------------------------------------------------------
        sc_logger& operator=(const sc_logger&){ return *this; };
        
        // ----------------------------------------------------------------------------
        //! Log file name.
        // ----------------------------------------------------------------------------
        static const std::string log_file_path_;
        
        // ----------------------------------------------------------------------------
        //! Singleton sc_logger class object pointer.
        // ----------------------------------------------------------------------------
        static sc_logger* logger_instance_;
        
        // ----------------------------------------------------------------------------
        //!   log file stream object.
        // ----------------------------------------------------------------------------
        static std::ofstream log_file_ ;
        
    };
    
} //namespace scvp

#endif //ETCDEMO_SC_LOGGER_H
