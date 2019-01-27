/**
 * @file    etc_demo/include/sca_throttle_body.h
 * @brief   Throtthle control acturator system incl.mechanical and electrical part
 *
 * @author  Xiao Pan <pan@cs.uni-kl.de>
 * @date    02-07-2018
 * @section LICENSE License
 *
 * Copyright (c) 2018 Design of Cyber-Physical Systems, TU Kaiserslautern
 *
 */


#ifndef ETCDEMO_THROTTLE_MOTOR_H 
#define ETCDEMO_THROTTLE_MOTOR_H

#include <systemc-ams>
#include "config_sim.h"
#include "sc_logger.h"

namespace scvp
{
// ----------------------------------------------------------------------------
//! @class sac_tdf_em_conv
//! @brief sca_tdf_module electromechanical converter
// ----------------------------------------------------------------------------
SCA_TDF_MODULE(sca_tdf_em_conv)
{
public:
    sca_tdf::sca_in<double>     pin_i;      // input of current
    sca_tdf::sca_in<double>     pin_w;      // input  angular velocity
    sca_tdf::sca_out<double>    pout_tm;    // motor torque output
    sca_tdf::sca_out<double>    pout_vb;    // output of back emf voltage
    
    //! @brief Custom constructor
    //! @para  Kb (electromotive force constant/back emf, V/rad/sec) and
    //! @para  Km (motor torque constant, Nm/Amp)
    sca_tdf_em_conv(sc_core::sc_module_name nm,
                      double _Km = 0.383,
                      double _Kb = 0.383)
    :pin_i("pin_i"),
    pin_w("pin_w"),
    pout_tm("pout_tm"),
    pout_vb("pout_vb")
    {
        Km = _Km;
        Kb = _Kb;
    }
    
private:
    double Km, Kb;

    void initialize(){}

    void set_attributes()
    {
        accept_attribute_changes();
        pout_tm.set_delay(1);
        pout_vb.set_delay(1);
    }
    
    void processing()
    {
        pout_tm.write( pin_i.read() * Km); //Tm(t) = Km * i(t);
        pout_vb.write( pin_w.read() * Kb); //Vb(t) = Kb * w(t);
    }
    
};



// ----------------------------------------------------------------------------
//! @class sca_eln_dcmotor_e
//! @brief sc_module implements the dcmotor electrical behavior
// ----------------------------------------------------------------------------
SC_MODULE(sca_eln_dcmotor_e)
{
    sca_tdf::sca_in<double>     pin_vs;     //! source voltage
    sca_tdf::sca_in<double>     pin_vb;     //! back emf voltage
    sca_tdf::sca_out<double>    pout_i;     //! output current

    
    sca_eln_dcmotor_e(sc_core::sc_module_name nm,
                    double _L = 1.5e-3,
                    double _R = 1.5):
    pin_vs("pin_vs"),
    pin_vb("pin_vb"),
    pout_i("pout_i"),
    n1("n1"), n2("n2"), n3("n3"), n4("n4"), gnd("gnd")
    {
        vs_in=std::make_shared<sca_eln::sca_tdf::sca_vsource>("vs_in");
        vs_in->inp(pin_vs);
        vs_in->p(n1);
        vs_in->n(gnd);
        
        // current sensor
        current_i=std::make_shared<sca_eln::sca_tdf::sca_isink>("current_i");
        current_i->p(n1);
        current_i->n(n2);
        current_i->outp(pout_i);
        
        motor_r=std::make_shared<sca_eln::sca_r>("motor_r", _R);
        motor_r->p(n2);
        motor_r->n(n3);
        
        motor_l=std::make_shared<sca_eln::sca_l>("motor_l", _L);
        motor_l->p(n3);
        motor_l->n(n4);
        
        // back emf voltage
        vb_in=std::make_shared<sca_eln::sca_tdf::sca_vsource>("vb_in");
        vb_in->inp(pin_vb);
        vb_in->p(n4);
        vb_in->n(gnd);
    }
    
private:
    sca_eln::sca_node n1, n2, n3, n4;
    sca_eln::sca_node_ref gnd;
    
    
    std::shared_ptr<sca_eln::sca_tdf::sca_vsource>  vs_in;
    std::shared_ptr<sca_eln::sca_tdf::sca_vsource>  vb_in;
    std::shared_ptr<sca_eln::sca_tdf::sca_isink>    current_i;
    
    std::shared_ptr<sca_eln::sca_l>  motor_l;
    std::shared_ptr<sca_eln::sca_r>  motor_r;
};




// ----------------------------------------------------------------------------
//! @class sca_lsf_dcmotor_m
//! @brief sc_module implements the dcmotor mechanical behavior
// ----------------------------------------------------------------------------
SC_MODULE(sca_lsf_dcmotor_m)
{
    sca_tdf::sca_in<double>     pin_tm;    //! DC motor output torque
    sca_tdf::sca_out<double>    pout_w;    //! output of the angular velocity (d/dt(theta))
    sca_tdf::sca_out<double>    pout_a;    //! output of the position of the shaft(theta)
    
    sca_lsf_dcmotor_m(sc_core::sc_module_name nm,
                    double _Jeq = 0.0021,
                    double _Beq = 0.0088,
                    double _K  = 0.087,
                    double _Ks  = 0.087,
                    double _Tpl = 0.396,
                    double _Tf  = 0.284
                    ):
    pin_tm("pin_tm"),
    pout_w("pout_w"),
    pout_a("pout_a"),
    sig_d2a("sig_d2a"),
    sig_d1a("sig_d1a"),
    sig_damp("sig_damp"),
    sig_spring("sig_spring"),
    sig_tpl("sig_tpl"),
    sig_tm("sig_tm"),
    sig_sub0("sig_sub0"),sig_sub1("sig_sub1")
    {
        sub_0 = std::make_shared<sca_lsf::sca_sub>("sub_0");
        sub_0->x1(sig_tm);
        sub_0->x2(sig_tpl);
        sub_0->y(sig_sub0);
        
        sub_1 = std::make_shared<sca_lsf::sca_sub>("sub_1");
        sub_1->x1(sig_sub0);
        sub_1->x2(sig_spring);
        sub_1->y(sig_sub1);
        
        sub_2 = std::make_shared<sca_lsf::sca_sub>("sub_2");
        sub_2->x1(sig_sub1);
        sub_2->x2(sig_damp);
        sub_2->y(sig_d2a);
        
        integ1 = std::make_shared<sca_lsf::sca_integ>("integ1", 1/_Jeq ,0);
        integ1->x(sig_d2a);
        integ1->y(sig_d1a);
        
        integ2 = std::make_shared<sca_lsf::sca_integ>("integ1", 1.0 ,0);
        integ2->x(sig_d1a);
        integ2->y(sig_a);
        
        gain_damping = std::make_shared<sca_lsf::sca_gain>("gain_damping",_Beq);
        gain_damping->x(sig_d1a);
        gain_damping->y(sig_damp);
        
        gain_rpring = std::make_shared<sca_lsf::sca_gain>("gain_rpring",_Ks);
        gain_rpring->x(sig_a);
        gain_rpring->y(sig_spring);
        
        Tpl = std::make_shared<sca_lsf::sca_source>("Tpl", _Tpl, _Tpl);
        Tpl->y(sig_tpl);

        //  TDF -> LSF converter from current to Tm
        tdf2lsf_tm = std::make_shared<sca_lsf::sca_tdf_source>("tdf2lsf_tm", 1.0);
        tdf2lsf_tm->inp(pin_tm);
        tdf2lsf_tm->y(sig_tm);

        //  LSF  -> TDF converter of position of the shaft
        lsf2tdf_w = std::make_shared<sca_lsf::sca_tdf_sink>("lsf2tdf_w", 1.0);
        lsf2tdf_w->x(sig_d1a);
        lsf2tdf_w->outp(pout_w);
        
        //  LSF  -> TDF converter of position of the shaft
        lsf2tdf_a = std::make_shared<sca_lsf::sca_tdf_sink>("lsf2tdf_a", 1.0);
        lsf2tdf_a->x(sig_a);
        lsf2tdf_a->outp(pout_a);
    }
    
private:
    std::shared_ptr<sca_lsf::sca_integ> integ1;
    std::shared_ptr<sca_lsf::sca_integ> integ2 ;
    std::shared_ptr<sca_lsf::sca_gain> gain_damping;
    std::shared_ptr<sca_lsf::sca_gain> gain_rpring;
    std::shared_ptr<sca_lsf::sca_sub> sub_0;
    std::shared_ptr<sca_lsf::sca_sub> sub_1;
    std::shared_ptr<sca_lsf::sca_sub> sub_2;
    std::shared_ptr<sca_lsf::sca_source> Tpl ; // preload torque
    std::shared_ptr<sca_lsf::sca_delay > delay_poutw ;
    std::shared_ptr<sca_lsf::sca_delay > delay_pouta ;
    
    std::shared_ptr<sca_lsf::sca_tdf_source> tdf2lsf_tm; // tdf2lsf
    std::shared_ptr<sca_lsf::sca_tdf_sink> lsf2tdf_w; //lsf2tdf
    std::shared_ptr<sca_lsf::sca_tdf_sink> lsf2tdf_a; //lsf2tdf

    
    sca_lsf::sca_signal sig_d2a; // signal of d2/dt2(theta), input to integ1
    sca_lsf::sca_signal sig_d1a; // signal of d/dt(theta)
    sca_lsf::sca_signal sig_a;   // signal of theta
    sca_lsf::sca_signal sig_damp;   // signal of damping force
    sca_lsf::sca_signal sig_spring;   // signal of return spring force
    sca_lsf::sca_signal sig_tpl;   // signal rep. preload torque
    sca_lsf::sca_signal sig_tm; // torque from motor

    sca_lsf::sca_signal sig_sub0, sig_sub1;   // output of sub0, sub1 module
};


// ----------------------------------------------------------------------------
//! @class sca_throttle_body
//! @brief top level model of DC Motor
// ----------------------------------------------------------------------------
SC_MODULE(sca_throttle_body)
{
    //! in/output ports
    sca_tdf::sca_in<double>   pin_v;   //! DC motor control voltage
    sca_tdf::sca_out<double>  pout_a;  //! throttle position (angle in rad)
    sca_tdf::sca_out<double>  pout_i;  //! instant current of the DC circuiy in ampere
    
    
    //! @brief Custom constructor of sca_throttle_body
    //! @para Jeq Equivalent- moment of inertia of the rotor,  kg.m^2
    //! @para Beq Equivalent- motor viscous friction constant, Nm/s
    //! @para K repl. Kb (electromotive force constant/back emf, V/rad/sec) and
    //!       Km (motor torque constant, Nm/Amp)
    //! @para R electric resistance, ohm
    //! @para L electric inductance, H
    //! @para Ks spring constant of the return spring, Nm/rad
    //! @para Tpl Spring pre-load torque, Nm
    //! @para Tf Friction torque, Nm
    sca_throttle_body(sc_core::sc_module_name nm,
                double _Jeq = 0.0021,
                double _Beq = 0.0088,
                double _K   = 0.383,
                double _R   = 1.5,
                double _L   = 1.5e-3,
                double _Ks  = 0.087,
                double _Tpl = 0.396,
                double _Tf  = 0.284 ):
    pin_v("pin_v"), pout_a("pout_a"),
    pout_i("pout_i"),
    sig_i("sig_i"),
    sig_v("sig_v"),
    sig_vb("sig_vb"),
    sig_w("sig_w"),
    sig_tm("sig_tm")
    {
        i_dcmotor_eln= std::make_shared<sca_eln_dcmotor_e>("dcmotor_eln",_L,_R);
        i_dcmotor_eln->pin_vs(pin_v);
        i_dcmotor_eln->pout_i(pout_i);
        i_dcmotor_eln->pin_vb(sig_vb);

        i_em_conv = std::make_shared<sca_tdf_em_conv>("em_conv", _K, _K);
        i_em_conv->pin_i(pout_i);
        i_em_conv->pout_tm(sig_tm);
        i_em_conv->pin_w(sig_w);
        i_em_conv->pout_vb(sig_vb);
        i_em_conv->set_timestep(sc_core::sc_time(k_tstep_in_sec, sc_core::SC_SEC ));

        i_dcmotor_lsf= std::make_shared<sca_lsf_dcmotor_m>("dcmotor_lsf", _Jeq, _Beq, _K,  _Ks, _Tpl, _Tf);
        i_dcmotor_lsf->pin_tm(sig_tm);
        i_dcmotor_lsf->pout_w(sig_w);
        i_dcmotor_lsf->pout_a(pout_a);

    }
    
private:
    std::shared_ptr<sca_eln_dcmotor_e> i_dcmotor_eln;
    std::shared_ptr<sca_lsf_dcmotor_m> i_dcmotor_lsf;
    std::shared_ptr<sca_tdf_em_conv>   i_em_conv;

    sca_tdf::sca_signal<double> sig_v;
    sca_tdf::sca_signal<double> sig_i;
    sca_tdf::sca_signal<double> sig_vb;
    sca_tdf::sca_signal<double> sig_w;
    sca_tdf::sca_signal<double> sig_tm;
};

} //namespace scvp
#endif // ETCDEMO_THROTTLE_MOTOR_H

//eof 
