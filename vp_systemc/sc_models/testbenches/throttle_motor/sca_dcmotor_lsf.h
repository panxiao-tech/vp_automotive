/**
 *
 * @file    sca_motor_lsf.h
 * @author  Xiao Pan (pan@cs.uni-kl.de)
 * @date    00:30:34 Jun  2 2018
 * @section LICENSE License (ADD YOUR LICENSE HERE)
 *
 * @section DESCRIPTION Description (ADD YOUR DESCRIPTION HERE)
 *          DC Motor for Throttle Control using linear signal flow model
 *          incl. mechanical part : sca_dcmotor_lsf
 *          and   electrical part : sca_dcmotor_eln
 *
 */


#ifndef sca_motor_lsf_h_
#define sca_motor_lsf_h_


#include <systemc-ams>
#include "config_sim.h"


SCA_TDF_MODULE(motor_delay)
{
    sca_tdf::sca_in<double>     pin;
    sca_tdf::sca_out<double>    pout;
    
    void initialize(){}
    
    void set_attribute()
    {
        pout.set_delay(1);
    }
    
    void processing()
    {
        pout.write( pin.read());
    }

    
    SCA_CTOR(motor_delay) {}
};



/**
 * @class sca_dcmotor_eln
 * @brief sc_module implements the dcmotor electrical behavior
 * @author XIAO Pan <pan@cs.uni-kl.de>
 * @version 0.1.0
 *
 */
SC_MODULE(sca_dcmotor_eln)
{
    sca_tdf::sca_in<double>     pin_vs;     //! source voltage
    sca_tdf::sca_in<double>     pin_vb;     //! back emf voltage
    sca_tdf::sca_out<double>    pout_i;     //! output current
    
    sca_dcmotor_eln(sc_core::sc_module_name nm,
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
        
        vb_in=std::make_shared<sca_eln::sca_tdf::sca_vsource>("vb_in");
        vb_in->inp(pin_vb);
        vb_in->p(n4);
        vb_in->n(gnd);
    }
    
private:
    sca_eln::sca_node n1, n2, n3, n4;
    sca_eln::sca_node_ref gnd;
    
    std::shared_ptr<sca_eln::sca_tdf::sca_vsource> vs_in;
    std::shared_ptr<sca_eln::sca_tdf::sca_vsource> vb_in;
    
    std::shared_ptr<sca_eln::sca_tdf::sca_isink>   current_i;
    std::shared_ptr<sca_eln::sca_l>                motor_l;
    std::shared_ptr<sca_eln::sca_r>                motor_r;
};



/**
 * @class sca_dcmotor_lsf
 * @brief sc_module implements the dcmotor mechanical behavior
 * @author XIAO Pan <pan@cs.uni-kl.de>
 * @version 0.1.0
 *
 */
SC_MODULE(sca_dcmotor_lsf)
{
    sca_lsf::sca_in  pin_Tm;    // DC motor output torque
    sca_lsf::sca_out pout_w;    // output of the angular velocity (d/dt(theta))
    sca_lsf::sca_out pout_a;    // output of the position of the shaft(theta)
    
    
    sca_dcmotor_lsf(sc_core::sc_module_name nm,
                    double _Jeq = 0.0021,
                    double _Beq = 0.0088,
                    double _Ks  = 0.087,
                    double _Tpl = 0.396,
                    double _Tf  = 0.284
                    ):
    pin_Tm("pin_Tm"),
    pout_w("pout_w"),
    pout_a("pout_a"),
    sig_d2a("sig_d2a"),
    sig_d1a("sig_d1a"),
    sig_damp("sig_damp"),
    sig_spring("sig_spring"),
    sig_tpl("sig_tpl"),
    sig_sub0("sig_sub0"),sig_sub1("sig_sub1")
    {
        sub_0 = std::make_shared<sca_lsf::sca_sub>("sub_0");
        sub_0->x1(pin_Tm);
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
    
        delay_poutw = std::make_shared<sca_lsf::sca_delay>("delay_poutw");// default ZERO delay
        delay_poutw->x(sig_d1a);
        delay_poutw->y(pout_w);
        
        delay_pouta = std::make_shared<sca_lsf::sca_delay>("delay_pouta");// default ZERO delay
        delay_pouta->x(sig_a);
        delay_pouta->y(pout_a);
        
        
        tf = sca_util::sca_create_tabular_trace_file("tracefile_dcmotor_lsf");
        sca_util::sca_trace(tf, sig_d2a, "sig_d2a");
        sca_util::sca_trace(tf, sig_damp, "sig_damp");
        sca_util::sca_trace(tf, sig_spring, "sig_spring");
        sca_util::sca_trace(tf, sig_tpl, "sig_tpl");

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


    
    sca_lsf::sca_signal sig_d2a; // signal of d2/dt2(theta), input to integ1
    sca_lsf::sca_signal sig_d1a; // signal of d/dt(theta)
    sca_lsf::sca_signal sig_a;   // signal of theta
    sca_lsf::sca_signal sig_damp;   // signal of damping force
    sca_lsf::sca_signal sig_spring;   // signal of return spring force
    sca_lsf::sca_signal sig_tpl;   // signal rep. preload torque

    sca_lsf::sca_signal sig_sub0, sig_sub1;   // output of sub0, sub1 module
    
    sca_util::sca_trace_file* tf;

};


/**
 * @class sca_dcmotor
 * @brief top level model of DC Motor
 * @author XIAO Pan <pan@cs.uni-kl.de>
 * @version 0.1.0
 *
 */
SC_MODULE(sca_dcmotor)
{
    //! in/output ports
    sca_tdf::sca_in<double>   pin_v;   //! DC motor control voltage
    sca_tdf::sca_out<double>  pout_a;  //! throttle position (angle in rad)
//    sca_tdf::sca_out<double>  pout_i;  //! instant current of the DC circuiy in ampere
    
    
    //! @brief Custom constructor of sca_dcmotor
    //! @para Jeq Equivalent- moment of inertia of the rotor,  kg.m^2
    //! @para Beq Equivalent- motor viscous friction constant, Nm/s
    //! @para K repl. Kb (electromotive force constant/back emf, V/rad/sec) and
    //!       Km (motor torque constant, Nm/Amp)
    //! @para R electric resistance, ohm
    //! @para L electric inductance, H
    //! @para Ks spring constant of the return spring, Nm/rad
    //! @para Tpl Spring pre-load torque, Nm
    //! @para Tf Friction torque, Nm
    sca_dcmotor(sc_core::sc_module_name nm,
                double _Jeq = 0.0021,
                double _Beq = 0.0088,
                double _K   = 0.383,
                double _R   = 1.5,
                double _L   = 1.5e-3,
                double _Ks  = 0.087,
                double _Tpl = 0.396,
                double _Tf  = 0.284 ):
    pin_v("pin_v"), pout_a("pout_a"),
//    pout_i("pout_i"),
    sig_i("sig_i"),sig_vb("sig_vb"),sig_tm("sig_tm"),
    sig_w("sig_w"),sig_a("sig_a"), sig_vb_delay("sig_vb_delay")
    {
        i_dcmotor_eln= std::make_shared<sca_dcmotor_eln>("i_dcmotor_eln",_L,_R);
        i_dcmotor_eln->pin_vs(pin_v);
        i_dcmotor_eln->pout_i(sig_i);
        i_dcmotor_eln->pin_vb(sig_vb_delay);

        i_dcmotor_lsf= std::make_shared<sca_dcmotor_lsf>("i_dcmotor_lsf", _Jeq,
                                                         _Beq, _Ks, _Tpl, _Tf);
        i_dcmotor_lsf->pin_Tm(sig_tm);
        i_dcmotor_lsf->pout_a(sig_a);
        i_dcmotor_lsf->pout_w(sig_w);

        i2tm = std::make_shared<sca_lsf::sca_tdf_source>("i2tm", _K); // Km=K
        i2tm->inp(sig_i);
        i2tm->y(sig_tm);
        
        w2vb = std::make_shared<sca_lsf::sca_tdf_sink>("w2vb", _K); // Kb=K
        w2vb->x(sig_w);
        w2vb->outp(sig_vb);

        
        delay_vb = std::make_shared<motor_delay>("delay_vb"); // Kb=K
        delay_vb->pin(sig_vb);
        delay_vb->pout(sig_vb_delay);

        
        a2a = std::make_shared<sca_lsf::sca_tdf_sink>("a2a", 1.0);
        a2a->x(sig_a);
        a2a->outp(pout_a);
        
        tf = sca_util::sca_create_tabular_trace_file("tracefile_dcmotor_sca");
        sca_util::sca_trace(tf, sig_tm, "sig_tm");
        sca_util::sca_trace(tf, sig_w, "sig_w");
        sca_util::sca_trace(tf, sig_a, "sig_a");

    }
    
private:
    std::shared_ptr<sca_dcmotor_eln> i_dcmotor_eln;
    std::shared_ptr<sca_dcmotor_lsf> i_dcmotor_lsf;
    
    /**
     *  TDF -> LSF converter from current to Tm
     *  Tm(t) = Km * i(t);
     */
    std::shared_ptr<sca_lsf::sca_tdf_source> i2tm;
    
    /**
     *  LSF  -> TDF converter from angular velocity to back emf voltage
     *  Vb(t) = Kb * w(t);
     */
    std::shared_ptr<sca_lsf::sca_tdf_sink> w2vb;

    
    /**
     *  LSF  -> TDF converter of position of the shaft
     */
    std::shared_ptr<sca_lsf::sca_tdf_sink> a2a;
    
    
    std::shared_ptr<motor_delay> delay_vb;
    
    
    sca_tdf::sca_signal<double> sig_i;
    sca_tdf::sca_signal<double> sig_vb;
    sca_tdf::sca_signal<double> sig_vb_delay;

    sca_lsf::sca_signal sig_tm;
    sca_lsf::sca_signal sig_w;
    sca_lsf::sca_signal sig_a;

    sca_util::sca_trace_file* tf;

};



#endif

