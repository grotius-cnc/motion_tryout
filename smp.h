﻿#ifndef SMP_H
#define SMP_H

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <chrono>
#include <unistd.h>

class smp
{
public:
    smp(){};

    enum period_type {
        acc,
        steady,
        dcc,
        none
    };

    struct smp_data {
        //! Inputs.
        double vo=0;
        double ve=0;
        double vm=0;
        double a=0;
        double s=0;

        double vi=0; //! Interpolated velocity.
        double si=0; //! Interpolated displacement.

        //! Outputs.
        double t1=0, t2=0, t3=0;
        double s1=0, s2=0, s3=0;
        period_type p1,p2,p3;
    };

    void print(double vo, double vm, double ve, double a,
               double t1, double t2, double t3,
               double s1, double s2, double s3,
               period_type p1, period_type p2, period_type p3,
               std::string debug_message){
        std::cout<<std::fixed<<" "<<std::endl;
        std::cout<<std::fixed<<debug_message<<std::endl;
        std::cout<<std::fixed<<"t1:"<<t1<<std::endl;
        std::cout<<std::fixed<<"t2:"<<t2<<std::endl;
        std::cout<<std::fixed<<"t3:"<<t3<<std::endl;
        std::cout<<std::fixed<<"ttot:"<<t1+t2+t3<<std::endl;
        std::cout<<std::fixed<<"s1:"<<s1<<std::endl;
        std::cout<<std::fixed<<"s2:"<<s2<<std::endl;
        std::cout<<std::fixed<<"s3:"<<s3<<std::endl;
        std::cout<<std::fixed<<"stot:"<<s1+s2+s3<<std::endl;
        //! Print period t1 type.
        if(p1==period_type::acc){
            std::cout<<std::fixed<<"p1:acc"<<std::endl;
        }
        if(p1==period_type::dcc){
            std::cout<<std::fixed<<"p1:dcc"<<std::endl;
        }
        if(p1==period_type::steady){
            std::cout<<std::fixed<<"p1:steady"<<std::endl;
        }
        if(p1==period_type::none){
            std::cout<<std::fixed<<"p1:none"<<std::endl;
        }
        //! Print period t2 type.
        if(p2==period_type::acc){
            std::cout<<std::fixed<<"p2:acc"<<std::endl;
        }
        if(p2==period_type::dcc){
            std::cout<<std::fixed<<"p2:dcc"<<std::endl;
        }
        if(p2==period_type::steady){
            std::cout<<std::fixed<<"p2:steady"<<std::endl;
        }
        if(p2==period_type::none){
            std::cout<<std::fixed<<"p2:none"<<std::endl;
        }
        //! Print period t3 type.
        if(p3==period_type::acc){
            std::cout<<std::fixed<<"p3:acc"<<std::endl;
        }
        if(p3==period_type::dcc){
            std::cout<<std::fixed<<"p3:dcc"<<std::endl;
        }
        if(p3==period_type::steady){
            std::cout<<std::fixed<<"p3:steady"<<std::endl;
        }
        if(p3==period_type::none){
            std::cout<<std::fixed<<"p3:none"<<std::endl;
        }

        std::cout<<std::fixed<<"vo:"<<vo<<std::endl;
        std::cout<<std::fixed<<"vm:"<<vm<<std::endl;
        std::cout<<std::fixed<<"ve:"<<ve<<std::endl;
        std::cout<<std::fixed<<"a:"<<a<<std::endl;
        std::cout<<std::fixed<<" "<<std::endl;
    }

    smp_data setVi(smp_data d, double vi, bool debug){
        d.vi=vi;
        if(debug){
            std::cout<<std::fixed<<"vi:"<<vi<<std::endl;
        }
        return d;
    }

    smp_data setSi(smp_data d, double si, bool debug){
        d.si=si;
        if(debug){
            std::cout<<std::fixed<<"si:"<<si<<std::endl;
        }
        return d;
    }

    smp_data setData(double vo, double vm, double ve, double a,
                     double t1, double t2, double t3,
                     double s1, double s2, double s3,
                     period_type p1, period_type p2, period_type p3,
                     bool debug, std::string debug_message){
        smp_data d;
        d.vo=vo;
        d.vm=vm;
        d.ve=ve;
        d.a=a;
        d.t1=t1;
        d.t2=t2;
        d.t3=t3;
        d.s1=s1;
        d.s2=s2;
        d.s3=s3;
        d.p1=p1;
        d.p2=p2;
        d.p3=p3;
        d.s=s1+s2+s3;

        if(debug){
            print(vo,vm,ve,a,t1,t2,t3,s1,s2,s3,p1,p2,p3,debug_message);
        }
        return d;
    }

    //! Calculate live motion.
    bool calculate_live_motion(double vo, double vm, double ve, double a, double s, smp_data &d, bool interpolate, double interval, bool debug_interpolation, bool debug_standard){

        if(calculate_motion_private(vo,vm,ve,a,s,d,debug_standard)){

            //! For live motion, we only use period t1.
            if(d.p1==period_type::acc){
                d.vi=calculate_acceleration_velocity_end(d.vo,interval,a);
                d.si=calculate_acceleration_displacement(d.vo,d.vi,a);
            }
            if(d.p1==period_type::dcc){
                d.vi=calculate_deceleration_velocity_end(d.vo,interval,a);
                d.si=calculate_deceleration_displacement(d.vo,d.vi,a);
            }
            if(d.p1==period_type::steady){
                d.vi=d.vm;
                d.si=calculate_steady_displacement(d.vi,interval);
            }
            if(d.p1==period_type::none){
                d.vi=0;
                d.si=0;
            }

            //! Limit.
            if(d.si>s){
                d.si=d.s;
                d.vi=ve;
            }
            return 1;
        } else {
            std::cerr<<"error from function : calculate_motion"<<std::endl;
            return 0;
        }
    }

    double dtg_to_stop(double vo, double a, double stop_margin=0.01){
        return calculate_deceleration_displacement(vo,0,a)+stop_margin;
    }

    //! Calculate motion.
    bool calculate_motion(double vo, double vm, double ve, double a, double s, smp_data &d, bool interpolate, double at_time, bool debug_interpolation, bool debug_standard){

        if(calculate_motion_private(vo,vm,ve,a,s,d,debug_standard)){
            std::cerr<<"calculate motion ok."<<std::endl;
            return 1;
        } else {
            std::cerr<<"error from function : calculate_motion"<<std::endl;
            return 0;
        }
    }


    //! Function elapsed time in milliseconds.
    double motion_calculation_duration_ms(){
        //! Convert from nano- to milliseconds.
        return motion_calculation_duration_nanoseconds*0.000001;
    }

    //! Function elapsed time in milliseconds.
    double interpolation_calculation_duration_ms(){
        //! Convert from nano- to milliseconds.
        return inperpolation_calculation_duration_nanoseconds*0.000001;
    }


private:

    bool calculate_motion_private(double vo, double vm, double ve, double a, double s, smp_data &d, bool debug){

        double t1=0, t2=0, t3=0, s1=0, s2=0, s3=0;
        period_type p1, p2, p3;
        p1=period_type::none;
        p2=period_type::none;
        p3=period_type::none;

        //! One period t1, acc,dcc with custom ve.
        if(!calculate_if_one_acceleration_deceleration_period_fits_displacement(vo,ve,a,s)){
            if(calculate_custom_ve(vo,ve,a,s)){
                if(calculate_acceleration_deceleration_time(vo,ve,a,t1,p1)){
                    s1=s;
                    d=setData(vo,vm,ve,a,t1,t2,t3,s1,s2,s3,p1,p2,p3,debug,"One periods t1, with custom ve.");
                    return 1;
                }
            }
        }
        //! One period t1, steady.
        if(vo==ve && vo==vm){
            if(calculate_steady_time(vo,s,t1,p1)){
                s1=s;
                d=setData(vo,vm,ve,a,t1,t2,t3,s1,s2,s3,p1,p2,p3,debug,"One period t1, steady.");
                return 1;
            }
        }
        //! Three periods, t1,t2,t3.
        if(calculate_if_two_acceleration_deceleration_period_fits_displacement(vo,vm,ve,a,s)){

            if(calculate_acceleration_deceleration_time(vo,vm,a,t1,p1)){
                if(calculate_acceleration_deceleration_displacment(vo,vm,a,s1)){
                    if(calculate_acceleration_deceleration_time(vm,ve,a,t3,p3)){
                        if(calculate_acceleration_deceleration_displacment(vm,ve,a,s3)){
                            s2=s-(s1+s3);
                            if(calculate_steady_time(vm,s2,t2,p2)){
                                d=setData(vo,vm,ve,a,t1,t2,t3,s1,s2,s3,p1,p2,p3,debug,"Three periods, t1,t2,t3");
                                return 1;
                            }
                        }
                    }
                }
            }
        }
        //! Three periods t1,t2,t3 sampled vm to fit curve.
        if(calculate_sampled_vm_periods(vo,vm,ve,a,s,t1,t2,t3,s1,s2,s3,p1,p2,p3)){
            d=setData(vo,vm,ve,a,t1,t2,t3,s1,s2,s3,p1,p2,p3,debug,"Sampled periods, t1,t2,t3");
            return 1;
        }
        std::cerr<<"error from function : calculate_motion"<<std::endl;
        return 0;
    }

    double motion_calculation_duration_nanoseconds=0;
    double inperpolation_calculation_duration_nanoseconds=0;

    bool calculate_sampled_vm_periods(double vo,double &vm,double ve,double a, double s,
                                      double &t1, double &t2, double &t3,
                                      double &s1, double &s2, double &s3,
                                      period_type &p1, period_type &p2, period_type &p3){
        double vms=vm;
        //! Sample down.
        if(vo<vm && vm>ve){
            while(1){
                if(!calculate_acceleration_deceleration_time(vo,vms,a,t1,p1)){
                    return 0;
                }
                if(!calculate_acceleration_deceleration_displacment(vo,vms,a,s1)){
                    return 0;
                }
                if(!calculate_acceleration_deceleration_time(vms,ve,a,t3,p3)){
                    return 0;
                }
                if(!calculate_acceleration_deceleration_displacment(vms,ve,a,s3)){
                    return 0;
                }
                s2=s-(s1+s3);
                if(calculate_steady_time(vms,s2,t2,p2)){
                    if(s2<0){
                        vms-=0.001;
                        if(vms<0){
                            return 0;
                        }
                    }
                    if(s2>0){
                        vm=vms;
                        return 1;
                    }
                } else {
                    return 0;
                }
            }
        }

        //! Sample up.
        if(vo>vm && ve>vm){

            while(1){
                if(!calculate_acceleration_deceleration_time(vo,vms,a,t1,p1)){
                    return 0;
                }
                if(!calculate_acceleration_deceleration_displacment(vo,vms,a,s1)){
                    return 0;
                }
                if(!calculate_acceleration_deceleration_time(vms,ve,a,t3,p3)){
                    return 0;
                }
                if(!calculate_acceleration_deceleration_displacment(vms,ve,a,s3)){
                    return 0;
                }

                s2=s-(s1+s3);
                if(calculate_steady_time(vms,s2,t2,p2)){
                    if(s2<0){
                        vms+=0.001;
                        if(vms>ve || vms>vo){
                            return 0;
                        }
                    }
                    if(s2>0){
                        vm=vms;
                        return 1;
                    }
                } else {
                    return 0;
                }
            }
        }
        return 0;
    }

    bool calculate_acceleration_deceleration_time(double vo, double ve, double a, double &t, period_type &p){
        if(vo<ve){
            t=calculate_acceleration_time(vo,ve,a);
            p=period_type::acc;
            return 1;
        }
        if(vo>ve){
            t=calculate_deceleration_time(vo,ve,a);
            p=period_type::dcc;
            return 1;
        }
        if(vo==ve){
            t=0;
            p=period_type::steady;
            return 1;
        }
        std::cerr<<"error from function: calculate_acceleration_deceleration_time"<<std::endl;
        return 0;
    }

    double calculate_acceleration_time(double vo, double ve, double a) {
        if(a == 0) {
            // if(in.debug_functions_and_motion_periods){std::cout << "Acceleration value cannot be zero\n";}
            return NAN;
        }
        if(a < 0){
            // if(in.debug_functions_and_motion_periods){std::cout << "Use a positive acceleration value" << std::endl;}
            return NAN;
        }
        if(ve <= vo) {
            // if(in.debug_functions_and_motion_periods){std::cout << "Final velocity must be greater than initial velocity\n";}
            return NAN;
        }

        return (ve - vo) / a;
    }

    double calculate_acceleration_displacement(double vo, double ve, double a) {
        if(a == 0) {
            // if(in.debug_functions_and_motion_periods){std::cout << "Acceleration value cannot be zero\n";}
            return NAN;
        }
        if(a < 0){
            // if(in.debug_functions_and_motion_periods){std::cout << "Use a positive acceleration value" << std::endl;}
            return NAN;
        }
        if(ve < vo) {
            // if(in.debug_functions_and_motion_periods){std::cout << "Final velocity must be greater than initial velocity\n";}
            return NAN;
        }
        if(ve == vo) {
            return 0;
        }
        return (ve*ve - vo*vo)/(2*a);
    }

    bool calculate_acceleration_deceleration_displacment(double vo, double ve, double a, double &s){
        if(vo<ve){
            s=calculate_acceleration_displacement(vo,ve,a);
            return 1;
        }
        if(vo>ve){
            s=calculate_deceleration_displacement(vo,ve,a);
            return 1;
        }
        if(vo==ve){
            s=0;
            return 1;
        }
        return 0;
    }

    double calculate_deceleration_displacement(double vo, double ve, double a) {
        if(a == 0) {
            std::cout << "Acceleration value cannot be zero\n";
            return NAN;
        }
        if(a < 0){
            std::cout << "Use a positive acceleration value" << std::endl;
            return NAN;
        }
        if(vo < ve) {
            std::cout << "Final velocity must be less than initial velocity\n";
            return NAN;
        }
        if(vo == ve) {
            return 0;
        }
        return (vo*vo - ve*ve)/(2*a);
    }

    bool calculate_acceleration_deceleration_velocity_at_time(double vo, double ve, double &vi, double a, double at_time){

        if(vo<ve){
            vi=calculate_acceleration_velocity_end(vo,at_time,a);
            return 1;
        }
        if(vo>ve){
            vi=calculate_deceleration_velocity_end(vo,at_time,a);
            return 1;
        }
        if(vo==ve){
            vi=vo;
            return 1;
        }
        return 0;
    }

    double calculate_deceleration_time(double vo, double ve, double a) {
        if(a == 0) {
            //if(in.debug_functions_and_motion_periods){std::cout << "Acceleration value cannot be zero" << std::endl;}
            return NAN;
        }
        if(a < 0){
            //if(in.debug_functions_and_motion_periods){std::cout << "Use a positive acceleration value" << std::endl;}
            return NAN;
        }
        if(ve > vo) {
            //if(in.debug_functions_and_motion_periods){std::cout << "Final velocity must be less than or equal to initial velocity" << std::endl;}
            return NAN;
        }

        return (vo - ve) / a;
    }

    bool calculate_steady_time(double v, double s, double &t, period_type &p) {
        if(v==0){
            std::cerr<<"error from function : calculate_steady_time, v=0"<<std::endl;
            return 0;
        }
        t=s/v;
        p=period_type::steady;
        return 1;
    }

    bool calculate_steady_displacement(double v, double t, double &si) {
        si=v*t;
        return 1;
    }

    double calculate_steady_displacement(double v, double t) {
        return v*t;
    }

    double calculate_acceleration_time_for_displacement(double vo, double a, double s) {
        return (sqrt(vo*vo + 2*a*s) - vo)/a;
    }

    double calculate_deceleration_time_for_displacement(double vo, double a, double s) {
        if(a>0){
            a=-abs(a);
        }
        return (vo - sqrt(vo*vo + 2*a*s))/-a;
    }

    double calculate_acceleration_velocity_end(double vo, double t, double a){
        return vo + a*t;
    }

    double calculate_deceleration_velocity_end(double vo, double t, double a){
        if(a>0){
            a=-abs(a);
        }
        return vo + a*t;
    }

    bool calculate_if_one_acceleration_deceleration_period_fits_displacement(double vo, double ve, double a, double s){
        double s1=0;
        if(vo==ve){
            s1=0;
        }
        if(vo>ve){
            s1=calculate_deceleration_displacement(vo,ve,a);
        }
        if(vo<ve){
            s1=calculate_acceleration_displacement(vo,ve,a);
        }
        if(s1>s){
            return false;
        }
        return true;
    }

    bool calculate_if_two_acceleration_deceleration_period_fits_displacement(double vo, double vm, double ve, double a, double s){
        double s1=0;
        double s2=0;

        if(vm==0){
            std::cerr<<"error, vm can not be zero"<<std::endl;
            return false;
        }
        //! First period.
        if(vo==vm){
            s1=0;
        }
        if(vo>vm){
            s1=calculate_deceleration_displacement(vo,vm,a);
        }
        if(vo<vm){
            s1=calculate_acceleration_displacement(vo,vm,a);
        }
        //! Second period.
        if(vm==ve){
            s2=0;
        }
        if(vm>ve){
            s2=calculate_deceleration_displacement(vm,ve,a);
        }
        if(vm<ve){
            s2=calculate_acceleration_displacement(vm,ve,a);
        }

        if(s1+s2>s){
            return false;
        }
        return true;
    }

    bool calculate_custom_ve(double vo, double &ve, double a, double s){
        double t=0;
        if(vo<ve){
            t=calculate_acceleration_time_for_displacement(vo,a,s);
            ve=calculate_acceleration_velocity_end(vo,t,a);
            return 1;
        }
        if(vo>ve){
            t=calculate_deceleration_time_for_displacement(vo,a,s);
            ve=calculate_deceleration_velocity_end(vo,t,a);
            return 1;
        }
        if(vo==ve){
            ve=vo;
            return 1;
        }
        return 0;
    }

    //! Function to check if above function output is correct.
    void check_private_functions(){
        double vo=0;
        double ve=0;
        double a=0;
        double t=0;
        double s=0;

        vo=0;
        ve=10;
        a=2;
        t= calculate_acceleration_time(vo,ve,a);
        std::cout<<"t acc:"<<t<<std::endl;

        vo=10;
        ve=0;
        a=2;
        t = calculate_deceleration_time(vo,ve,a);
        std::cout<<"t dcc:"<<t<<std::endl;

        vo=0;
        ve=10;
        a=2;
        s = calculate_acceleration_displacement(vo,ve,a);
        std::cout<<"s acc:"<<s<<std::endl;

        vo=10;
        ve=0;
        a=2;
        s = calculate_deceleration_displacement(vo,ve,a);
        std::cout<<"s dcc:"<<s<<std::endl;

        vo=0;
        a=2;
        s=100; //! s=vo*t + 0.5*a*t*t   -> t = (sqrt(vo*vo + 2*a*s) - vo)/a;
        t = calculate_acceleration_time_for_displacement(vo,a,s);
        std::cout<<"t acc:"<<t<<std::endl;
        ve = calculate_acceleration_velocity_end(vo,t,a);
        std::cout<<"ve acc:"<<ve<<std::endl;

        vo=20;
        a=2;
        s=100;
        t = calculate_deceleration_time_for_displacement(vo,a,s);
        std::cout<<"t dcc:"<<t<<std::endl;
        ve = calculate_deceleration_velocity_end(vo,t,a);
        std::cout<<"ve dcc:"<<ve<<std::endl;
    }
};

#endif // SMP_H
