#ifndef SMP_H
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
        dcc,
        steady,
        none
    };

    struct smp_data {
        //! Inputs.
        double vo=0;
        double ve=0;
        double vm=0;
        double a=0;
        double s=0;
        bool debug_functions_and_motion=0;
        bool debug_motion_function_time=0;
        bool debug_interpolation_time=0;
        bool debug_interpolation_result=0;
        //! Outputs.
        double t1=0, t2=0, t3=0;
        double s1=0, s2=0, s3=0;
        period_type p1,p2,p3;

        double ttot(){
            return t1+t2+t3;
        }
        double stot(){
            return s1+s2+s3;
        }
    };
    smp_data in,out;

    struct smp_interpolate {
        double t=0;
        double v=0;
        double a=0;
        double s=0;
    };
    smp_interpolate interpolate_result;

    void setData(smp_data theData){
        in=theData;
        in.t1=0; in.t2=0; in.t3=0;
        in.s1=0; in.s2=0; in.s3=0;
    }
    smp_data Result(){
        return out;
    }

    void interpolate_motion(double at_time){

        auto start = std::chrono::steady_clock::now();

        double vo=out.vo;
        double a=out.a;
        double t1=out.t1, t2=out.t2, t3=out.t3;
        double ttot=out.ttot();
        double s1=out.s1, s2=out.s2, s3=out.s3;
        bool debug=out.debug_functions_and_motion;
        bool debug_interpolation_time=out.debug_interpolation_time;
        bool debug_interpolation_result=out.debug_interpolation_result;
        double t=at_time;
        double v=0;
        double s=0;
        double ve=0;
        double vm=0;
        period_type p1=out.p1;
        period_type p2=out.p2;
        period_type p3=out.p3;

        //! Go on if time point is inside the motion time.
        if(t<=ttot){
            //! Period p1.
            if(t>=0 && t<t1){
                if(p1==period_type::acc){
                    v=calculate_acceleration_velocity_end(vo,t,a);
                    s=calculate_acceleration_displacement(vo,v,a);
                    a=abs(a);
                }
                if(p1==period_type::dcc){
                    v=calculate_deceleration_velocity_end(vo,t,a);
                    s=calculate_deceleration_displacement(vo,v,a);
                    a=-abs(a);
                }
            }
            //! Period p2.
            if(t>=t1 && t<=t1+t2){
                t-=t1;
                if(p2==period_type::steady){
                    v=calculate_acceleration_velocity_end(vo,t1,a);
                    s=calculate_steady_displacement(v,t);
                    a=0;
                }
                s+=s1;
            }
            //! Period p3.
            if(t>t1+t2 && t<=t1+t2+t3){
                t-=t1;
                t-=t2;
                if(p3==period_type::acc){
                    if(p1==period_type::acc){ //! Calculate period p1.
                        vm=calculate_acceleration_velocity_end(vo,t1,a);
                    }
                    if(p1==period_type::dcc){
                        vm=calculate_deceleration_velocity_end(vo,t1,a);
                    }
                    ve=calculate_acceleration_velocity_end(vm,t,a); //! Calculate period p3.
                    s=calculate_acceleration_displacement(vm,ve,a);
                    a=abs(a);
                }
                if(p3==period_type::dcc){
                    if(p1==period_type::acc){
                        vm=calculate_acceleration_velocity_end(vo,t1,a);
                    }
                    if(p1==period_type::dcc){
                        vm=calculate_deceleration_velocity_end(vo,t1,a);
                    }
                    ve=calculate_deceleration_velocity_end(vm,t,a);
                    s=calculate_deceleration_displacement(vm,ve,a);
                    a=-abs(a);
                }
                v=ve;
                s+=s1+s2;
            }
        }
        interpolate_result.v=v;
        interpolate_result.s=s;
        interpolate_result.a=a;
        interpolate_result.t=at_time;

        if(debug_interpolation_result){
            std::cout<<"v:"<<interpolate_result.v<<" ";
            std::cout<<"s:"<<interpolate_result.s<<" ";
            std::cout<<"a:"<<interpolate_result.a<<" ";
            std::cout<<"t:"<<interpolate_result.t<<" ";
        }

        auto end = std::chrono::steady_clock::now();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        inperpolation_calculation_duration_nanoseconds=nanoseconds.count();

        if(debug_interpolation_time){
            std::cout<<"interpolation ms:"<<std::fixed<<interpolation_calculation_duration_ms()<<std::endl;
        }

    }

    void calculate_motion(){

        auto start = std::chrono::steady_clock::now();

        double vo=in.vo;
        double ve=in.ve;
        double vm=in.vm;
        double a=in.a;
        double t1=0, t2=0, t3=0;
        double ttot=in.ttot();
        double s1=0, s2=0, s3=0;
        double stot=in.s;
        bool debug=in.debug_functions_and_motion;
        period_type p1=period_type::none;
        period_type p2=period_type::none;
        period_type p3=period_type::none;

        if(vo==vm && vm==ve){
            t1=0;
            s1=0;
            t3=0;
            s3=0;
            p1=period_type::none;
            p3=period_type::none;
        }
        if(vo==vm && vm<ve){
            t1=0;
            s1=0;
            t3=calculate_acceleration_time(vm,ve,a);
            s3=calculate_acceleration_displacement(vm,ve,a);
            p1=period_type::acc;
            p3=period_type::acc;
        }
        if(vo==vm && vm>ve){
            t1=0;
            s1=0;
            t3=calculate_deceleration_time(vm,ve,a);
            s3=calculate_deceleration_displacement(vm,ve,a);
            p1=period_type::none;
            p3=period_type::dcc;
        }
        if(vo<vm && vm==ve){
            t1=calculate_acceleration_time(vo,vm,a);
            s1=calculate_acceleration_displacement(vo,vm,a);
            t3=0;
            s3=0;
            p1=period_type::acc;
            p3=period_type::none;
        }
        if(vo>vm && vm==ve){
            t1=calculate_deceleration_time(vo,vm,a);
            s1=calculate_deceleration_displacement(vo,vm,a);
            t3=0;
            s3=0;
            p1=period_type::dcc;
            p3=period_type::none;
        }
        if(vo<vm && vm>ve){
            t1=calculate_acceleration_time(vo,vm,a);
            s1=calculate_acceleration_displacement(vo,vm,a);
            t3=calculate_deceleration_time(vm,ve,a);
            s3=calculate_deceleration_displacement(vm,ve,a);
            p1=period_type::acc;
            p3=period_type::dcc;
        }
        if(vo>vm && vm<ve){
            t1=calculate_deceleration_time(vo,vm,a);
            s1=calculate_deceleration_displacement(vo,vm,a);
            t3=calculate_acceleration_time(vm,ve,a);
            s3=calculate_acceleration_displacement(vm,ve,a);
            p1=period_type::dcc;
            p3=period_type::acc;
        }
        if(vo>vm && vm>ve){
            t1=calculate_deceleration_time(vo,vm,a);
            s1=calculate_deceleration_displacement(vo,vm,a);
            t3=calculate_deceleration_time(vm,ve,a);
            s3=calculate_deceleration_displacement(vm,ve,a);
            p1=period_type::dcc;
            p3=period_type::dcc;
        }
        if(vo<vm && vm<ve){
            t1=calculate_acceleration_time(vo,vm,a);
            s1=calculate_acceleration_displacement(vo,vm,a);
            t3=calculate_acceleration_time(vm,ve,a);
            s3=calculate_acceleration_displacement(vm,ve,a);
            p1=period_type::acc;
            p3=period_type::acc;
        }

        //! Normal curve with periods t1,t2,t3.
        if(s1+s3<stot){
            s2=stot-(s1+s3);
            t2=calculate_steady_time(vm,s2);
            p2=period_type::steady;
        }

        if((s1+s3)>stot){

            if(vo<vm && vm>ve){
                double vms=vm;
                while((stot-(s1+s3))<0){
                    t1=calculate_acceleration_time(vo,vms,a);
                    s1=calculate_acceleration_displacement(vo,vms,a);
                    t3=calculate_deceleration_time(vms,ve,a);
                    s3=calculate_deceleration_displacement(vms,ve,a);
                    vms-=0.001;
                }
                s2=stot-(s1+s3);
                t2=calculate_steady_time(vms,s2);

                if(!isnan(s2)){
                    vm=vms;
                    p1=period_type::acc;
                    p2=period_type::steady;
                    p3=period_type::dcc;
                }

                //! Still no solution after sampling. Create a single acc move.
                if(isnan(s2)){
                    if(vo>ve){
                        if(debug){std::cout<<"sampling not possible, creating a single dcc move with new ve."<<std::endl;}
                        t1=0;
                        s1=0;
                        t2=0;
                        s2=0;
                        s3=stot;
                        t3=calculate_deceleration_time_for_displacement(vo,a,stot);
                        ve=calculate_deceleration_velocity_end(vo,t3,a);
                        p1=period_type::none;
                        p2=period_type::none;
                        p3=period_type::dcc;
                    }
                    if(vo<ve){
                        if(debug){std::cout<<"sampling not possible, creating a single acc move with new ve."<<std::endl;}
                        t1=0;
                        s1=0;
                        t2=0;
                        s2=0;
                        s3=stot;
                        t3=calculate_acceleration_time_for_displacement(vo,a,stot);
                        ve=calculate_acceleration_velocity_end(vo,t3,a);
                        p1=period_type::none;
                        p2=period_type::none;
                        p3=period_type::acc;
                    }
                }
            }

            if(vo>vm && vm<ve){
                double vms=vm;
                while((stot-(s1+s3))<0){
                    t1=calculate_deceleration_time(vo,vms,a);
                    s1=calculate_deceleration_displacement(vo,vms,a);
                    t3=calculate_acceleration_time(vms,ve,a);
                    s3=calculate_acceleration_displacement(vms,ve,a);
                    vms+=0.001;
                }
                s2=stot-(s1+s3);
                t2=calculate_steady_time(vms,s2);

                if(!isnan(s2)){
                    if(debug){std::cout<<"sampling ok."<<std::endl;}
                    vm=vms;
                    p1=period_type::dcc;
                    p2=period_type::steady;
                    p3=period_type::acc;
                }

                //! Still no solution after sampling. Create a single move.
                if(isnan(s2)){
                    if(vo>ve){
                        if(debug){std::cout<<"sampling not possible, creating a single dcc move with new ve."<<std::endl;}
                        t1=0;
                        s1=0;
                        t2=0;
                        s2=0;
                        s3=stot;
                        t3=calculate_deceleration_time_for_displacement(vo,a,stot);
                        ve=calculate_deceleration_velocity_end(vo,t3,a);
                        p1=period_type::none;
                        p2=period_type::none;
                        p3=period_type::dcc;
                    }
                    if(vo<ve){
                        if(debug){std::cout<<"sampling not possible, creating a single acc move with new ve."<<std::endl;}
                        t1=0;
                        s1=0;
                        t2=0;
                        s2=0;
                        s3=stot;
                        t3=calculate_acceleration_time_for_displacement(vo,a,stot);
                        ve=calculate_acceleration_velocity_end(vo,t3,a);
                        p1=period_type::none;
                        p2=period_type::none;
                        p3=period_type::acc;
                    }
                }
            }

            if(vo>vm && vm>ve){
                if(debug){std::cout<<"creating a single dcc move, new ve."<<std::endl;}
                t1=0;
                s1=0;
                t2=0;
                s2=0;
                s3=stot;
                t3=calculate_deceleration_time_for_displacement(vo,a,stot);
                ve=calculate_deceleration_velocity_end(vo,t3,a);
                p1=period_type::none;
                p2=period_type::none;
                p3=period_type::dcc;

            }

            if(vo<vm && vm<ve){
                if(debug){std::cout<<"creating a single acc move, new ve."<<std::endl;}
                t1=0;
                s1=0;
                t2=0;
                s2=0;
                s3=stot;
                t3=calculate_acceleration_time_for_displacement(vo,a,stot);
                ve=calculate_acceleration_velocity_end(vo,t3,a);
                p1=period_type::none;
                p2=period_type::none;
                p3=period_type::acc;
            }
        }

        if(debug){
            std::cout<<"t1:"<<t1<<std::endl;
            std::cout<<"t2:"<<t2<<std::endl;
            std::cout<<"t3:"<<t3<<std::endl;
            std::cout<<"ttot:"<<t1+t2+t3<<std::endl;
            std::cout<<"l1:"<<s1<<std::endl;
            std::cout<<"l2:"<<s2<<std::endl;
            std::cout<<"l3:"<<s3<<std::endl;
            std::cout<<"ltot:"<<s1+s2+s3<<std::endl;
            std::cout<<"vo:"<<vo<<std::endl;
            std::cout<<"vm:"<<vm<<std::endl;
            std::cout<<"ve:"<<ve<<std::endl;
        }

        out.t1=t1;
        out.t2=t2;
        out.t3=t3;
        out.s1=s1;
        out.s2=s2;
        out.s3=s3;
        out.a=a;
        out.vo=vo;
        out.vm=vm;
        out.ve=ve;
        out.p1=p1;
        out.p2=p2;
        out.p3=p3;
        out.debug_functions_and_motion=debug;

        auto end = std::chrono::steady_clock::now();
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        motion_calculation_duration_nanoseconds=nanoseconds.count();
        if(in.debug_motion_function_time){
            std::cout<<"duration motion planning milliseconds:"<<std::fixed<<motion_calculation_duration_ms()<<std::endl;
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

    double motion_calculation_duration_nanoseconds=0;
    double inperpolation_calculation_duration_nanoseconds=0;

    double calculate_acceleration_time(double vo, double ve, double a) {
        if(a == 0) {
            if(in.debug_functions_and_motion){std::cout << "Acceleration value cannot be zero\n";}
            return NAN;
        }
        if(a < 0){
            if(in.debug_functions_and_motion){std::cout << "Use a positive acceleration value" << std::endl;}
            return NAN;
        }
        if(ve <= vo) {
            if(in.debug_functions_and_motion){std::cout << "Final velocity must be greater than initial velocity\n";}
            return NAN;
        }

        return (ve - vo) / a;
    }

    double calculate_acceleration_displacement(double vo, double ve, double a) {
        if(a == 0) {
            if(in.debug_functions_and_motion){std::cout << "Acceleration value cannot be zero\n";}
            return NAN;
        }
        if(a < 0){
            if(in.debug_functions_and_motion){std::cout << "Use a positive acceleration value" << std::endl;}
            return NAN;
        }
        if(ve < vo) {
            if(in.debug_functions_and_motion){std::cout << "Final velocity must be greater than initial velocity\n";}
            return NAN;
        }
        if(ve == vo) {
            return 0;
        }
        return (ve*ve - vo*vo)/(2*a);
    }

    double calculate_deceleration_displacement(double vo, double ve, double a) {
        if(a == 0) {
            if(in.debug_functions_and_motion){std::cout << "Acceleration value cannot be zero\n";}
            return NAN;
        }
        if(a < 0){
            if(in.debug_functions_and_motion){std::cout << "Use a positive acceleration value" << std::endl;}
            return NAN;
        }
        if(vo < ve) {
            if(in.debug_functions_and_motion){std::cout << "Final velocity must be less than initial velocity\n";}
            return NAN;
        }
        if(vo == ve) {
            return 0;
        }
        return (vo*vo - ve*ve)/(2*a);
    }

    double calculate_deceleration_time(double vo, double ve, double a) {
        if(a == 0) {
            if(in.debug_functions_and_motion){std::cout << "Acceleration value cannot be zero" << std::endl;}
            return NAN;
        }
        if(a < 0){
            if(in.debug_functions_and_motion){std::cout << "Use a positive acceleration value" << std::endl;}
            return NAN;
        }
        if(ve > vo) {
            if(in.debug_functions_and_motion){std::cout << "Final velocity must be less than or equal to initial velocity" << std::endl;}
            return NAN;
        }

        return (vo - ve) / a;
    }

    double calculate_steady_time(double v, double s) {
        return s/v;
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
