#ifndef SCP_H
#define SCP_H

#include <iostream>
#include <math.h>
#include <cmath>

class scp
{
public:
    scp(){};

    //! Examples
    void examples(){
        //! example_motion_from_zero_to_vm_to_ve();
        //! example_get_curve_start_and_end_time_using_acceleration_begin_and_end_values_for_acceleration_curve();
        //! example_get_curve_start_and_end_time_using_acceleration_begin_and_end_values_for_deceleration_curve();
        //! example_motion_from_zero_to_vm();
        //! example_motion_from_zero_to_vm_to_ve();
    }
    //! Print interpolated curve outputs.
    void print(double si, double vi, double ai, double at_time){
        std::cout<<"s:"<<si<<" v:"<<vi<<" a:"<<ai<<" t:"<<at_time<<std::endl;
    }
    //! Calculate a scurve for deceleration period, including the concave + convex periods. This is a mirrored acc curve algo.
    //! Inputs: "vo" velocity begin, "ve" velocity end, "a" acceleration, "at_time" interpolate at time.
    //! Get the results by reference, "si" interpolated displacement, "vi" velocity interpolated, "ai" acceleration interpolated.
    //! Assuming the curve starts at acceleration 0, and ends with acceleration 0.
    bool calculate_dcc_curve_s_v_a_given_time_point(double vo, double ve, double a, double at_time, double &si, double &vi, double &ai){
        double t1=0;
        double th=0;

        //! For deceleration period, use vo>ve.
        if(vo<ve){
            std::cerr<<"vo<ve for deceleration period, Error from function : calculate_s_v_a_at_time_deceleration_period."<<std::endl;
            return 0;
        }
        //! Get period time "t1".
        if(!calculate_curve_time(vo,ve,a,t1,th)){
            std::cerr<<"Error from function : calculate_s_v_a_at_time_deceleration_period."<<std::endl;
            return 0;
        }
        //! Acceleration can not be zero.
        if(a==0){
            std::cerr<<"no acceleration given, Error from function : calculate_s_v_a_at_time_deceleration_period."<<std::endl;
            return 0;
        }
        //! Use positive values for a.
        if(a<0){
            a=abs(a);
        }
        //! Get the period displacement "s1".
        double s1=0;
        if(!calculate_curve_displacment(vo,ve,a,s1)){
            std::cerr<<"Error from function : calculate_s_v_a_at_time_deceleration_period."<<std::endl;
            return 0;
        }
        //! Mirror the acceleration curve.
        if(!calculate_acc_curve_s_v_a_given_time_point(ve,vo,a,t1-at_time,si,vi,ai)){
            std::cerr<<"Error from function : calculate_s_v_a_at_time_deceleration_period."<<std::endl;
            return 0;
        }
        //! Mirror displacement.
        si=s1-si;
        //! Negative acceleration.
        ai=-abs(ai);

        return 1;
    }
    //! Calculate a scurve for acceleration period, including the concave + convex periods.
    //! Inputs: "vo" velocity begin, "ve" velocity end, "a" acceleration, "at_time" interpolate at time.
    //! Get the results by reference, "si" interpolated displacement, "vi" velocity interpolated, "ai" acceleration interpolated.
    //! Assuming the curve starts at acceleration 0, and ends with acceleration 0.
    bool calculate_acc_curve_s_v_a_given_time_point(double vo, double ve, double a, double at_time, double &si, double &vi, double &ai){
        double t1=0;
        double th=0;
        double t=0;
        if(!calculate_curve_time(vo,ve,a,t1,th)){
            std::cerr<<"t1,th, error from function: calculate_s_v_a_at_time_acceleration_period."<<std::endl;
            return 0;
        }
        //! For acceleration use vo<ve.
        if(vo>ve){
            std::cerr<<"vo>ve for acceleration period, error from function: calculate_s_v_a_at_time_acceleration_period."<<std::endl;
            return 0;
        }
        //! Acceleration can not be zero.
        if(a==0){
            std::cerr<<"no acceleration given, error from function: calculate_s_v_a_at_time_acceleration_period."<<std::endl;
            return 0;
        }
        //! Use a positive value for a.
        if(a<0){
            a=abs(a);
        }

        double stot=0;
        if(!calculate_curve_displacment(vo,ve,a,stot)){
            std::cerr<<"stot, error from function: calculate_s_v_a_at_time_acceleration_period."<<std::endl;
            return 0;
        }

        std::cout<<"time:"<<at_time<<" t1:"<<t1<<std::endl;

        //! Max acceleration at inflection point. [2*A]
        double as=2*a;
        //! Max jerk. Document page 2.
        double jm=2*as/t1;
        //! Velocity at 50% of curve time.
        double vh=vo+jm*(th*th)/2;
        //! Displacement at 50% of curve time.
        double s1=vo*th+jm*(th*th*th)/6;

        t=at_time;
        //! Concave period, curve up.
        if(t<th){
            vi=vo+jm*(t*t)/2;                       //! Velocity current.
            si=vo*t+jm*(t*t*t)/6;                   //! Displacment.
            ai=jm*t;                                //! Acceleration.
            return 1;
        }
        //! At inflection point.
        if(t==th){
            vi=vh;
            si=s1;
            ai=as;
            return 1;
        }
        //! Convex period, curve down.
        if(t>th && t<t1){
            t-=th;                                  //! Substract concave time.
            vi=vh + as*t - jm*(t*t)/2;              //! Velocity.
            si=vh*t + as*(t*t)/2 - jm*(t*t*t)/6;    //! Displacment convex curve.
            si+=s1;                                 //! Add the concave displacment.
            ai=as-jm*t;                             //! Acceleration.

            //! Verify formula ok.
            //! double ti=0;
            //! calculate_time_acceleration_end(vo,ve,a,ai,ti);
            //! std::cout<<"ti:"<<ti<<std::endl;
            return 1;
        }
        //! Absolute end values.
        if(t>=t1){
            vi=ve;
            si=stot;
            ai=0;
            return 1;
        }
        //! std::cerr<<"Error from function : calculate_displacement_at_time_acceleration_period."<<std::endl;
        std::cout<<"error, reached end of function"<<std::endl;
        return 0;
    }
    //! Calculate jerk max, given "a" acceleration, "t1" time concove + convex.
    bool calculate_jm(double a, double t1, double &jm){
        double as=2*a;
        jm=2*as/t1;
        return 1;
    }
    //! For a deceleration curve wich includes the concave & convex periods,
    //! calculate the "ti" time point for a given "acs" acceleration start value.
    bool calculate_time_point_acs_dcc_curve(double vo, double ve, double a, double acs, double &ti) {

        double t1=0, th=0;
        double as=2*a;

        if(acs<0){
            acs=abs(acs);
        }
        if(acs>as){
            std::cerr<<"a_start>as, error"<<std::endl;
            return 0;
        }
        //! Calculate t1,th.
        calculate_curve_time(vo,ve,a,t1,th);
        //! Max jerk. Document page 2.
        double jm=2*as/t1;

        ti=(as-acs)/jm;
        ti=th-ti;
        return 1;
    }
    //! For a deceleration curve wich includes the concave & convex periods,
    //! calculate the "ti" time point for a given "ace" acceleration end value.
    bool calculate_time_point_ace_dcc_curve(double vo, double ve, double a, double ace, double &ti) {

        double t1=0, th=0;
        double as=2*a;

        if(ace<0){
            ace=abs(ace);
        }
        if(ace>as){
            std::cerr<<"a_start>as, error"<<std::endl;
            return 0;
        }
        //! Calculate t1,th.
        calculate_curve_time(vo,ve,a,t1,th);
        //! Max jerk. Document page 2.
        double jm=2*as/t1;

        ti=ace/jm;
        ti=th-ti;
        ti+=th;
        return 1;
    }
    //! For a acceleration curve wich includes the concave & convex periods,
    //! calculate the "ti" time point for a given "acs" acceleration start value.
    bool calculate_time_point_acs_acc_curve(double vo, double ve, double a, double acs, double &ti) {

        double t1=0, th=0;
        double as=2*a;

        if(acs>as){
            std::cerr<<"a_start>as, error"<<std::endl;
            return 0;
        }
        //! Calculate t1,th.
        calculate_curve_time(vo,ve,a,t1,th);
        //! Max jerk. Document page 2.
        double jm=2*as/t1;

        ti=acs/jm;
        return 1;
    }
    //! For a acceleration curve wich includes the concave & convex periods,
    //! calculate the "ti" time point for a given "ace" acceleration end value.
    bool calculate_time_point_ace_acc_curve(double vo, double ve, double a, double a_end, double &ti) {
        double t1=0, th=0;
        double as=2*a;

        if(a_end>as){
            std::cerr<<"a_end>as, error"<<std::endl;
            return 0;
        }
        //! Calculate t1,th.
        calculate_curve_time(vo,ve,a,t1,th);
        //! Max jerk. Document page 2.
        double jm=2*as/t1;

        ti=(as-a_end)/jm;
        ti+=th;
        return 1;
    }
    //! Calculate "s" displacment at cruise speed.
    bool calculate_steady_displacment(double v, double t, double &s){
        s=v*t;
        return 1;
    }
    //! Calculate "t" time at cruise speed.
    bool calculate_steady_time(double v, double s, double &t){
        t=s/v;
        return 1;
    }
    //! Calculate "v" velocity at cruise speed.
    bool calculate_steady_velocity(double s, double t, double &v){
        v=s/t;
        return 1;
    }
    //! May input acceleratoin or deceleration curves.
    bool calculate_scurve_total_s_t_jm(double vo, double ve, double a, double &stot, double &ttot, double &jm){
        if(!calculate_curve_displacment(vo,ve,a,stot)){
            std::cerr<<"error from function: calculate_acceleration_deceleration_period_s_t_jm."<<std::endl;
            return 0;
        }
        double th=0;
        if(!calculate_curve_time(vo,ve,a,ttot,th)){
            std::cerr<<"error from function: calculate_acceleration_deceleration_period_s_t_jm."<<std::endl;
            return 0;
        }
        if(!calculate_jm(a,ttot,jm)){
            std::cerr<<"error from function: calculate_acceleration_deceleration_period_s_t_jm."<<std::endl;
            return 0;
        }
        return 1;
    }
    //! Calculate displacement for scurve including the concave & convex periods.
    //! Inputs of type acc, dcc are valid.
    bool calculate_curve_displacment(double vo, double ve, double a, double &s){
        if(a==0){
            std::cerr<<"No acceleration given, error from function : calculate_acceleration_deceleration_period_displacment."<<std::endl;
            return 0;
        }
        if(a<0){
            a=abs(a);
        }
        //! Steady.
        if(vo==ve){
            s=0;
            return 1;
        }
        //! Acceleration.
        if(vo<ve){
            s=(ve*ve - vo*vo)/(2*a);
            return 1;
        }
        //! Deceleration.
        if(vo>ve){
            s=(vo*vo - ve*ve)/(2*a);
            return 1;
        }
        std::cerr<<"Error from function : calculate_acceleration_deceleration_period_displacment."<<std::endl;
        return 0;
    }
    //! Calculate curve time, including concave + convex periods. Input may be of type acc, dcc.
    bool calculate_curve_time(double vo, double ve, double a, double &time_period, double &time_half_period){
        if(a==0){
            std::cerr<<"No acceleration given, error from function : calculate_acceleration_deceleration_period_time."<<std::endl;
            return 0;
        }
        if(a<0){
            a=abs(a);
        }
        //! Steady.
        if(vo==ve){
            time_period=0;
            time_half_period=0;
            return 1;
        }
        //! Acceleration.
        if(vo<ve){
            time_period=(ve-vo)/a;
            time_half_period=0.5*time_period;
            return 1;
        }
        //! Deceleration.
        if(vo>ve){
            time_period=(vo-ve)/a;
            time_half_period=0.5*time_period;
            return 1;
        }
        std::cerr<<"Error from function : calculate_acceleration_deceleration_period_time."<<std::endl;
        return 0;
    }
    //! Calculate netto curve time for a accelerating scurve containing concave + convex period using "acs" acceleration start, "ace" acceleration end.
    bool calculate_curve_time_given_acs_ace_for_acc_curve(double vo, double ve, double a, double a_start, double a_end, double &ttot, bool debug){
        double t1=0, t2=0;

        if(!calculate_time_point_acs_acc_curve(vo,ve,a,a_start,t1)){
            return 0;
        }
        if(debug){
            std::cout<<"t1:"<<t1<<" a_start:"<<a_start<<std::endl;
        }

        if(!calculate_time_point_ace_acc_curve(vo,ve,a,a_end,t2)){
            return 0;
        }
        if(debug){
            std::cout<<"t2:"<<t2<<" a_end:"<<a_end<<std::endl;
        }

        ttot=t2-t1;
        return 1;
    }
    //! Calculate netto curve time for a deceleration scurve containing concave + convex period using "acs" acceleration start, "ace" acceleration end.
    bool calculate_curve_time_given_acs_ace_for_dcc_curve(double vo, double ve, double a, double a_start, double a_end, double &ttot, bool debug){
        double t1=0, t2=0;

        if(!calculate_time_point_acs_dcc_curve(vo,ve,a,a_start,t1)){
            return 0;
        }
        if(debug){
            std::cout<<"t1:"<<t1<<" a_start:"<<a_start<<std::endl;
        }

        if(!calculate_time_point_ace_dcc_curve(vo,ve,a,a_end,t2)){
            return 0;
        }
        if(debug){
            std::cout<<"t2:"<<t2<<" a_end:"<<a_end<<std::endl;
        }

        ttot=t2-t1;
        return 1;
    }
    //! Using concave + convex periods, vo<ve.
    void example_get_curve_start_and_end_time_using_acceleration_begin_and_end_values_for_acceleration_curve(){

        double vo=0;
        double ve=10;
        double a=2;
        double a_start=2;
        double a_end=0;
        double ti=0;
        double ttot=0, th=0;

        double at_time=0, si=0, vi=0, ai=0;

        //! Calculate "ttot" total time, time half "th".
        calculate_curve_time(vo,ve,a,ttot,th);
        std::cout<<"ttot:"<<ttot<<std::endl;

        //! Acceleration start input, calculates the time in the concave part of curve.
        calculate_time_point_acs_acc_curve(vo,ve,a,a_start,ti);
        std::cout<<"ti:"<<ti<<" a_start:"<<a_start<<std::endl;
        //! Verify :
        at_time=ti;
        calculate_acc_curve_s_v_a_given_time_point(vo,ve,a,at_time,si,vi,ai);
        std::cout<<"verify at time:"<<at_time<<" a:"<<ai<<std::endl;

        //! Acceleration end input, calculates the time in the convex part of curve.
        calculate_time_point_ace_acc_curve(vo,ve,a,a_end,ti);
        std::cout<<"ti:"<<ti<<" a_end:"<<a_end<<std::endl;
        //! Verify :
        at_time=ti;
        calculate_acc_curve_s_v_a_given_time_point(vo,ve,a,at_time,si,vi,ai);
        std::cout<<"verify at time:"<<at_time<<" a:"<<ai<<std::endl;

    }
    //! Using concave + convex periods, vo>ve.
    void example_get_curve_start_and_end_time_using_acceleration_begin_and_end_values_for_deceleration_curve(){

        double vo=10;
        double ve=0;
        double a=2;
        double a_start=2.5;
        double a_end=1;
        double ti=0;
        double ttot=0, th=0;

        double at_time=0, si=0, vi=0, ai=0;

        //! Calculate "ttot" total time, time half "th".
        calculate_curve_time(vo,ve,a,ttot,th);
        std::cout<<"ttot:"<<ttot<<std::endl;

        //! Mention this is the convex type used to calculate the start time.
        calculate_time_point_acs_dcc_curve(vo,ve,a,a_start,ti);
        std::cout<<"ti:"<<ti<<" a_start:"<<-abs(a_start)<<std::endl;
        //! Verify :
        at_time=ti;
        calculate_dcc_curve_s_v_a_given_time_point(vo,ve,a,at_time,si,vi,ai);
        std::cout<<"verify at time:"<<at_time<<" a:"<<ai<<std::endl;

        //! Mention this is the concave type, used to calculate the end time.
        calculate_time_point_ace_dcc_curve(vo,ve,a,a_end,ti);
        std::cout<<"ti:"<<ti<<" a_end:"<<-abs(a_end)<<std::endl;
        //! Verify:
        at_time=ti;
        calculate_dcc_curve_s_v_a_given_time_point(vo,ve,a,at_time,si,vi,ai);
        std::cout<<"verify at time:"<<at_time<<" a:"<<ai<<std::endl;
    }
    //! Calculate motion from zero velocity to cruise velocity.
    void example_motion_from_zero_to_vm(){

        double vo=0.0;
        double vm=10.0;
        double a=2.0;

        double si=0.0, vi=0.0, ai=0.0, t1=0.0, t2=0.0, th=0.0;
        calculate_curve_time(vo,vm,a,t1,th);
        std::cout<<"t1:"<<t1<<std::endl;

        std::cout<<""<<std::endl;
        std::cout<<"Acceleration curve:"<<std::endl;
        for(double i=0; i<=t1; i+=0.001){
            if(!calculate_acc_curve_s_v_a_given_time_point(vo,vm,a,i,si,vi,ai)){
                std::cerr<<"error."<<std::endl;
            }
            print(si,vi,ai,i);
        }
        //! Absolute end.
        if(!calculate_acc_curve_s_v_a_given_time_point(vo,vm,a,t1,si,vi,ai)){
            std::cerr<<"error."<<std::endl;
        }
        print(si,vi,ai,t1);
    }
    //! Calculate motion from zero velocity to cruise velocity to end velocity.
    void example_motion_from_zero_to_vm_to_ve(){
        double vo=0.0;
        double vm=10.0;
        double ve=0.0;
        double a=2.0;

        double sai=0.0, sbi=0.0, vi=0.0, ai=0.0, t1=0.0, t2=0.0, th=0.0;
        calculate_curve_time(vo,vm,a,t1,th);
        calculate_curve_time(vm,ve,a,t2,th);
        std::cout<<"t1:"<<t1<<std::endl;
        std::cout<<"t2:"<<t2<<std::endl;

        std::cout<<""<<std::endl;
        std::cout<<"Acceleration curve:"<<std::endl;
        for(double i=0.0; i<=t1; i+=0.1){
            if(!calculate_acc_curve_s_v_a_given_time_point(vo,vm,a,i,sai,vi,ai)){
                std::cerr<<"error."<<std::endl;
            } else {
                print(sai,vi,ai,i);
            }
        }

        std::cout<<""<<std::endl;
        std::cout<<"Deceleration curve:"<<std::endl;
        for(double i=0.0; i<=t1; i+=0.1){
            if(!calculate_dcc_curve_s_v_a_given_time_point(vm,ve,a,i,sbi,vi,ai)){
                std::cerr<<"error."<<std::endl;
            } else {
                print(sai+sbi,vi,ai,i);
            }
        }
    }
};

#endif






