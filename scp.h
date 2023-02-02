#ifndef SCP_H
#define SCP_H

#include <iostream>
#include <math.h>
#include <cmath>
#include <smp.h>

class scp
{
public:

    //! Todo:
    //!
    //! 1. Create function: curve displacment for acs & ace values.
    //! 2. For scurve, make a extra front period to catch up velocity interupts?
    //!
    scp(){};

    //! Examples
    void examples(){
        example_try_if_curve_fits_displacement();
        //! example_motion_from_zero_to_vm_to_ve();
        //! example_get_curve_start_and_end_time_using_acceleration_begin_and_end_values_for_acceleration_curve();
        //! example_get_curve_start_and_end_time_using_acceleration_begin_and_end_values_for_deceleration_curve();
        //! example_motion_from_zero_to_vm();
        //! example_motion_from_zero_to_vm_to_ve();
    }
    //! Print interpolated curve outputs.
    void print(double si, double vi, double ai, double at_time){
        std::cout<<std::fixed<<"s:"<<si<<" v:"<<vi<<" a:"<<ai<<" t:"<<at_time<<std::endl;
    }

    //! Generalized function to interpolate scurve at a given time-point.
    //! Inputs: "vo" velocity begin, "ve" velocity end, "a" acceleration, "at_time" interpolate at time.
    //! Get the results by reference, "si" interpolated displacement, "vi" velocity interpolated, "ai" acceleration interpolated.
    //! Assuming the curve starts at acceleration 0, and ends with acceleration 0.
    bool calculate_acc_dcc_curve_s_v_a_given_time_point(double vo, double ve, double a, double at_time, double &si, double &vi, double &ai){
        //! Type acc.
        if(vo<ve){
            if(!calculate_acc_curve_s_v_a_given_time_point(vo,ve,a,at_time,si,vi,ai)){
                std::cerr<<"error constructing acc curve."<<std::endl;
                return 0;
            }
        }
        //! Type dcc.
        if(vo>ve){
            if(!calculate_dcc_curve_s_v_a_given_time_point(vo,ve,a,at_time,si,vi,ai)){
                std::cerr<<"error constructing dcc curve."<<std::endl;
                return 0;
            }
        }
        if(vo==ve){
            std::cerr<<"error: use steady curve."<<std::endl;
            return 0;
        }
        return 1;
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
    bool calculate_time_point_ace_acc_curve(double vo, double ve, double a, double ace, double &ti) {
        double t1=0, th=0;
        double as=2*a;

        if(ace>as){
            std::cerr<<"a_end>as, error"<<std::endl;
            return 0;
        }
        //! Calculate t1,th.
        calculate_curve_time(vo,ve,a,t1,th);
        //! Max jerk. Document page 2.
        double jm=2*as/t1;

        ti=(as-ace)/jm;
        ti+=th;
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
    //! Calculate the steady parameters s2,t2 for a motion block if s1,t1 & s3,t3 parameters are known.
    //! Given "v" steady velocity, "stot" total displacment.
    //! Results by reference : "s2" displacement steady period, "t2" time steady period.
    //! Returns 0 if s2<0.
    bool calculate_steady_time_displacment(double v, double stot, double s1, double s3, double &s2, double &t2){
        s2=stot-(s1+s3);
        if(s2>0){
            smp::period_type p2;
            //! Using the smp here to avoid function duplication.
            smp().calculate_steady_time(v,s2,t2,p2);
            return 1;
        }
        return 0;
    }
    //! Calculate total displacement and total curve time for scurve including the concave & convex periods.
    //! Inputs of type acc, dcc are valid.
    bool calculate_curve_time_displacment(double vo, double ve, double a, double &s, double &t){
        if(!calculate_curve_displacment(vo,ve,a,s)){
            std::cerr<<"Error from function : calculate_curve_displacment_curve_time."<<std::endl;
            return 0;
        }
        double th=0;
        if(!calculate_curve_time(vo,ve,a,t,th)){
            std::cerr<<"Error from function : calculate_curve_displacment_curve_time."<<std::endl;
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
    bool calculate_curve_time_given_acs_ace_for_acc_curve(double vo, double ve, double a, double acs, double ace, double &ttot, bool debug){
        double t1=0, t2=0;

        if(!calculate_time_point_acs_acc_curve(vo,ve,a,acs,t1)){
            return 0;
        }
        if(debug){
            std::cout<<"t1:"<<t1<<" acs:"<<acs<<std::endl;
        }

        if(!calculate_time_point_ace_acc_curve(vo,ve,a,ace,t2)){
            return 0;
        }
        if(debug){
            std::cout<<"t2:"<<t2<<" ace:"<<ace<<std::endl;
        }

        ttot=t2-t1;
        return 1;
    }
    //! Calculate netto curve time for a deceleration scurve containing concave + convex period using "acs" acceleration start, "ace" acceleration end.
    bool calculate_curve_time_given_acs_ace_for_dcc_curve(double vo, double ve, double a, double acs, double ace, double &ttot, bool debug){
        double t1=0, t2=0;

        if(!calculate_time_point_acs_dcc_curve(vo,ve,a,acs,t1)){
            return 0;
        }
        if(debug){
            std::cout<<"t1:"<<t1<<" acs:"<<acs<<std::endl;
        }
        if(!calculate_time_point_ace_dcc_curve(vo,ve,a,ace,t2)){
            return 0;
        }
        if(debug){
            std::cout<<"t2:"<<t2<<" ace:"<<ace<<std::endl;
        }

        ttot=t2-t1;
        return 1;
    }
    //! Calculate netto curve time for a acc,dcc scurve containing concave + convex period using "acs" acceleration start, "ace" acceleration end.
    //! This works for acc,dcc curves.
    //! By reference output : "sl" left displacment, "sm" mid displacement, "sr" right displacement.
    bool calculate_curve_time_given_acs_ace_for_acc_dcc_curve(double vo, double ve, double a, double acs, double ace, bool debug,
                                                              double &sl, double &sm, double &sr,
                                                              double &v1, double &v2,
                                                              double &t1, double &t2,
                                                              double &a1, double &a2,
                                                              double stot, double ttot){

        double s1=0, s2=0;
        //! Calculate "ttot" total time, "stot" total displacement.
        calculate_curve_time_displacment(vo,ve,a,stot,ttot);
        if(debug){std::cout<<"ttot:"<<ttot<<" stot:"<<stot<<std::endl;}

        //! Acc curve.
        if(vo<ve){
            if(debug){std::cout<<"curve of type : acc."<<std::endl;}
            //! Acs input.
            if(!calculate_time_point_acs_acc_curve(vo,ve,a,acs,t1)){
                std::cerr<<"error."<<std::endl;
                return 0;
            }
            if(!calculate_acc_curve_s_v_a_given_time_point(vo,ve,a,t1,s1,v1,a1)){
                std::cerr<<"error."<<std::endl;
                return 0;
            }
            //! Ace input.
            if(!calculate_time_point_ace_acc_curve(vo,ve,a,ace,t2)){
                return 0;
            }
            if(!calculate_acc_curve_s_v_a_given_time_point(vo,ve,a,t2,s2,v2,a2)){
                std::cerr<<"error."<<std::endl;
                return 0;
            }
        }
        //! Dcc curve.
        if(vo>ve){
            if(debug){std::cout<<"curve of type : dcc."<<std::endl;}
            //! Acs input.
            if(!calculate_time_point_acs_dcc_curve(vo,ve,a,acs,t1)){
                std::cerr<<"error."<<std::endl;
                return 0;
            }
            if(!calculate_dcc_curve_s_v_a_given_time_point(vo,ve,a,t1,s1,v1,a1)){
                std::cerr<<"error."<<std::endl;
                return 0;
            }
            //! Ace input.
            if(!calculate_time_point_ace_dcc_curve(vo,ve,a,ace,t2)){
                std::cerr<<"error."<<std::endl;
                return 0;
            }
            if(!calculate_dcc_curve_s_v_a_given_time_point(vo,ve,a,t2,s2,v2,a2)){
                std::cerr<<"error."<<std::endl;
                return 0;
            }
        }
        //! Dcc curve.
        if(vo==ve){
            std::cerr<<"error for calculating acs,ace, vo==ve"<<std::endl;
            return 0;
        }

        //! Calculate displacment results.
        sl=s1;
        sm=s2-s1;
        sr=stot-s2;

        if(debug){
            std::cout<<"results for acs:"<<acs<<" t1:"<<t1<<" a:"<<a1<<" v:"<<v1<<" s:"<<s1<<std::endl;
            std::cout<<"results for ace:"<<ace<<" t2:"<<t2<<" a:"<<a2<<" v:"<<v2<<" s:"<<s2<<std::endl;

            std::cout<<"displacmenent left,  sl:"<<sl<<std::endl;
            std::cout<<"displacmenent mid,   sm:"<<sm<<std::endl;
            std::cout<<"displacmenent right, sr:"<<sr<<std::endl;
        }
        return 1;
    }

    //! For scurve motion block's are predifined.
    bool calculate_scurve_live_motion(smp::smp_data d, double at_time, double &si, double &vi, double &ai){

        double vo=d.vo;
        double vm=d.vm;
        double ve=d.ve;
        double a=d.a;
        double t=at_time;
        si=0, vi=0, ai=0;

        //! Period t1, possible type acc,dcc,steady,none.
        if(t<d.t1){
            if(d.p1==smp::period_type::acc || d.p1==smp::period_type::dcc){

                //! If just one period, move directly to ve.
                if(d.p2==smp::period_type::none && d.p3==smp::period_type::none){
                    calculate_acc_dcc_curve_s_v_a_given_time_point(vo,ve,a,t,si,vi,ai);
                }

                //! Move to vm in acc period.
                if(d.p2!=smp::period_type::none && d.p3!=smp::period_type::none){
                    calculate_acc_dcc_curve_s_v_a_given_time_point(vo,vm,a,t,si,vi,ai);
                }

            }
            if(d.p1==smp::period_type::steady){
                vi=vm;
                ai=0;
                smp().calculate_steady_displacement(vi,t,si);
            }
            return 1;
        }
        //! End of period t1, next period is of type acc,dcc, this means motion to vm.
        if(t==d.t1 && (d.p2==smp::period_type::acc || d.p2==smp::period_type::dcc)){
            vi=vm;
            ai=0;
            si=d.s1;
            return 1;
        }
        //! End of period t1, next period is of type none, this means motion to ve.
        if(t==d.t1 && (d.p2==smp::period_type::none )){
            vi=ve;
            ai=0;
            si=d.s1;
            return 1;
        }

        //! Period t2.
        if(t>d.t1 && t<d.t1+d.t2){
            //! Substract period t1 from t.
            t-=d.t1;
            //! If motion t1,t2 is of type acc,dcc or sampled, move from vm to ve.
            if(d.p2==smp::period_type::acc || d.p2==smp::period_type::dcc){
                calculate_acc_dcc_curve_s_v_a_given_time_point(vm,ve,a,t,si,vi,ai);
            }
            //! Could be a steady motion.
            if(d.p2==smp::period_type::steady){
                vi=vm;
                ai=0;
                smp().calculate_steady_displacement(vi,t,si);
            }
            //! Add displacment period t1.
            si+=d.s1;
            return 1;
        }
        //! End of period t2, absolute values.
        if(t==d.t1+d.t2){
            vi=vm;
            ai=0;
            si=d.s1+d.s2;
            return 1;
        }

        //! Check in the mean time if there is a period t3. If there are none, return here.
        if(d.p3==smp::period_type::none){
            vi=vm;
            ai=0;
            si=d.s1+d.s2;
            return 1;
        }

        //! Period t3.
        if(t>d.t1+d.t2){
            //! Limit time to absolute end.
            if(t>d.t1+d.t2+d.t3){
                t=d.t1+d.t2+d.t3;
            }

            //! Substract period t1,t2 from t.
            t-=(d.t1+d.t2);
            //! If motion t1,t2 is of type acc,dcc or sampled, move from vm to ve.
            if(d.p3==smp::period_type::acc || d.p3==smp::period_type::dcc){
                calculate_acc_dcc_curve_s_v_a_given_time_point(vm,ve,a,t,si,vi,ai);
            }
            //! Could be a steady motion.
            if(d.p3==smp::period_type::steady){
                vi=vm;
                ai=0;
                smp().calculate_steady_displacement(vi,t,si);
            }
            //! Add displacment period t1.
            si+=(d.s1+d.s2);
            return 1;
        }

        std::cerr<<"error: eof, function: calculate_scurve_live_motion eof scurve."<<std::endl;
        return 0;
    }

    //! Calculate start-time, end-time for a acceleration curve using concave + convex periods,
    //! the curve uses "acs" acceleration start, "ace" acceleration end values to define the curve
    //! start, end points.
    //! "vo" velocity begin, "ve" velocity end, "a" acceleration.
    //! "ts" time start, "te" time end.
    void example_calculate_curve_times_using_acs_ace_for_acc_curve(){

        double vo=0;
        double ve=10;
        double a=2;
        double acs=2;
        double ace=2;
        double ts=0;
        double te=0;
        double ttot=0, stot=0, th=0;

        double at_time=0, s1=0, s2=0, v1=0, v2=0, a1=0, a2=0, sl=0, sm=0, sr=0;

        std::cout<<"- acc curve -"<<std::endl;

        //! Calculate "ttot" total time, "stot" total displacement.
        calculate_curve_time_displacment(vo,ve,a,stot,ttot);
        std::cout<<"ttot:"<<ttot<<" stot:"<<stot<<std::endl;

        //! Acceleration start input, calculates the time in the concave part of curve.
        calculate_time_point_acs_acc_curve(vo,ve,a,acs,ts);
        std::cout<<"ts:"<<ts<<" acs:"<<acs<<std::endl;
        //! Verify :
        calculate_acc_curve_s_v_a_given_time_point(vo,ve,a,ts,s1,v1,a1);
        std::cout<<"verify at time:"<<at_time<<" a:"<<a1<<" v:"<<v1<<" s:"<<s1<<std::endl;

        //! Acceleration end input, calculates the time in the convex part of curve.
        calculate_time_point_ace_acc_curve(vo,ve,a,ace,te);
        std::cout<<"te:"<<te<<" ace:"<<ace<<std::endl;
        //! Verify :
        calculate_acc_curve_s_v_a_given_time_point(vo,ve,a,te,s2,v2,a2);
        std::cout<<"verify at time:"<<at_time<<" a:"<<a2<<" v:"<<v2<<" s:"<<s2<<std::endl;

        sl=s1;
        sm=s2-s1;
        sr=stot-s2;

        std::cout<<"displacmenent left, sl:"<<sl<<std::endl;
        std::cout<<"displacmenent mid, sm:"<<sm<<std::endl;
        std::cout<<"displacmenent right, sr:"<<sr<<std::endl;

    }
    //! Calculate start-time, end-time for a decelleration curve using concave + convex periods,
    //! the curve uses "acs" acceleration start, "ace" acceleration end values to define the curve
    //! start, end points.
    //! "vo" velocity begin, "ve" velocity end, "a" acceleration.
    //! //! "ts" time start, "te" time end.
    void example_calculate_curve_times_using_acs_ace_for_dcc_curve(){

        double vo=10;
        double ve=0;
        double a=2;
        double acs=2;
        double ace=2;
        double ts=0;
        double te=0;
        double ttot=0, stot=0, th=0;

        double at_time=0, s1=0, s2=0, v1=0, v2=0, a1=0, a2=0, sl=0, sm=0, sr=0;

        std::cout<<"- dcc curve -"<<std::endl;

        //! Calculate "ttot" total time, "stot" total displacement.
        calculate_curve_time_displacment(vo,ve,a,stot,ttot);
        std::cout<<"ttot:"<<ttot<<" stot:"<<stot<<std::endl;

        //! Mention this is the convex type used to calculate the start time.
        calculate_time_point_acs_dcc_curve(vo,ve,a,acs,ts);
        std::cout<<"ts:"<<ts<<" acs:"<<-abs(acs)<<std::endl;
        //! Verify :
        at_time=ts;
        calculate_dcc_curve_s_v_a_given_time_point(vo,ve,a,at_time,s1,v1,a1);
        std::cout<<"verify at time:"<<at_time<<" a:"<<a1<<" v:"<<v1<<" s:"<<s1<<std::endl;

        //! Mention this is the concave type, used to calculate the end time.
        calculate_time_point_ace_dcc_curve(vo,ve,a,ace,te);
        std::cout<<"te:"<<te<<" ace:"<<-abs(ace)<<std::endl;
        //! Verify:
        at_time=te;
        calculate_dcc_curve_s_v_a_given_time_point(vo,ve,a,at_time,s2,v2,a2);
        std::cout<<"verify at time:"<<at_time<<" a:"<<a2<<" v:"<<v2<<" s:"<<s2<<std::endl;

        sl=s1;
        sm=s2-s1;
        sr=stot-s2;

        std::cout<<"displacmenent left, sl:"<<sl<<std::endl;
        std::cout<<"displacmenent mid, sm:"<<sm<<std::endl;
        std::cout<<"displacmenent right, sr:"<<sr<<std::endl;
    }
    //! Calculate motion from zero velocity to cruise velocity.
    void example_motion_from_zero_to_vm(){

        double vo=0.0;
        double vm=10.0;
        double a=2.0;

        double si=0.0, vi=0.0, ai=0.0, t1=0.0, th=0.0;
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

    //! This example is the same as the smp.h planner does the job.
    void example_try_if_curve_fits_displacement(){

        double t1=0;
        double t2=0;
        double t3=0;
        double s1=0;
        double s2=0;
        double s3=0;
        double vo=0;
        double vm=10;
        double ve=0;
        double a=2;
        double stot=100;

        //! Start motion. Acc to vm.
        if(!calculate_curve_time_displacment(vo,vm,a,s1,t1)){
            std::cerr<<"error calculating start motion."<<std::endl;
        }
        //! End motion. Dcc to ve.
        if(!calculate_curve_time_displacment(vm,ve,a,s3,t3)){
            std::cerr<<"error calculating end motion."<<std::endl;
        }
        //! Steady motion. Cruise.
        if(!calculate_steady_time_displacment(vm,stot,s1,s3,s2,t2)){
            std::cerr<<"s2<0"<<std::endl;
        }

        std::cout<<"t1:"<<t1<<" s1:"<<s1<<std::endl;
        std::cout<<"t2:"<<t2<<" s2:"<<s2<<std::endl;
        std::cout<<"t3:"<<t3<<" s3:"<<s3<<std::endl;
    }
};

#endif




















