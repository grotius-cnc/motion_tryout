
# Motion
  
*Controlling motion*

By Skynet Cyberdyne & ChatGdpAI, the year is : 2023.

"The Simple Motion Planner program calculates motion for periods 
t1, t2, and t3 using acceleration, steady, and deceleration periods. 
The program takes input values for initial velocity, final velocity, 
maximum velocity, acceleration, and distance traveled, 
and uses these inputs to calculate the values of t1, t2, and t3.

The program can produce a combination of t1, t2, and t3 periods based 
on the input values. For example, if the distance traveled is 
enough to reach the maximum velocity and decelerate to the final velocity, 
the program will calculate all three periods (t1, t2, and t3) 
and display them in the OpenGL graph.
On the other hand, if the distance traveled is not enough to reach 
the maximum velocity, the program will only produce the periods needed.

The program provides an easy to use and flexible solution for 
motion planning, and allows the user to visualize the motion 
planning based on the inputs given."

This text explains how the program calculates motion 
for periods t1, t2, and t3 and how the program can produce 
a combination of t1, t2, and t3 periods based on the input values. 
It also mentions that the program is easy to use and flexible, 
and allows the user to visualize the motion planning based on the inputs given.

Get started now and see how an object's motion can be easily planned and 
understood using Simple Motion Planner".

To add the simple motion planner to your project, just include the smp.h file.
This smp (simple motion planner) library is header only.

Performance motion planning + interpolation cycle  : 

        ~ 0.006897 milliseconds.

Typical debug information interpolation at time request : 

        v:0.000000 s:100.000000 a:-2.000000 t:15.000000 interpolation ms:0.006962

Acceleration-, Deceleration-, Steady- functions designed and reviewed by Chat GDP AI.

To calculate periods t1,t2,t3, 18 different algo's are used to include all possible scenario's.

Implementation example :

        #include <smp.h>
        
        smp *mySmp=new smp();
     
        //! Set input values.
        mySmp->in.s=100;                                //! s=displacment.
        mySmp->in.a=2;                                  //! a=acceleration.
        mySmp->in.vo=0;                                 //! vo=velocity initial, velocity begin.
        mySmp->in.vm=10;                                //! vm=velocity max, often called cruise speed.
        mySmp->in.ve=0;                                 //! ve=velocity final, velocity end.
        //! Set debug info.
        mySmp->in.debug_functions_and_motion=false;     //! debug.
        mySmp->in.debug_motion_function_time=true;      //! debug.
        
        //! Calculate motion for periods t1,t2,t3
        mySmp->calculate_motion();
        
        //! Sample of calculation motion values :
        double s=mySmp->out.s;                          //! s=displacement.
        double a=mySmp->out.a;                          //! a=acceleration.
        double vo=mySmp->out.vo;                        //! vo=velocity begin.
        double vm=mySmp->out.vm;                        //! vm=velocity max.
        double ve=mySmp->out.ve;                        //! ve=velocity end.
        double t1=mySmp->out.t1;                        //! t1=period t1. Is acceleration or deceleration period.
        double t2=mySmp->out.t2;                        //! t2=period t2. The steady speed period.
        double t3=mySmp->out.t3;                        //! t3=period t3. Is acceleration or deceleration period.
        double s1=mySmp->out.s1;                        //! s1=displacment period t1.
        double s2=mySmp->out.s2;                        //! s2=displacment period t2.
        double s3=mySmp->out.s3;                        //! s3=displacment period t3.
        
        double ttot=mySmp->out.ttot();                  //! ttot=total time, t1+t2+t3.
        double stot=mySmp->out.stot();                  //! stot=total displacement, s1+s2+s3.
        
        //! Set debug info :
        mySmp->out.debug_interpolation_time=true;       //! debug.
        mySmp->out.debug_interpolation_result=true;     //! debug.
        
        //! Perform a interpolation at a certain time stamp to retrieve values.
        std::cout<<"interpolation:"<<std::endl;
        double interval=0.01;                           //! interval time, servo cycle in seconds.
        for(double i=0; i<ttot; i+=interval){
            mySmp->interpolate_motion(i);
        }
        
        //! Check if interpolated end results match the motion calculated inputs.
        std::cout<<"ve:"<<mySmp->out.ve<<" =ve interpolated:"<<mySmp->interpolate_result.v<<std::endl;
        std::cout<<"stot:"<<mySmp->out.s<<" =stot interpolated:"<<mySmp->interpolate_result.s<<std::endl;
        
        //! Overall performance.
        std::cout<<"performance motion planning + interpolation cycle ms:"<<mySmp->motion_calculation_duration_ms()+mySmp->interpolation_calculation_duration_ms()<<std::endl;

Qt User interface :

        ![screen](https://github.com/grotius-cnc/motion_tryout/blob/main/screen.jpg)
        

To make the qt project, in project dir open terminal :
      
        mkdir build
        cd build
        cmake ..
        make










