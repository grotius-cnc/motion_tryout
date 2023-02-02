#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //! OpenGl output to verify.
    myOpenGl = new opengl();
    //! Graph scale.
    myOpenGl->setScale(20,20);

    //! Simple motion planner class.
    mySmp=new smp();

    //! Simple curve planner class.
    myScp=new scp();

    //! Startup calculation.
    //! on_pushButton_pressed();

    //! Timer to simulate servo cycle.
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(thread()));
    timer->start(1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

//! Parameters used by the live motion planner.
smp::smp_data datas;
double vo=0;                //! Velocity begin.
double vm=0;                //! Velocity max, cruise speed.
double ve=0;                //! Velocity end.
double a=0;                 //! Acceleration.
double dtg=0;               //! Distance to go.
double motion_timer=0;      //! Current time of motion.
double position=0;          //! Overall position.
bool go=0;
bool start=0, stop=0, stop_init=0;
bool reverse=0;

//! Parameters used by the scurve motion planner.
bool sc_go=0;
double sc_vo=10;             //! Velocity begin.
double sc_ve=0;            //! Velocity end.
double sc_a=2;              //! Acceleration.
double sc_ai=0;             //! Acceleration interpolated, current.
double sc_vi=0;             //! Velocity interpolated.
double sc_ti=0;             //! Time interpolated.
double sc_si=0;             //! Displacment interpolated.
double sc_init=0;           //! Init motion block.
double sc_ttot=0;           //! Time of motion block.
double sc_stot=0;           //! Total displacment of motion block.
double sc_th=0;             //! Ttot * 0.5
double sc_jm=0;             //! Jm of current motion block.

//! Scurve live parameters.
bool sc_live_go=0;
bool sc_live_init=0;
smp::smp_data ld;
double s=0;
bool done=0;
double t=0, si=0, vi=0,ai=0;
bool interupt=0;
std::vector<double> xpos,ypos;

//! Live motion planner start button.
void MainWindow::on_pushButton_start_pressed()
{
    vo=ui->doubleSpinBox_vo->value();
    vm=ui->doubleSpinBox_vm->value();
    ve=ui->doubleSpinBox_ve->value();
    a=ui->doubleSpinBox_a->value();
    dtg=ui->doubleSpinBox_s->value();

    dtg-=position;
    if(dtg<0){ //! Keep positive.
        dtg=abs(dtg);
        reverse=true;
    } else {
        reverse=false;
    }

    start=true;
    timer=0;
}

//! Live motion planner stop or pause button.
void MainWindow::on_pushButton_stop_pressed()
{
    stop=true;
}

//! This function simulates the servo cycle. And is called every 1 millisecond.
void MainWindow::thread(){

    //! Scurve live motion test.
    if(sc_live_go){
        //! Init the motion.
        if(!sc_live_init){
            xpos.clear();
            ypos.clear();
            xpos.push_back(0);
            ypos.push_back(0);
            vo=ui->doubleSpinBox_vo->value();
            vm=ui->doubleSpinBox_vm->value();
            ve=ui->doubleSpinBox_ve->value();
            a=ui->doubleSpinBox_a->value();
            dtg=ui->doubleSpinBox_s->value();
            done=false;

            smp().calculate_offline_motion(vo,vm,ve,a,dtg,ld,true);
            t=0;
            sc_live_init=true;
        }

        //! Check for interupts, stop for now.
        if(interupt){

            std::cout<<"incoming interupt from vm, current scurve values:"<<std::endl;
            scp().print(si,vi,ai,t);
            dtg-=si;
            std::cout<<"new dtg:"<<dtg<<" new vm:"<<vm<<std::endl;

            //! Calculate new periods, print them, then compare a new scurve period t1 with it.
            std::cout<<"new smp motion block:"<<std::endl;
            smp().calculate_offline_motion(vi,vm,ve,a,dtg,ld,true);

            std::cout<<"t1 to investegate for fit scurve:"<<ld.t1<<std::endl;
            std::cout<<"s1 to investegate for fit scurve:"<<ld.s1<<std::endl;
            std::cout<<"scurve starts with a:"<<ai<<std::endl;

            double ttot=0;
            scp().calculate_curve_time_given_acs_ace_for_dcc_curve(vi,ve,a,ai,0,ttot,false);

            std::cout<<"scurve needs ttot:"<<ttot<<std::endl;

            timer->stop();
        }

        if(!interupt){
            //! Perform the motion.
            if(scp().calculate_scurve_live_motion(ld,t,si,vi,ai)){

                if(!done){
                    scp().print(si,vi,ai,t);
                    t+=0.001;

                    //! Draw trapezium curve opengl.
                    if(ld.t1>0 && ld.t2==0 && ld.t3==0){
                        myOpenGl->set1Vec({0,ld.t1},{ld.vo,ld.ve});
                    }
                    if(ld.t1>0 && ld.t2>0 && ld.t3==0){
                        myOpenGl->set1Vec({0,ld.t1,ld.t1+ld.t2},{ld.vo,ld.vm,ld.ve});
                    }

                    if(ld.t1>=0 && ld.t2>0 && ld.t3>0){
                        myOpenGl->set1Vec({0,ld.t1,ld.t1+ld.t2,ld.t1+ld.t2+ld.t3},{ld.vo,ld.vm,ld.vm,ld.ve});
                    }

                    //! Draw scurve opengl.
                    xpos.push_back(t);
                    ypos.push_back(vi); //! Add 1 to avoid double lines by opengl.
                    myOpenGl->set2Vec(xpos,ypos);
                }

                if(si>=dtg){
                    done=true;
                    //! Reset flag.
                    sc_live_init=false;
                    sc_live_go=false;
                }
            } else {
                //! Stop when error.
                sc_live_go=false;
            }
        }

    }

    //! Test a scurve of type deceleration.
    if(sc_go){
        if(!sc_init){
            scp().calculate_scurve_total_s_t_jm(sc_vo,sc_ve,sc_a,sc_stot,sc_ttot,sc_jm);
            std::cout<<"ttot"<<sc_ttot<<std::endl;
            sc_init=1;
        }

        if(!scp().calculate_dcc_curve_s_v_a_given_time_point(sc_vo,sc_ve,sc_a,sc_ti,sc_si,sc_vi,sc_ai)){
            std::cerr<<"error."<<std::endl;
        }
        scp().print(sc_si,sc_vi,sc_ai,sc_ti);

        if(sc_ti<=sc_ttot){
            sc_ti+=0.001;
        }
    }

    //! Perform a stop sequence.
    if(stop){
        //! Function to calculate distance needed to stop motion.
        dtg=smp().dtg_to_stop(vo,a);
        //! Set velocity end to zero when request a motion stop.
        ve=0;
        std::cerr<<"dtg to stop:"<<dtg<<std::endl;
        //! Reset flag.
        stop=0;
    }
    //! Execute motion.
    if(start){
        //! Get the current "vm" velocity max value.
        vm=ui->doubleSpinBox_vm->value();
        //! Calculate motion, get results for the 0.001s interval.
        if(smp().calculate_live_motion(vo,vm,ve,a,dtg,datas,0.001,false)){
            //! Update values for next servo cycle.
            vo=datas.vi;
            //! Update distance to go, dtg.
            dtg-=datas.si;
            //! If user requests a negative displacment "s".
            if(!reverse){
                position+=datas.si;
            } else {
                position-=datas.si;
            }

            //! Terminal output.
            std::cout<<std::fixed<<"time:"<<motion_timer<<" v:"<<datas.vi<<" dtg:"<<dtg<<" position:"<<position<<" performance ms:"<<datas.performance_ms<<std::endl;
            motion_timer+=0.001;

            if(dtg==0){
                //! Reset flag.
                start=0;
                std::cerr<<"motion complete."<<std::endl;
            }
        } else {
            //! Reset flag.
            start=0;
            std::cerr<<"error from function calculate live motion."<<std::endl;
        }
    }
}

//! Standard motion calculation example.
void MainWindow::on_pushButton_pressed()
{
    smp::smp_data d;
    //! calculate_motion(double vo, double vm, double ve, double a, double s, smp_data &d, double at_time, bool debug_standard){
    if(!smp().calculate_offline_motion(ui->doubleSpinBox_vo->value(),
                                       ui->doubleSpinBox_vm->value(),
                                       ui->doubleSpinBox_ve->value(),
                                       ui->doubleSpinBox_a->value(),
                                       abs(ui->doubleSpinBox_s->value()),d,true)){
        std::cerr<<"error from function calculate motion."<<std::endl;
    }

    //! By reference &d outputs.
    if(d.t1>0 && d.t2==0 && d.t3==0){
        myOpenGl->set1Vec({0,d.t1},{d.vo,d.ve});
    }
    if(d.t1>0 && d.t2>0 && d.t3==0){
        myOpenGl->set1Vec({0,d.t1,d.t1+d.t2},{d.vo,d.vm,d.ve});
    }

    if(d.t1>=0 && d.t2>0 && d.t3>0){
        myOpenGl->set1Vec({0,d.t1,d.t1+d.t2,d.t1+d.t2+d.t3},{d.vo,d.vm,d.vm,d.ve});
    }
}

//! Update values imidiate.
void MainWindow::on_doubleSpinBox_s_valueChanged(double arg1)
{
    //on_pushButton_pressed();
}

void MainWindow::on_doubleSpinBox_ve_valueChanged(double arg1)
{
    //on_pushButton_pressed();
}

void MainWindow::on_doubleSpinBox_vo_valueChanged(double arg1)
{
    //on_pushButton_pressed();
}

void MainWindow::on_doubleSpinBox_a_valueChanged(double arg1)
{
    // on_pushButton_pressed();
}

void MainWindow::on_doubleSpinBox_vm_valueChanged(double arg1)
{
    //on_pushButton_pressed();
}


void MainWindow::on_pushButton_scurve_pressed()
{
    sc_go=true;
    //! myScp->example_try_if_curve_fits_displacement();
}

void MainWindow::on_pushButton_calculate_scurve_motion_pressed()
{
    smp::smp_data d;
    smp().calculate_offline_motion(ui->doubleSpinBox_vo->value(),
                                   ui->doubleSpinBox_vm->value(),
                                   ui->doubleSpinBox_ve->value(),
                                   ui->doubleSpinBox_a->value(),
                                   ui->doubleSpinBox_s->value(),d, true);

    //! By reference &d outputs.
    if(d.t1>0 && d.t2==0 && d.t3==0){
        myOpenGl->set1Vec({0,d.t1},{d.vo,d.ve});
    }
    if(d.t1>0 && d.t2>0 && d.t3==0){
        myOpenGl->set1Vec({0,d.t1,d.t1+d.t2},{d.vo,d.vm,d.ve});
    }
    if(d.t1>=0 && d.t2>0 && d.t3>0){
        myOpenGl->set1Vec({0,d.t1,d.t1+d.t2,d.t1+d.t2+d.t3},{d.vo,d.vm,d.vm,d.ve});
    }
}

void MainWindow::on_pushButton_scurve_live_motion_pressed()
{
    sc_live_go=true;
}

void MainWindow::on_pushButton_interupt_pressed()
{
    interupt=1;
}

void MainWindow::on_pushButton_acs_ace_pressed()
{
    //! scp().example_calculate_curve_times_using_acs_ace_for_acc_curve();
    //! scp().example_calculate_curve_times_using_acs_ace_for_dcc_curve();

    double vo=0;
    double ve=10;
    double a=2;
    double acs=1;
    double ace=1;
    double debug=true;
    double sl=0, sm=0, sr=0;
    double v1=0, v2=0, t1=0, t2=0, a1=0, a2=0, stot=0, ttot=0;

    //! Acc request.
    scp().calculate_curve_time_given_acs_ace_for_acc_dcc_curve(vo,ve,a,acs,ace,debug,sl,sm,sr,v1,v2,t1,t2,a1,a2,stot,ttot);

    //! Dcc request.
    vo=10;
    ve=0;
    scp().calculate_curve_time_given_acs_ace_for_acc_dcc_curve(vo,ve,a,acs,ace,debug,sl,sm,sr,v1,v2,t1,t2,a1,a2,stot,ttot);
}





























