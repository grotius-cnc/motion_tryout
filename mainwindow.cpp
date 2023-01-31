#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <iostream>
#include <stdio.h>
#include <math.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //! OpenGl output to verify.
    myOpenGl = new opengl();
    //! Graph scale.
    myOpenGl->setScale(10,10);

    //! Simple motion planner class.
    mySmp=new smp();

    //! Simple curve planner class.
    myScp=new scp();

    //! Startup calculation.
    on_pushButton_pressed();

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

//! Parameters used by the scurve live motion planner.
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

    if(sc_go){
        if(!sc_init){
            myScp->calculate_scurve_total_s_t_jm(sc_vo,sc_ve,sc_a,sc_stot,sc_ttot,sc_jm);
            std::cout<<"ttot"<<sc_ttot<<std::endl;
            sc_init=1;
        }

        if(!myScp->calculate_dcc_curve_s_v_a_given_time_point(sc_vo,sc_ve,sc_a,sc_ti,sc_si,sc_vi,sc_ai)){
            std::cerr<<"error."<<std::endl;
        }
        myScp->print(sc_si,sc_vi,sc_ai,sc_ti);

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
        if(mySmp->calculate_live_motion(vo,vm,ve,a,dtg,datas,0.001,false)){
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
    if(!mySmp->calculate_motion(ui->doubleSpinBox_vo->value(),
                                ui->doubleSpinBox_vm->value(),
                                ui->doubleSpinBox_ve->value(),
                                ui->doubleSpinBox_a->value(),
                                abs(ui->doubleSpinBox_s->value()),d,0.0,true)){
        std::cerr<<"error from function calculate motion."<<std::endl;
    }

    //! By reference &d outputs.
    if(d.t1>0 && d.t2==0 && d.t3==0){
        myOpenGl->setVec({0,d.t1},{d.vo,d.ve});
    }
    if(d.t1>0 && d.t2>0 && d.t3==0){
        myOpenGl->setVec({0,d.t1,d.t1+d.t2},{d.vo,d.vm,d.ve});
    }

    if(d.t1>=0 && d.t2>0 && d.t3>0){
        myOpenGl->setVec({0,d.t1,d.t1+d.t2,d.t1+d.t2+d.t3},{d.vo,d.vm,d.vm,d.ve});
    }
}

//! Update values imidiate.
void MainWindow::on_doubleSpinBox_s_valueChanged(double arg1)
{
    on_pushButton_pressed();
}

void MainWindow::on_doubleSpinBox_ve_valueChanged(double arg1)
{
    on_pushButton_pressed();
}

void MainWindow::on_doubleSpinBox_vo_valueChanged(double arg1)
{
    on_pushButton_pressed();
}

void MainWindow::on_doubleSpinBox_a_valueChanged(double arg1)
{
    on_pushButton_pressed();
}

void MainWindow::on_doubleSpinBox_vm_valueChanged(double arg1)
{
    on_pushButton_pressed();
}


void MainWindow::on_pushButton_scurve_pressed()
{
    sc_go=true;
}




















