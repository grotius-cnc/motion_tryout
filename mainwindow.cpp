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
    myOpenGl = new opengl();

    myOpenGl->setTimescale(10);
    myOpenGl->setAccscale(10);
    myOpenGl->setVelocityscale(10);
    myOpenGl->setDistscale(10);

    mySmp=new smp();

    on_pushButton_pressed();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_pressed()
{
    //! Set input values.
    mySmp->in.s=ui->doubleSpinBox_s->value();       //! s=displacment.
    mySmp->in.a=ui->doubleSpinBox_a->value();       //! a=acceleration.
    mySmp->in.vo=ui->doubleSpinBox_vo->value();     //! vo=velocity initial, velocity begin.
    mySmp->in.vm=ui->doubleSpinBox_vm->value();     //! vm=velocity max, often called cruise speed.
    mySmp->in.ve=ui->doubleSpinBox_ve->value();     //! ve=velocity final, velocity end.
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

    //! Draw periods in opengl.
    if(t1==0 && t2==0 && t3==0){
        myOpenGl->setTimevec({0});
        myOpenGl->setVelvec({0});
    }

    if(t1==0 && t2==0 && t3>0){
        myOpenGl->setTimevec({0,t3});
        myOpenGl->setVelvec({vo,ve});
    }

    if(t1>0 && t2>0 && t3==0){
        myOpenGl->setTimevec({0,t1,t1+t2});
        myOpenGl->setVelvec({vo,vm,ve});
    }

    if(t1==0 && t2>0 && t3==0){
        myOpenGl->setTimevec({0,t2});
        myOpenGl->setVelvec({vo,vo});
    }

    if(t1==0 && t2>0 && t3>0){
        myOpenGl->setTimevec({0,t2,t2+t3});
        myOpenGl->setVelvec({vo,vm,ve});
    }

    if(t1>0 && t2>0 && t3>0){
        myOpenGl->setTimevec({0,t1,t1+t2,t1+t2+t3});
        myOpenGl->setVelvec({vo,vm,vm,ve});
    }
}

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
