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

    myOpenGl->setScale(10,10);

    mySmp=new smp();

    on_pushButton_pressed();
    //!
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(thread()));
    timer->start(1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

smp::smp_data datas;
double vo=0,vm=0,ve=0,a=0, dtg=0, motion_timer=0, position=0;
bool go=0;
bool start=0, stop=0, stop_init=0;
bool reverse=0;

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


void MainWindow::on_pushButton_stop_pressed()
{
    stop=true;
}


void MainWindow::thread(){

    if(stop){
        dtg=smp().dtg_to_stop(vo,a);
        ve=0;
        std::cerr<<"dtg to stop:"<<dtg<<std::endl;
        stop=0;
    }

    if(start){
        vm=ui->doubleSpinBox_vm->value();
        if(mySmp->calculate_live_motion(vo,vm,ve,a,dtg,datas,true,0.001,true,false)){
            vo=datas.vi;
            dtg-=datas.si;
            if(!reverse){
                position+=datas.si;
            } else {
                position-=datas.si;
            }

            std::cout<<"time:"<<motion_timer<<" v:"<<datas.vi<<" dtg:"<<dtg<<" position:"<<position<<std::endl;
            motion_timer+=0.001;

            if(dtg==0){
                start=0;
                std::cerr<<"motion complete."<<std::endl;
            }
        } else {
            std::cerr<<"error from function calculate live motion."<<std::endl;
            start=0;
        }
    }
}

void MainWindow::on_pushButton_pressed()
{
    smp::smp_data d;
    if(!mySmp->calculate_motion(ui->doubleSpinBox_vo->value(),
                                ui->doubleSpinBox_vm->value(),
                                ui->doubleSpinBox_ve->value(),
                                ui->doubleSpinBox_a->value(),
                                ui->doubleSpinBox_s->value(),d,false,0.0,false,true)){
        std::cerr<<"error from function calculate motion."<<std::endl;
    }

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








