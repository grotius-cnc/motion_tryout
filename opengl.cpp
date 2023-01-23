#include <math.h>
#include <mainwindow.h>
#include <iostream>
#include <opengl.h>
#include <vector>

double timescale=0;
double velocityscale=0;
double accscale=0;
double distscale=0;
std::vector<double> velvec;
std::vector<double> accvec;
std::vector<double> distvec;
std::vector<double> timevec;

opengl::opengl(QWidget *parent)
    : QOpenGLWidget(parent)
{
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(50);
}

opengl::~opengl()
{
    //! destructor
}

void opengl::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_STIPPLE);
    setMouseTracking(true);
}

void opengl::resizeGL(int w, int h)
{
    // std::cout<<"resize"<<std::endl;
    glViewport(0, 0, w, h);
}

void opengl::reset(){

}

void opengl::setVelvec(std::vector<double> theVec){
    velvec=theVec;
}

void opengl::setAccvec(std::vector<double> theVec){
    accvec=theVec;
}

void opengl::setDistvec(std::vector<double> theVec){
    distvec=theVec;
}

void opengl::setTimevec(std::vector<double> theVec){
    timevec=theVec;
}

void opengl::setTimescale(double theScale){
    timescale=theScale;
}

void opengl::setAccscale(double theScale){
    accscale=theScale;
}

void opengl::setVelocityscale(double theScale){
    velocityscale=theScale;
}

void opengl::setDistscale(double theScale){
    distscale=theScale;
}

void opengl::paintGL()
{
    // std::cout<<"updating"<<std::endl;

    glViewport(0, 0, this->width(), this->height());
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, this->width(), this->height(), 0, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();


    glColor4d(255,255,0,255);
    glBegin(GL_LINE_STRIP);
    for(uint i=0; i<timevec.size(); i++){
        glVertex2d(timevec.at(i)*timescale,this->height()-(velvec.at(i)*velocityscale));

    }
    glEnd();



}












