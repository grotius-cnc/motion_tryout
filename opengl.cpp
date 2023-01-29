#include <math.h>
#include <mainwindow.h>
#include <iostream>
#include <opengl.h>
#include <vector>

double xScale=0;
double yScale=0;

std::vector<double> xVec;
std::vector<double> yVec;

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

void opengl::setVec(std::vector<double> thexVec, std::vector<double> theyVec){
    xVec=thexVec;
    yVec=theyVec;
}

void opengl::setScale(double thexScale, double theyScale){
    xScale=thexScale;
    yScale=theyScale;
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
    for(uint i=0; i<xVec.size(); i++){
        glVertex2d(xVec.at(i)*xScale,this->height()-(yVec.at(i)*yScale));

    }
    glEnd();



}












