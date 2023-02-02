#include <math.h>
#include <mainwindow.h>
#include <iostream>
#include <opengl.h>
#include <vector>

double xScale=0;
double yScale=0;

std::vector<double> x1Vec, y1Vec, x2Vec, y2Vec;

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

void opengl::set1Vec(std::vector<double> thexVec, std::vector<double> theyVec){
    x1Vec=thexVec;
    y1Vec=theyVec;
}

void opengl::set2Vec(std::vector<double> thexVec, std::vector<double> theyVec){
    x2Vec=thexVec;
    y2Vec=theyVec;
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

    glColor4d(5,5,5,50);
    glBegin(GL_LINE_STRIP);
    glLineWidth(1);
    for(uint i=0; i<x1Vec.size(); i++){
        glVertex2d(x1Vec.at(i)*xScale,this->height()-(y1Vec.at(i)*yScale)-50);

    }
    glEnd();

    glColor4d(255,0,0,255);
    glBegin(GL_LINE_STRIP);
    for(uint i=0; i<x2Vec.size(); i++){
        glVertex2d(x2Vec.at(i)*xScale,this->height()-(y2Vec.at(i)*yScale)); // add 5 to avoid double lines.

    }
    glEnd();



}












