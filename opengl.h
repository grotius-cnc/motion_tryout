#ifndef OPENGL_H
#define OPENGL_H

#include <QMainWindow>
#include <mainwindow.h>
#include <QWidget>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QToolTip>
#include <GL/gl.h>
#include <GL/glu.h>
#include <QtWidgets>

class opengl : public QOpenGLWidget{

public:
    opengl(QWidget *parent = nullptr);
    ~opengl();

    void reset();

    void setTimescale(double theScale);
    void setAccscale(double theScale);
    void setVelocityscale(double theScale);
    void setDistscale(double theScale);

    void setVelvec(std::vector<double> theVec);
    void setAccvec(std::vector<double> theVec);
    void setDistvec(std::vector<double> theVec);
    void setTimevec(std::vector<double> theVec);

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

private:
    opengl *myOpengl;
};

#endif
