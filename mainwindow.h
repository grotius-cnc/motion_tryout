#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opengl.h>
#include <smp.h>
#include <scp.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void thread();

    void on_pushButton_pressed();

    void on_doubleSpinBox_s_valueChanged(double arg1);

    void on_doubleSpinBox_ve_valueChanged(double arg1);

    void on_doubleSpinBox_vo_valueChanged(double arg1);

    void on_doubleSpinBox_a_valueChanged(double arg1);

    void on_doubleSpinBox_vm_valueChanged(double arg1);

    void on_pushButton_start_pressed();

    void on_pushButton_stop_pressed();

    void on_pushButton_scurve_pressed();

    void on_pushButton_calculate_scurve_motion_pressed();

    void on_pushButton_scurve_live_motion_pressed();

    void on_pushButton_interupt_pressed();

    void on_pushButton_acs_ace_pressed();

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    smp *mySmp;
    scp *myScp;
    opengl *myOpenGl;
};
#endif // MAINWINDOW_H
