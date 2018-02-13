#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimer>
#include <qwidget.h>
#include <QSettings>
#include <myhelper.h>
#include <models.hpp>
#include <thread>
#include <mutex>
#include <frames.hpp>
#include <frames_io.hpp>
#include <rotational_interpolation_sa.hpp>

//#include <qt_windows.h>

namespace Ui {
class MainWindow;
}



class KDLfun:public QObject
{
    Q_OBJECT
public:

    KDLfun()
    {
        snake = SnakeRobot();
    }
    bool IfInit = false;
    void GetForwardKDL(JntArray init);
    void Getinkdl();
    void CharToAngle(const char* serial);
    void IntToAngle(const int * serial);//go to int first
    void JntToInt(char * Sendbuf);
    Frame Now_Pos;
    Frame Goal_Pos;
    JntArray Jnt_Now;
    JntArray Jnt_Goal;
    bool IfCaul = true;
    bool IfClick =false;
    bool firstFlag =true;
    bool IfPortOpen = false;
    QSerialPort* MySerialPort;
    QByteArray SendBuf;
    void SendMassage();
    char GraspFlag = 0;
    char Grab1 = 0XFF;
    char Grab2 = 0XFF;
    double Roll;
    int SendBufShow[6];
    double Pitch;
    double Yaw;
    bool First = true;
    bool JntGFlag = true;
    bool NextMoveFlag =false;
    unsigned char MoveAction = 0;

signals:
    void SendSignal();


public:
    Chain snake;
};



class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = 0);
    void SetPort();
    ~MainWindow();
    void ComSend();
    char* readbuf;
    KDLfun Snake;
    void NewThread(KDLfun* Snake);
    void ThreadSend(KDLfun* Snake);
    std::mutex Mutex;
    std::mutex WholeMutex;



private slots:
    void on_Connect_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void ReadMyCom();//read data

    void on_Pause_clicked();

    void on_Forward_clicked();

    void on_Leftward_clicked();

    void on_Rightward_clicked();

    void on_Backward_clicked();

    void on_Upward_clicked();

    void on_Downward_clicked();

    void SendMess();

    void on_claw1_clicked();

    void on_claw2_clicked();

    void on_pushButton_3_clicked();

    void on_xup_clicked();

    void on_xdown_clicked();

    void on_yup_clicked();

    void on_ydown_clicked();

    void on_zup_clicked();

    void on_zdown_clicked();

    void on_Joint1Add_clicked();

    void on_Joint2Add_clicked();

    void on_Joint3Add_clicked();

    void on_Joint4Add_clicked();

    void on_Joint5Add_clicked();

    void on_Joint6Add_clicked();

    void on_Joint1D_clicked();

    void on_Joint2D_clicked();

    void on_Joint3D_clicked();

    void on_Joint4D_clicked();

    void on_Joint5D_clicked();

    void on_Joint6D_clicked();

    void on_NextMove_clicked();

private:
    Ui::MainWindow *ui;
    QTimer* MyTimer;


    //QApplication TestCon;


};

#endif // MAINWINDOW_H
