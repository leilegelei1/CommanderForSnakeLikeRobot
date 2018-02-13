#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QApplication>
#include <myhelper.h>
#include <thread>
#include <kdl.h>
using namespace std;
#define SetSpeed 5
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //how can we do this


    ui->setupUi(this);

    Eigen::VectorXd Buf(6);
    Snake.Jnt_Now.data = Buf;
    this->Snake.Jnt_Goal.data = Buf;

    ui->pushButton->setShortcut(Qt::Key_M);
    ui->Joint1Add->setShortcut(Qt::Key_Q);
    ui->Joint1D->setShortcut(Qt::Key_A);
    ui->Joint2Add->setShortcut(Qt::Key_W);
    ui->Joint2D->setShortcut(Qt::Key_S);
    ui->Joint3Add->setShortcut(Qt::Key_E);
    ui->Joint3D->setShortcut(Qt::Key_D);
    ui->Joint4Add->setShortcut(Qt::Key_R);
    ui->Joint4D->setShortcut(Qt::Key_F);
    ui->Joint5Add->setShortcut(Qt::Key_T);
    ui->Joint5D->setShortcut(Qt::Key_G);
    ui->Joint6Add->setShortcut(Qt::Key_Y);
    ui->Joint6D->setShortcut(Qt::Key_H);

    Snake.MySerialPort = new QSerialPort();
    SetPort();
    connect(Snake.MySerialPort,SIGNAL(readyRead()),this,SLOT(ReadMyCom()));
    connect((QObject*)(&Snake),SIGNAL(SendSignal()),this,SLOT(SendMess()));
}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::NewThread(KDLfun* Snake){
    while(1)
    {
        if(Snake->IfCaul){
            if(Snake->IfInit){
                if(Mutex.try_lock()){
                    Snake->GetForwardKDL(Snake->Jnt_Now);
                    if(WholeMutex.try_lock())
                    {
                        //cout<<"I am here"<<endl;
                        if(Snake->IfClick){
                            cout<<BOLDGREEN<<"Go to ik caluacation"<<endl;
                            Snake->Getinkdl();
                        }
                        WholeMutex.unlock();
                    }
                    Mutex.unlock();
                }
            }
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(50));//必须延时，不然程序跑死了
    }
}

void MainWindow::SendMess(){
    //char tx_b[1]  = {0xff};
    // QByteArray tx_buf(tx_b);
    //Snake.MySerialPort->write(tx_buf);
    int Test = Snake.SendBuf[14] & 0x000000ff;
    //   cout<<hex<<Test<<endl;
    cout<<RED;
    if(!Snake.firstFlag)
    {
        for(int i =0;i<17;i++)
        {
            if(i>1&&i<14)
            {
                int AS = (int)(Snake.SendBuf[i]&0x000000ff)+(int)(Snake.SendBuf[i+1]&0x000000ff)*256;
                cout<<dec<<AS<<" ";
                i++;
                continue;
            }
            cout<<hex<<(int)(Snake.SendBuf[i]&0x000000ff)<<" ";//<<Snake.SendBuf[15]&0x000000ff<<endl;
        }
        cout<<endl;
        if(Mutex.try_lock()){
            int TT[12];
            for(int i=0;i<12;i++)
            {
                TT[i]=(int)(Snake.SendBuf[i+2]&0x000000ff);//<<" ";
            }
            for(int i=0;i<6;i++)
            {
                int j = i*2;
                Snake.SendBufShow[i] = TT[j] + TT[j+1]*256;
            }


            Snake.MySerialPort->write(Snake.SendBuf);
            Mutex.unlock();
        }
    }
}


void MainWindow::ThreadSend(KDLfun* Snake){
    while(true){
        if(Snake->IfPortOpen){
            //Snake->SendMassage();
            if(Snake->IfInit){
                if(Mutex.try_lock()){
                    char tx_b[17];
                    Snake->JntToInt(tx_b);
                    QByteArray tx_buf(tx_b);
                    Snake->SendBuf = tx_buf;
                    //Snake->MySerialPort->write(tx_buf);
                    // if(Snake->IfClick){
                    Snake->SendMassage();
                    //  }
                    //if(Snake->NextMoveFlag ==true)
                    //  Snake->IfInit = false;
                    Mutex.unlock();
                }
                //cout<<BOLDMAGENTA<<"I am sending massage"<<endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));//必须延时，不然程序跑死了
        }
    }

}

void byteToint(QByteArray bytes,int * test){
    for(int i =0;i<bytes.length();i++)
        test[i] = bytes[i]& 0x000000FF;
}

void MainWindow::on_Connect_clicked()
{
    static bool temp = 0;
    //std::thread ReadCom(&MainWindow::ThreadRead,this);
    if(!temp)
    {

        temp = 1;
        //How to set
        Snake.MySerialPort->setPortName(ui->Port->currentText());
        Snake.MySerialPort->open(QIODevice::ReadWrite);
        Snake.MySerialPort->setBaudRate(ui->Baudrate->currentText().toInt());
        Snake.MySerialPort->setDataBits(QSerialPort::Data8);
        Snake.MySerialPort->setParity(QSerialPort::NoParity);
        Snake.MySerialPort->setStopBits(QSerialPort::OneStop);
        Snake.MySerialPort->setFlowControl(QSerialPort::NoFlowControl);

        if(Snake.MySerialPort->isOpen()){
            ui->Connect->setText("Disconnect");
            std::thread KDLThread(&MainWindow::NewThread,this,&Snake);
            KDLThread.detach();
            Snake.IfPortOpen = true;
            std::thread SendThread(&MainWindow::ThreadSend,this,
                                   &Snake);
            SendThread.detach();
        }
        else
        {

            cout << "can't open the device"<<endl;
        }
        // ReadCom.join();
    }
    else
    {
        Snake.IfPortOpen =false;
        ui->Connect->setText("Connect");
        temp = 0;
        Snake.MySerialPort->close();
        //  ReadCom.detach();
    }
}

void MainWindow::on_pushButton_clicked()
{

    cout <<GREEN<<endl;

    JntArray Test1;
    Test1.data.resize(6);
    double AB[6] = {10,10,10,10,10,10};
    //    //double AB[6] = {0,0,0,0,0,0};
    //    //double AB[6] = {M_PI_2,0.0,-M_PI_2,M_PI_2,0.0,-M_PI_2};
    for(int i =0;i<6;i++)
        Test1.data[i] = AB[i]/180.0*M_PI;
    //      ChainFkSolverPos_recursive fwdkin(Snake.snake);
    //      Frame pos_goal;
    //    fwdkin.JntToCart(Test1,pos_goal);
    //  Snake.GetForwardKDL(Test1);

    Snake.GetForwardKDL(Snake.Jnt_Now);
    cout<<"pos_now " << Snake.Now_Pos << endl;
    cout << "pos_goal " << Snake.Goal_Pos << endl;
    //cout << "pos_goal "<<endl << pos_goal << endl;
    cout<<"q_now"<<endl;
    for(int i =0;i<6;i++)
        cout<<Snake.Jnt_Now.data[i]/M_PI*180<<" ";
    cout<<endl;
    //QString tempDataHex=myHelper::ByteArrayToHexStr(Snake.SendBuf);
    cout<<"RPY_now"<<endl;
    cout<<Snake.Roll/M_PI*180<<" ";
    cout<<Snake.Pitch/M_PI*180<<" ";
    cout<<Snake.Yaw/M_PI*180<<" ";
    cout<<endl;
    cout<<"SendAngel"<<endl;
    for(int i =0;i<6;i++)
    {
        // cout<<Snake.SendBufShow[i]<<" ";
        if(i!=1&&i!=4)//Here
            cout<<Snake.SendBufShow[i]/4096.0*360.0- 180 <<" ";
        else
            cout<<  Snake.SendBufShow[i]/1024.0*300.0 - 150<<" ";
    }
    cout<<endl;
    int Test = Snake.SendBuf[14]&0x000000ff;
    //cout<<YELLOW<<hex<<Test;


    //cout << "reached_pos " << solver.T_base_head << endl;
    //return a.exec();
}

void MainWindow::SetPort()
{
    QSerialPortInfo info;
    Snake.MySerialPort->setPort(info);
    if(Snake.MySerialPort->open(QIODevice::ReadWrite))
    {
        ui->Port->addItem(info.portName());
        Snake.MySerialPort->close();
    }
}

void MainWindow::ReadMyCom(){
    //这个判断尤为重要,否则的话直接延时再接收数据,空闲时和会出现高内存占用
    if (Snake.MySerialPort->bytesAvailable()<=0){return ;}
    //myHelper::Sleep(100);//延时100毫秒保证接收到的是一条完整的数据,而不是脱节的
    QByteArray buffer= Snake.MySerialPort->readAll();

    Snake.GraspFlag &= 0b00000011;//set the next move flag

    if(ui->ShowInHex->isChecked()){
        QString tempDataHex=myHelper::ByteArrayToHexStr(buffer);
        if(tempDataHex.length()<6)
            ui->TextShower->append(QString("接收:%1 时间:%2")
                                   .arg(tempDataHex)
                                   .arg(QTime::currentTime().toString("HH:mm:ss")));
        //this->readbuf = buffer.data();
        if(Mutex.try_lock()){
            int IntBuf[20];
            byteToint(buffer,IntBuf);
            //this->Snake.CharToAngle(this->readbuf);
            this->Snake.IntToAngle(IntBuf);
            // if(Snake.NextMoveFlag)
            //      Snake.Jnt_Goal.data = Snake.Jnt_Now.data;
            Mutex.unlock();
        }
    }
    else
    {
        QString tempDataString;
        tempDataString.prepend(buffer);
        tempDataString = tempDataString.left(tempDataString.length()-2)+" ";

        ui->TextShower->append(QString("接收:%1 时间:%2")
                               .arg(tempDataString)
                               .arg(QTime::currentTime().toString("HH:mm:ss")));

    }
}


void MainWindow::on_pushButton_2_clicked()
{
    SetPort();
}

void MainWindow::on_Pause_clicked()
{
    this->Snake.IfCaul = !this->Snake.IfCaul;
}

void MainWindow::on_Forward_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        Snake.Goal_Pos = Snake.Now_Pos;
        Snake.Goal_Pos.p.data[0] = Snake.Now_Pos.p.data[0]+0.04;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}

void MainWindow::on_Backward_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        Snake.Goal_Pos = Snake.Now_Pos;
        cout << BOLDMAGENTA <<"Now_Pos"<<endl;
        cout<<Snake.Goal_Pos.p<<endl;
        Snake.Goal_Pos.p.data[0] = Snake.Now_Pos.p.data[0]-0.04;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}


void MainWindow::on_Leftward_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        Snake.Goal_Pos = Snake.Now_Pos;
        Snake.Goal_Pos.p.data[1] = Snake.Now_Pos.p.data[1]+0.02;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}

void MainWindow::on_Rightward_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        Snake.Goal_Pos = Snake.Now_Pos;
        Snake.Goal_Pos.p.data[1] = Snake.Now_Pos.p.data[1]-0.02;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}

void MainWindow::on_Upward_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        Snake.Goal_Pos = Snake.Now_Pos;
        Snake.Goal_Pos.p.data[2] = Snake.Now_Pos.p.data[2]+0.02;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}

void MainWindow::on_Downward_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        Snake.Goal_Pos = Snake.Now_Pos;
        Snake.Goal_Pos.p.data[2] = Snake.Now_Pos.p.data[2]-0.02;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}



void MainWindow::on_claw1_clicked()
{
    if(ui->claw1->text()=="Grab"){
        ui->claw1->setText("Loosen");
        Snake.GraspFlag |= 0b00000001;
        Snake.Grab2 = 0xfe;
    }
    else
    {
        ui->claw1->setText("Grab");
        Snake.GraspFlag &= 0b00001110;
        Snake.Grab2 = 0xff;
    }
    Snake.NextMoveFlag = false;
}

void MainWindow::on_claw2_clicked()
{
    if(ui->claw2->text()=="Grab"){
        ui->claw2->setText("Loosen");
        Snake.GraspFlag |= 0b00000010;
        Snake.Grab1 = 0xfe;
    }
    else
    {
        ui->claw2 ->setText("Grab");
        Snake.GraspFlag &= 0b00001101;
        Snake.Grab1 = 0xff;
    }
    Snake.NextMoveFlag = false;
}

void MainWindow::on_pushButton_3_clicked()
{
    Snake.GraspFlag |= 0b00000100;
    //go to next turn
}

void MainWindow::on_xup_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        cout<<RED<<Snake.Roll<<"1111 ";
        Frame AS = Snake.Now_Pos;
        Snake.Now_Pos.M.GetRPY(Snake.Roll,Snake.Pitch,Snake.Yaw);
        cout<<RED<<Snake.Roll<<"222 "<<endl;
        Vector Pos = Snake.Now_Pos.p;
        Snake.Roll += M_PI/18;
        Frame AB(Rotation::RPY(Snake.Roll,Snake.Pitch,Snake.Yaw),Pos);
        Snake.Goal_Pos = AB;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}

void MainWindow::on_xdown_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){

        Frame AS = Snake.Now_Pos;
        Snake.Now_Pos.M.GetRPY(Snake.Roll,Snake.Pitch,Snake.Yaw);
        Vector Pos = Snake.Now_Pos.p;
        Snake.Roll -= M_PI/18;
        Frame AB(Rotation::RPY(Snake.Roll,Snake.Pitch,Snake.Yaw),Pos);
        Snake.Goal_Pos = AB;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}

void MainWindow::on_yup_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        Frame AS = Snake.Now_Pos;
        Snake.Now_Pos.M.GetRPY(Snake.Roll,Snake.Pitch,Snake.Yaw);
        Vector Pos = Snake.Now_Pos.p;
        Snake.Pitch += M_PI/18;
        Frame AB(Rotation::RPY(Snake.Roll,Snake.Pitch,Snake.Yaw),Pos);
        Snake.Goal_Pos = AB;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}

void MainWindow::on_ydown_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        Frame AS = Snake.Now_Pos;
        Snake.Now_Pos.M.GetRPY(Snake.Roll,Snake.Pitch,Snake.Yaw);
        Vector Pos = Snake.Now_Pos.p;
        Snake.Pitch -= M_PI/18;
        Frame AB(Rotation::RPY(Snake.Roll,Snake.Pitch,Snake.Yaw),Pos);
        Snake.Goal_Pos = AB;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}

void MainWindow::on_zup_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        Frame AS = Snake.Now_Pos;
        Snake.Now_Pos.M.GetRPY(Snake.Roll,Snake.Pitch,Snake.Yaw);
        Vector Pos = Snake.Now_Pos.p;
        Snake.Yaw += M_PI/18;
        Frame AB(Rotation::RPY(Snake.Roll,Snake.Pitch,Snake.Yaw),Pos);
        Snake.Goal_Pos = AB;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}

void MainWindow::on_zdown_clicked()
{
    Snake.IfClick = true;
    if(WholeMutex.try_lock()){
        Frame AS = Snake.Now_Pos;
        Snake.Now_Pos.M.GetRPY(Snake.Roll,Snake.Pitch,Snake.Yaw);
        Vector Pos = Snake.Now_Pos.p;
        Snake.Yaw -= M_PI/18;
        Frame AB(Rotation::RPY(Snake.Roll,Snake.Pitch,Snake.Yaw),Pos);
        Snake.Goal_Pos = AB;
        Snake.NextMoveFlag = false;
        WholeMutex.unlock();
    }
}

void MainWindow::on_Joint1Add_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        //cout<<RED<<"lueluelueluelue"<<endl;
        Snake.Jnt_Goal.data[0] += SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint2Add_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[1] += SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint3Add_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[2] += SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint4Add_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[3] += SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint5Add_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[4] += SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint6Add_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[5] += SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint1D_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[0] -= SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint2D_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[1] -= SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint3D_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[2] -= SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint4D_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[3] -= SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint5D_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[4] -= SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_Joint6D_clicked()
{
    if(Mutex.try_lock())
    {
        //char Buf[16];
        Snake.Jnt_Goal.data[5] -= SetSpeed/180.0*M_PI;
        Snake.NextMoveFlag = false;
        //Snake.JntToInt(Buf);
        Mutex.unlock();
    }
}

void MainWindow::on_NextMove_clicked()
{
    if(Mutex.try_lock())
    {
        Snake.MoveAction += 1;
        Snake.NextMoveFlag = true;
        Snake.firstFlag = true;
        //Snake.IfInit = false;
        Mutex.unlock();
    }
}
