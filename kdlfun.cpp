#include <mainwindow.h>
#include <kdl.h>
double IntToAng(int A,int B){
    double Set=0.0;
    if(B!=1&&B!=4)//Here
        Set = A/4096.0*360.0/180.0*M_PI - 180/180.0*M_PI ;
    else
        Set =  A/1024.0*300.0/180.0*M_PI - 150/180.0*M_PI;

    if(B==2)
        return -1*Set;
    else
        return Set;
}
void  KDLfun::SendMassage(){

    emit SendSignal();

}
void KDLfun::JntToInt(char *Sendbuf){
    int send[6];



    for(int i =0;i<6;i++){
        if(i==1||i==4)
            send[i] = this->Jnt_Goal.data[i]/M_PI*180/300*1024 +512 ;
        else
            send[i] = this->Jnt_Goal.data[i]/M_PI*180/360*4096 + 2048;
    }
    Sendbuf[0] = 0xFF;
    Sendbuf[1] = 0xFE;
    int j = 2;
    for(int i = 0;i<6;i++){
        Sendbuf[j] = send[i]%256;
        Sendbuf[j+1] = send[i]/256;
        j+=2;
    }
    //Sendbuf[14] = this->GraspFlag|0xf0;
    Sendbuf[14] = this->Grab1;
    Sendbuf[15] = this->Grab2;
    Sendbuf[16] = this->MoveAction;
    if(this->First)
    {
        int j = 2;
        for(int i = 0;i<6;i++){
            Sendbuf[j] = 0;
            Sendbuf[j+1] = 0;
            j+=2;
        }
        this->First = false;
    }
}


void KDLfun::IntToAngle(const int* serial){


    if(serial[0] == 0XFF && serial[1] == 0XFE )
    {
        if(serial[2]==0XFF&&serial[3]==0XFF)
            return;
        //cout<<"Fit Now"<<endl;
        //double buf[6];
        int j = 2;
        for(int i = 0;i<6;i++){
            this->Jnt_Now.data[i] = serial[j]+serial[j+1]*256;
            //cout<<"1"<<endl;
            this->Jnt_Now.data[i] = IntToAng(Jnt_Now.data[i],i);
            j+=2;
        }
        if(this->JntGFlag)
        {
            this->Jnt_Goal = this->Jnt_Now;
            this->JntGFlag =false ;
        }
      //  for(int i =0;i<6;i++)
   //        cout<<"lue"<<this->Jnt_Now.data[i]/M_PI*180<<" ";
     //   cout<<endl;
        this->IfInit = true;
        if(this->firstFlag)
        {
            this->Jnt_Goal = Jnt_Now;
            this->firstFlag = false;
        }
    }
    else{

        //cout<<(int)serial[0]<<" "<<(int)serial[1]<<endl;
        //cout<<(int)0XFF<<" "<<(int)0XFE<<endl;
        //cout<<"Not fit the head"<<endl;
        return;
    }

}


void KDLfun::GetForwardKDL(JntArray init){

    Frame pos_goal;
    ChainFkSolverPos_recursive fwdkin(snake);
    fwdkin.JntToCart(init,pos_goal);
    this->Now_Pos = pos_goal;
    //cout<<this->Now_Pos.p<<endl;
}

void KDLfun::Getinkdl(){
    boost::timer timer;
    int num_of_trials = 10000;
    int total_number_of_iter = 0;
    int n = this->snake.getNrOfJoints();
    int nrofresult_ok = 0;
    int nrofresult_minus1=0;
    int nrofresult_minus2=0;
    int nrofresult_minus3=0;
    int min_num_of_iter = 10000000;
    int max_num_of_iter = 0;
    Eigen::Matrix<double,6,1> L;
    L(0)=1;L(1)=1;L(2)=1;
    L(3)=0.01;L(4)=0.01;L(5)=0.01;
    ChainFkSolverPos_recursive fwdkin(snake);
    ChainIkSolverPos_LMA solver(snake,L);
    JntArray q(n);
    JntArray q_init(n);
    JntArray q_sol(n);
    Frame pos_goal,pos_reached,pos_now;
    pos_goal = this->Goal_Pos;
    q_init = this->Jnt_Now;
    fwdkin.JntToCart(q_init,pos_now);
    for (int trial=0;trial<num_of_trials;++trial) {
        int retval;
        retval = solver.CartToJnt(q_init,pos_goal,q_sol);
        switch (retval) {
        case 0:
            nrofresult_ok++;
            break;
        case -1:
            nrofresult_minus1++;
            break;
        case -2:
            nrofresult_minus2++;
            break;
        case -3:
            nrofresult_minus3++;
            break;
        }
//        if (retval !=0) {
//            cout <<WHITE<< "-------------- failed ------------------" << endl;
//            cout << "pos " << pos_goal << endl;
//            cout << "reached pos " << solver.T_base_head << endl;
//            cout << "TF from pos to head \n" << pos_goal.Inverse()*solver.T_base_head << endl;
//            cout << "gradient " << solver.grad.transpose() << endl;
//            cout << "q   " << q.data.transpose()/M_PI*180.0 << endl;
//            cout << "q_sol " << q_sol.data.transpose()/M_PI*180.0 << endl;
//            cout << "q_init " << q_init.data.transpose()/M_PI*180.0 << endl;
//            cout << "return value " << retval << endl;
//            cout << "last #iter " << solver.lastNrOfIter << endl;
//            cout << "last diff  " << solver.lastDifference << endl;
//            cout << "jacobian of goal values ";
//            solver.display_jac(q);
//            std::cout << "jacobian of solved values ";
//            solver.display_jac(q_sol);
//        }
        if(retval==0){

            this->IfClick = false;
            this->Jnt_Goal = q_sol;

            fwdkin.JntToCart(q_sol,pos_reached);
            this->Jnt_Goal = q_sol;
            cout<<BOLDYELLOW <<"Succeed ik "<<endl;//endl<<q_sol.data<<endl;
            for(int i = 0;i<6;i++)
            {
                cout<<q_init.data[i]/M_PI*180<<" ";
            }
            cout<<endl;
            break;
        }
       // else
      //  {
            //cout<<"unsolved"<<endl;
      //  }
    }


    ////////////////////////
    for(int i =0;i<6;i++)
    {
        if(i==2)
           this->Jnt_Goal.data[i] = -1*this->Jnt_Goal.data[i];
    }

    double el = timer.elapsed();
    cout <<WHITE<< "pos_now "<<endl << pos_now << endl;
   // cout << q_init.data <<endl;
    cout << "pos_goal "<<endl << pos_goal << endl;
    cout << "reached_pos " <<endl<< solver.T_base_head << endl;
    cout<<"q_sol"<<endl;
    for(int i=0;i<6;i++)
        cout<<q_sol.data[i]*180/M_PI<<" ";
    cout<<endl;
    cout << "elapsed time " << el << endl;
//    //cout << "estimate of average time per invposkin (ms)" << el/num_of_trials*1000 << endl;
    //cout << "estimate of longest time per invposkin (ms) " << el/total_number_of_iter*max_num_of_iter *1000 << endl;
    //cout << "estimate of shortest time per invposkin (ms) " << el/total_number_of_iter*min_num_of_iter *1000 << endl;

}


