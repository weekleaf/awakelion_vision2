#include "auto_aim/AngleSolver/Filter.h"

//一阶卡尔曼预测

KF_two::KF_two(){
    //状态协方差矩阵附初值
    Eigen::MatrixXd P_in = Eigen::MatrixXd(6,6);
    P_in <<1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    P = P_in;

    //过程噪声矩阵附初值
    Eigen::MatrixXd Q_in(6,6);
    Q_in<<1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Q = Q_in;

    //测量矩阵附初值
    Eigen::MatrixXd H_in(6,6);
    H_in<<1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    H = H_in;

    //测量噪声矩阵附初值
    Eigen::MatrixXd R_in(6,6);
    R_in<<1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    R = R_in;
}

KF_two::KF_two(Eigen::MatrixXd P_in , Eigen::MatrixXd Q_in,Eigen::MatrixXd H_in,Eigen::MatrixXd R_in){
    P = P_in;
    Q = Q_in;
    H = H_in;
    R = R_in;
}



void KF_two::set_x(Eigen::VectorXd x,Eigen::MatrixXd _F){
    F = _F;
    x_ = x;
    is_set_x = true;
}

void KF_two::set_x(Eigen::VectorXd x){
    x_ = x;
    is_set_x = true;
}

void KF_two::Prediction(Eigen::MatrixXd _F){
    F = _F;
    x_ = F * x_;
    P = F*P*F.transpose() + Q;
}

//更新状态
void KF_two::update(Eigen::VectorXd z,Eigen::MatrixXd _F){
    F = _F;
        Eigen::MatrixXd y = z - H*x_;
        Eigen::MatrixXd S = H*P*H.transpose() + R;
        Eigen::MatrixXd K = P*H.transpose()*S.inverse();
    x_ = x_ + (K*y);
    int size = x_.size();

    if(size == 4){
        Eigen::MatrixXd I(4,4);
        I << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
        P = (I - K*H)*P;
    }

    if(size == 2){
        Eigen::MatrixXd I(2,2);
        I << 1, 0,
             0, 1;
            P = (I - K*H)*P;
    }

    if(size == 6){
        Eigen::MatrixXd I(6,6);
        I << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            P = (I - K*H)*P;
    }

    if(size == 4){
        Eigen::MatrixXd I(4,4);
        I << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
            P = (I - K*H)*P;
    }
}
