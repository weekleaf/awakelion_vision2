#ifndef FILTER_H
#define FILTER_H

#include<eigen3/Eigen/Dense>

//一阶卡尔曼预测
class KF_two{
private:
    double pitch;                                       //测量pitch角度
    double yaw;                                       //测量yaw角度

public:
    KF_two();                                          //无参构造
    KF_two(Eigen::MatrixXd P_in , Eigen::MatrixXd Q_in,Eigen::MatrixXd H_in,Eigen::MatrixXd R_in);//带参数的构造函数，创建类对象的时候不附初值不会调用

    /**
     * @brief  预测公式
     * @param  状态转移矩阵
     * @author 吴凯杰
     * @date   2023-
     */
    void Prediction(Eigen::MatrixXd _F);

    /**
     * @brief  更新公式
     * @param  观测值，状态转移矩阵
     * @author 吴凯杰
     * @date   2023-
     */
    void update(Eigen::VectorXd z,Eigen::MatrixXd _F);

    /**
     * @brief  状态矩阵附初值
     * @param  状态矩阵，状态转移矩阵
     * @author 吴凯杰
     * @date   2023-
     */
    void set_x(Eigen::VectorXd x,Eigen::MatrixXd _F);

    /**
     * @brief  状态矩阵附初值
     * @param  状态矩阵
     * @author 吴凯杰
     * @date   2023-
     */
    void set_x(Eigen::VectorXd x);

    Eigen::VectorXd x_;                         //状态向量
    Eigen::MatrixXd F;                           //状态转移矩阵

    Eigen::MatrixXd P;                          //状态协方差矩阵
    Eigen::MatrixXd Q;                          //过程噪声
    Eigen::MatrixXd H;                          //测量矩阵

    Eigen::MatrixXd R;                          //测量噪声矩阵
    Eigen::MatrixXd K;                          //卡尔曼增益
    bool is_set_x = false;                     //判断是否赋初值

};

#endif // FILTER_H
