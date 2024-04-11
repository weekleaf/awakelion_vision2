#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <math.h>
// #include <QTextStream>
// #include <QFile>

#include "auto_aim/Settings/Settings.h"
#include "auto_aim/AngleSolver/GravityCompensateResolve.h"
#include"auto_aim/AngleSolver/Filter.h"

#define grav 9.8


class AngleSolver
{
public:
    /**
     * @brief  缓冲阶段
     * @param  最优装甲板
     * @param  收数结构体
     * @author 吴凯杰
     * @date   2023-
     */
    void BufferSetFilter(struct ArmorInfo & BestArmor,struct CarData CarDatas);                       //击打缓冲

    /**
     * @brief  解析角度
     * @param  最优装甲板
     * @param  CarDatas
     * @author 吴凯杰
     * @date   2023-
     */
     void GetArmorAngle(struct ArmorInfo & BestArmor, struct CarData CardDatas);

     /**
      * @brief  获取2D点
      * @param  最优装甲板
      * @param  2D点
      * @author 吴凯杰
      * @date   2023-
      */
    void GetPoint2D(ArmorInfo & BestArmor,std::vector<cv::Point2f>&point2D);

    /**
     * @brief  获取3D点
     * @param  最优装甲板
     * @param  3D点
     * @author 吴凯杰
     * @date   2023-
     */
    void GetPoint3D(ArmorInfo & BestArmor,std::vector<cv::Point3f>&point3D);

    /**
     * @brief  解决pnp问题
     * @param  最优装甲板
     * @param  2D点
     * @param  3D点
     * @author 吴凯杰
     * @date   2023-
     */
    void CountAngleXY(const std::vector<cv::Point2f>&point2D,const std::vector<cv::Point3f>&point3D, ArmorInfo & BestArmor);

    /**
     * @brief  解决坐标系转化问题
     * @param  tx，ty，tz
     * @param  Carpitch，Caryaw
     * @author 吴凯杰
     * @date   2023-
     */
    void ShootAdjust(float & tx,float & ty,float & tz,float Carpitch,float Caryaw);

    /**
     * @brief  移动预测弹道补偿计算
     * @param  最优装甲板
     * @param  CarDatas
     * @param  转化后的坐标
     * @author 吴凯杰
     * @date   2023-
     */
    void MovePrediction( ArmorInfo &BestArmor, CarData CarDatas, cv::Point3f RelativePoisition);

    /**
     * @brief  计算射击缓冲及时间
     * @param  txtytz
     * @param  CarDatas
     * @param  转化后的坐标
     * @return 击打缓冲计算返回
     * @author 吴凯杰
     * @date   2023-
     */
    Angle_t ComputeShootTime(float tx, float ty, float distance,struct CarData CarDatas);

    /**
     * @brief  初始化滤波
     * @param  最优装甲板
     * @param  CarDatas
     * @param  时间t
     * @author 吴凯杰
     * @date   2023-
     */
    cv::Point3f SetKF(ArmorInfo & BestArmor,CarData CarDatas,double t);

    /**
     * @brief  第一次发现目标
     * @param  最优装甲板
     * @author 吴凯杰
     * @date   2023-
     */
    void FirstFind(ArmorInfo BestArmor);

    /**
     * @brief  首次击打相同目标
     * @param  最优装甲板
     * @param  CarDatas
     * @author 吴凯杰
     * @date   2023-
     */
    void FirstSetFilter(ArmorInfo & BestArmor,CarData carDatas);

    /**
     * @brief  连续击打同一目标
     * @param  最优装甲板
     * @param  CarDatas
     * @author 吴凯杰
     * @date   2023-
     */
    void ContinueSetFilter(ArmorInfo & BestArmor,CarData carDatas);

    AngleSolver(const char *camera_param_file_name, double z_scale = 1.0);

    /**
     * @brief  设置相机安装参数
     * @param  第一个参数含义
     * @author 梁尧森
     * @date   2018.9.21
     */
    void setRelationPoseCameraPTZ(const double ptz_camera_x, const double ptz_camera_y,
                                  const double ptz_camera_z, double y_offset_barrel_ptz);

    /**
     * @brief  设置目标尺寸
     * @param  目标矩形真实宽度
     * @param  目标矩形真实高度
     * @return 返回值含义
     * @author 梁尧森
     * @date   2018.9.21
     */
    void setTargetSize(double width, double height);

    /**
     * @brief  解析角度
     * @author 梁尧森
     * @date   2018.9.21
     */
    bool getAngle(std::vector<cv::Point2f> target2d, double bullet_speed,
                  double current_ptz_angle=0);

    /**
     * @brief  solvePnP
     * @author 参与开发人员
     * @date   2018-
     */
    void solvePnP4Points(const std::vector<cv::Point2f> &points2d,
                         cv::Mat &rot, cv::Mat &trans);

    /**
     * @brief  相机坐标转换到PTZ坐标
     * @param  目标在相机坐标下的位置
     * @param  目标在PTZ坐标下的位置
     * @author 梁尧森
     * @date   2018.9.21
     */
    void tranformationCamera2PTZ(const cv::Mat &pos, cv::Mat &transed_pos);

    /**
     * @brief  根据目标在PTZ坐标中的位置，计算偏移角度，使枪管瞄准目标
     * @param  第一个参数含义
     * @author 梁尧森
     * @date   2018.9.21
     */
    void adjustPTZ2Barrel(const cv::Mat &pos_in_ptz,double &angle_x, double &angle_y,
                          double bullet_speed, double current_ptz_angle);

    /**
     * @brief  rectPnpSolver
     * @param  camera_param_file_name
     * @author 参与开发人员
     * @date   2018-
     */
    void rectPnpSolver(const char *camera_param_file_name);





    /**
     * @brief  获取矩形四个角点的坐标
     * @author 梁尧森
     * @date   2018.9.21
     */
    void getTarget2dPoinstion(const cv::RotatedRect &rect, std::vector<cv::Point2f> &target2d,
                              const cv::Point2f &offset);



    /**
     * @brief
     * @param  第一个参数含义
     * @return 返回角度、距离、姿态角等信息
     * @author 梁尧森
     * @date   2018.9.21
     */
    double getPitRef();

    /**
     * @brief
     * @param  第一个参数含义
     * @return 返回角度、距离、姿态角等信息
     * @author 梁尧森
     * @date   2018.9.21
     */
    double getYawRef();

    /**
     * @brief  单目测距
     * @param  armor_rect
     * @param  dist
     * @author 陈强
     * @date   2021-
     */
    void getDistanceDanmu(std::vector<cv::Point2f>armor_rect, double &dist);

    /**
     * @brief  单目测距pnp算法
     * @param  2d点
     * @param  dist
     * @author 吴凯杰
     * @date   2022-
     */
    void getDistancePnP(const std::vector<cv::Point2f> &points2d,double &dist);

public:
    cv::Mat cam_matrix;                 // 内参矩阵
    cv::Mat distortion_coeff;           // 畸变系数
    double width_target;                // 目标矩形真实宽度
    double height_target;               // 目标矩形真实高度
    std::vector<cv::Point3f> point3d;   // 目标三维坐标
    cv::Mat rot;                        // 旋转矩阵
    cv::Mat position_in_camera;         // 偏移向量
    cv::Mat position_in_ptz;            // 目标在PTZ坐标中的位置
    double pit_ref;                     // 俯仰角
    double yaw_ref;                     // 偏航角
private:
    cv::Mat rot_camera2ptz;             // 旋转矩阵（相机坐标转换到PTZ坐标）
    cv::Mat trans_camera2ptz;           // 偏移向量（相机坐标转换到PTZ坐标）
    double offset_y_barrel_ptz;         // 云台中心与枪管的偏移量
    double scale_z;                     // 放缩比例

    float ptz_camera_y = 2;                  //相机与云台垂直方向距离差
    float ptz_camera_z = 20;                  //相机与云台轴向方向距离差
    float ptz_camera_x = -50;                    //相机与云台水平方向距离差

    float ptz_camera_pitch ;            //对应绕x旋转角度,弧度，角度乘0.017453
    float ptz_camera_yaw;               //对应绕y旋转角度，弧度
    float ptz_camera_roll ;                 //对应绕z旋转角度，弧度
};

#endif
