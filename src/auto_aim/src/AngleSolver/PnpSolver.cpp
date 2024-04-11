#include "auto_aim/AngleSolver/PnpSolver.h"

#define SHOOT_DELAY_TIME 0                //发弹延迟

KF_two KF_forecast;

//波形？
float _time = 0;


//用于缓冲计算
float SendPitch = 0;                //记录上一帧pitch发送角度量
float SendYaw = 0;                //记录上一帧yaw发送角度量
//预测计算保留量
CarData old_carDatas;                //上次收数

float v_tx_old;                     //线速度保留量
float v_ty_old;
float v_tz_old;
float p_tx_old;                  //位置保留量
float p_ty_old;
float p_tz_old;

CarData carDatasOld;                //上次收数
cv::Point3f old_Poisition;              //上级的二阶预测的位置
cv::Point3f old_objectP;
//记录前后时间
double  old_CarTime = 0;

AngleSolver::AngleSolver(const char *camera_param_file_name, double z_scale)
{
    rectPnpSolver(camera_param_file_name);
    scale_z = z_scale;

    rot_camera2ptz = cv::Mat::eye(3, 3, CV_64FC1);
    trans_camera2ptz = cv::Mat::zeros(3, 1, CV_64FC1);
    offset_y_barrel_ptz = 0;
}




//以下为修改：：

void AngleSolver::BufferSetFilter( ArmorInfo & BestArmor, CarData CarDatas){
    //Send为发送角度，run为一定时间t内云台转动角度量
    //add为t内目标估计运动量，add = v*t
    //新的发送量Send_now = Send + add - run
    if(!KF_forecast.is_set_x){
        BestArmor.pitch = SendPitch ;//- (CarDatas.pitch - old_carDatas.pitch);
        BestArmor.yaw = SendYaw;// - (CarDatas.yaw - old_carDatas.yaw);
//        old_carDatas = CarDatas;
        return ;
    }


    double t = CarDatas.BeginToNowTime - old_carDatas.BeginToNowTime;
    BestArmor.tx =old_objectP.x + KF_forecast.x_(3)/1000*t;
    BestArmor.ty =old_objectP.y + KF_forecast.x_(4)/1000*t;
    BestArmor.tz =old_objectP.z + KF_forecast.x_(5)/1000*t;
    Angle_t at = ComputeShootTime(BestArmor.tx,BestArmor.ty,BestArmor.tz,CarDatas);

    BestArmor.pitch = at.pitch   /*- (CarDatas.pitch - old_carDatas.pitch)*/;
    BestArmor.yaw = at.yaw /*- (CarDatas.yaw - old_carDatas.yaw)*/;
    BestArmor.tx = old_objectP.x;
    BestArmor.ty = old_objectP.y;
    BestArmor.tz = old_objectP.z;
}


void AngleSolver::GetArmorAngle(ArmorInfo & BestArmor,CarData CarDatas){
    std::vector<cv::Point2f>point2D;
    std::vector<cv::Point3f>point3D;

    GetPoint2D(BestArmor,point2D);                                                                                                     //矩形转换为2d坐标
    GetPoint3D(BestArmor,point3D);                                                                                                     //矩形转换为3d坐标
    CountAngleXY(point2D,point3D,BestArmor);                                                                             //解决pnp问题
    //相机坐标与云台初始枪口坐标转换,坐标系做云台当前角度反向旋转得到绝对坐标
    cv::Point3f RelativePoisition = cv::Point3f(BestArmor.tx,BestArmor.ty,BestArmor.tz);              //保留相对坐标
    ShootAdjust(BestArmor.tx,BestArmor.ty,BestArmor.tz,CarDatas.pitch,CarDatas.yaw);     //转换为绝对坐标   //带回txtytz的值 为绝对坐标
    //RelativePoisition.y = BestArmor.ty;                       //ty坐标不使用绝对坐标
    RelativePoisition.x = BestArmor.tx;
    RelativePoisition.y = BestArmor.ty;
    RelativePoisition.z = BestArmor.tz;
    BestArmor.pnp_pitch = atan2( BestArmor.ty, BestArmor.tz)*180/PI;
    BestArmor.pnp_yaw = atan2( BestArmor.tx, BestArmor.tz)*180/PI;
      //移动预测
    MovePrediction(BestArmor,CarDatas,RelativePoisition);//src 不启用加速度则是无用的变量 bykj
}

void AngleSolver::GetPoint2D(ArmorInfo & BestArmor,std::vector<cv::Point2f>&point2D ){
    cv::Point2f lu,ld,ru,rd;

    lu = BestArmor.pos.p[1];
    ld = BestArmor.pos.p[0];
    ru = BestArmor.pos.p[2];
    rd = BestArmor.pos.p[3];

//     cout<<"lu:"<<lu<<endl;
    //顺时针
    point2D.clear();//先清空再存入
    point2D.push_back(ld);
    point2D.push_back(lu);
    point2D.push_back(ru);
    point2D.push_back(rd);
}

void AngleSolver::GetPoint3D(ArmorInfo & BestArmor,std::vector<cv::Point3f>&point3D ){
    float fHalfX=0;
    float fHalfY=0;
    if(BestArmor.is_small)                                  //大装甲
    {
//        std::cout<<"小"<<endl;
        fHalfX=width_target/2.0;
        fHalfY=height_target/2.0;
    }else{                                                                            //小装甲
//         std::cout<<"大"<<endl;
        fHalfX=width_target/2.0;
        fHalfY=height_target/2.0;
    }
    point3D.push_back(cv::Point3f(-fHalfX,-fHalfY,0.0));//ld
    point3D.push_back(cv::Point3f(-fHalfX,fHalfY,0.0));//lu
    point3D.push_back(cv::Point3f(fHalfX,fHalfY,0.0));//ru
    point3D.push_back(cv::Point3f(fHalfX,-fHalfY,0.0));//rd
}

void AngleSolver::CountAngleXY(const std::vector<cv::Point2f>&point2D,const std::vector<cv::Point3f>&point3D,ArmorInfo & BestArmor){
    cv::Mat rvecs=cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat tvecs=cv::Mat::zeros(3,1,CV_64FC1);

    cv::solvePnP(point3D,point2D,cam_matrix,distortion_coeff,rvecs,tvecs);

//   cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, rvec, trans/*, true, CV_P3P*/); //自瞄
//     cout<<"旋转向量："<<rvecs<<endl;
//     cout<<"平移向量："<<tvecs<<endl;
    //    x坐标分量指向世界坐标系的zuo侧方向。
    //    y坐标分量指向世界坐标系的xia方方向。
    //    z坐标分量指向世界坐标系的前方方向。
    double tx = tvecs.ptr<double>(0)[0];
    double ty = tvecs.ptr<double>(0)[1];
    double tz = tvecs.ptr<double>(0)[2];

    BestArmor.tx = -tx;
    BestArmor.ty = ty;
    BestArmor.tz = tz;

    BestArmor.pitch = atan2( BestArmor.ty, BestArmor.tz)*180/PI;//atan2(y, x) 表示计算 y/x 的反正切值。
    BestArmor.yaw = atan2( BestArmor.tx, BestArmor.tz)*180/PI;
    BestArmor.before_pnp_pitch = BestArmor.pitch;
    BestArmor.before_pnp_yaw = BestArmor.yaw;
    std::cout<<"pnp解算后之解算出来的 此时的x为："<<BestArmor.tx<<"   此时y为："<<BestArmor.ty<<"   此时z为："<<BestArmor.tz;
    std::cout<<"此时的pitch角为："<<BestArmor.pitch<<"   此时yaw角为："<<BestArmor.yaw<<std::endl;
}

void AngleSolver::ShootAdjust(float & tx,float & ty,float & tz,float Carpitch,float Caryaw){
    //角度转弧度
    // std::cout<<"before_change_tx:"<<tx<<"   before_change_ty:"<<ty<<"   before_change_tz"<<tz<<std::endl;
     //std::cout<<"car_pitch:"<<Carpitch<<"car_yaw:"<<Caryaw<<std::endl;
    Carpitch *=PI/180;//陀螺仪的数据
    Caryaw *=PI/180;

    //绕roll轴旋转，即为绕z轴旋转
    Eigen::MatrixXd r_Roll(3,3);
    r_Roll<<1 , 0 , 0,
                    0 , cos(ptz_camera_roll) , sin(ptz_camera_roll),
                    0 , -sin(ptz_camera_roll) , cos(ptz_camera_roll);
    //绕pitch轴旋转，即为绕x轴旋转
    Eigen::MatrixXd r_Pitch(3,3);
    r_Pitch<<cos(ptz_camera_pitch) , 0 , -sin(ptz_camera_pitch),
                        0 , 1 , 0,
                        sin(ptz_camera_pitch) , 0 , cos(ptz_camera_pitch);
    //绕yaw轴旋转，即为绕y轴旋转
    Eigen::MatrixXd r_Yaw(3,3);
    r_Yaw<<cos(ptz_camera_yaw) , sin(ptz_camera_yaw) , 0,
                     -sin(ptz_camera_yaw) , cos(ptz_camera_yaw) , 0 ,
                     0 , 0 , 1;

    Eigen::VectorXd original(3,1);             //按z，x，y传入，即变化对应到左手坐标系  xyz
    original<<tx,ty,tz;
    //std::cout<<"before change:"<<original<<std::endl;

    //平移变换,先旋转再平移
    Eigen::VectorXd translation(3,1);
    translation<<ptz_camera_x,ptz_camera_y,ptz_camera_z;//相机到云台的差距 bykj
    //original = original + translation;

    std::cout<<"after change:"<<original<<std::endl;
    Eigen::VectorXd change(3,1);
    change = original;
    //旋转变换
//    change =  r_Roll * original;
//    change = r_Pitch*change;
//    change = r_Yaw*change;
    //以上部分的偏移参数调节

    tx = change(0);
    ty = change(1);
    tz = change(2);
   // std::cout<<"after_change_tx:"<<tx<<"   after_change_ty:"<<ty<<"   after_change_tz"<<tz<<std::endl;   //left up  +
  //  std::cout<<"car_pitch:"<<Carpitch<<"car_yaw:"<<Caryaw<<std::endl;
    //去掉车云台旋转相对影响,坐标系转换到相对初始位的绝对坐标
    //pitch转换
    Eigen::MatrixXd r_pitch_car(3,3);
    r_pitch_car<<1 , 0 , 0,
                        0 , cos(Carpitch) , -sin(Carpitch),
                        sin(Carpitch) , 0 , cos(Carpitch);
    //yaw转换
    Eigen::MatrixXd r_yaw_car(3,3);
    r_yaw_car<<cos(Caryaw) , 0 , sin(Caryaw),
               0, 1 , 0 ,
             -sin(Caryaw), 0 , cos(Caryaw);
    change = r_pitch_car*change;
    change = r_yaw_car*change;

    tx = change(0);
    ty = change(1);
    tz = change(2);
    std::cout<<"after_change_tx:"<<tx<<"   after_change_ty:"<<ty<<"   after_change_tz"<<tz<<std::endl;   //left up  +
}

void AngleSolver::MovePrediction(ArmorInfo &BestArmor, CarData CarDatas,cv::Point3f RelativePoisition){
    //抬升角度计算,得到绝对角度
    Angle_t ShootBuff = ComputeShootTime(RelativePoisition.x,RelativePoisition.y,RelativePoisition.z,CarDatas);
//    float old_CurveValue = RelativePoisition.y;                         //波形绘制
//     float old_CurveValue2 = RelativePoisition.z;                         //波形绘制

     if(ShootBuff.t == 0){
         //出现可能性极小的无解情况,出现往往时算法错误
//         BestArmor.Armor.yaw = atan2(BestArmor.Armor.tx,BestArmor.Armor.tz)*180/CV_PI;
//         BestArmor.Armor.pitch = atan2(BestArmor.Armor.ty,BestArmor.Armor.tz)*180/CV_PI;
         std::cout<<"角度计算无解"<<std::endl;
         ShootBuff.t = 15;
     }else{
         //传参
         BestArmor.pitch = ShootBuff.pitch;
         BestArmor.yaw = ShootBuff.yaw;
     }


     //卡尔曼滤波
     cv::Point3f poisition = SetKF(BestArmor,CarDatas,ShootBuff.t);
     Angle_t AT = ComputeShootTime(poisition.x,poisition.y,poisition.z,CarDatas);       //计算打击时间和偏向角
     //更新打击偏向角
     BestArmor.pitch = AT.pitch  ;
     BestArmor.yaw = AT.yaw;
     std::cout<<"经过滤波器后：  此时的pitch角为："<<BestArmor.pitch<<"    此时yaw角为："<<BestArmor.yaw<<std::endl;
     SendPitch = AT.pitch;
     SendYaw = AT.yaw;
     BestArmor.tx = poisition.x;
     BestArmor.ty = poisition.y;
     BestArmor.tz = poisition.z;
     old_objectP.y = BestArmor.ty;                                                 //保留位置坐标,为计算缓冲做准备
     old_objectP.x = BestArmor.tx;
     old_objectP.z = BestArmor.tz;
//     //绘制波形
//     DrawCurve drawCurve;
//     if(KF_forecast.is_set_x){
//         Angle_t AT2 = ComputeShootTime(poisition.x,old_CurveValue + v*_time,old_CurveValue2+vz*_time,CarDatas);
//         drawCurve.InsertData(AT2.pitch,BestArmor.Armor.pitch,"Actual value","Predictive value");
////         drawCurve.InsertData(KF_forecast.x_(1));
////         drawCurve.InsertData(old_CurveValue);

//     }
//     old_carDatas = CarDatas;
     return;
}

Angle_t AngleSolver::ComputeShootTime(float tx, float ty, float distance,struct CarData CarDatas){
    //g表示重力加速度，LevelDistance表示水平距离(sqrt(pow(tz,2)+pow(tx,2)),HeightDistance为垂直高度ty，v为射速，tan_angle为所要求俯仰角的正切值
    //计算公式: -0.5g*pow(LevelDistance,2)/pow(V,2)*pow(tan_angle,2) + tan_angle*LevelDistance - 0.5g*pow(LevelDistance,2)/pow(V,2) - HeightDistance
    //计算时间使用t = LevelDistance/(v*cos_angle)


    //单位转换
    tx /= 100.0;
    ty /= 100.0;
    float tz = distance/100.0;
//    std::cout<<"缓冲计算tx:"<<tx<<"  ty:"<<ty<<" tz:"<<distance<<std::endl;
    float speed = CarDatas.ShootSpeed;
    speed = 25;            //不用电控发来的速度 自己给speed赋值
    double a = -0.5*grav*(pow(tz,2)+pow(tx,2));
    double b = sqrt((pow(tz,2)+pow(tx,2)))*pow(speed,2);
    double c = -0.5*grav*(pow(tz,2)+pow(tx,2)) - pow(speed,2)*ty;
    //判别式
    double Discriminant =pow(b,2)-4*a*c;// pow(a,2)+pow(b,2)-4*a*c;
//    cout<<"判别式:"<<Discriminant<<"   a:"<<a<<"   b:"<<b<<"   c:"<<c<<endl;
    Angle_t ShootBuff = {0,0,0,0};
    if(Discriminant<0)
        return ShootBuff;
    double angle_tan_1 = atan((-b + sqrt(Discriminant))/(2*a))*180/PI;
    double angle_tan_2 = atan((-b - sqrt(Discriminant))/(2*a))*180/PI;
//    double angle_tan = ty/b;
//    double real_angle = fabs(angle_tan - angle_tan_1)<fabs(angle_tan - angle_tan_2)?angle_tan_1:angle_tan_2;
    //角度取舍,并转换为相对角度
    std::cout<<"计算所得打击角度1:"<<angle_tan_1<<"  计算所得打击角度2:"<<angle_tan_2<<std::endl;
    if(fabs(angle_tan_1)<=fabs(angle_tan_2)&&fabs(angle_tan_1)<45){
        ShootBuff.pitch = angle_tan_1;
    }else if(fabs(angle_tan_2)<45){
        ShootBuff.pitch = angle_tan_2;//- CarDatas.pitch
    }else{      //都不符合要求
        std::cout<<"计算解不符合实际"<<std::endl;
        return ShootBuff;
    }
    ShootBuff.yaw = atan2(tx,tz)*180/CV_PI;
    //std::cout<<"缓冲计算tx:"<<tx<<"  ty:"<<ty<<" tz:"<<tz<<"yaw"<<ShootBuff.yaw<<"最小角度"<<atan2(ty,tz)*180/PI<<std::endl;
    ShootBuff.t = tz/(speed*cos(ShootBuff.pitch*PI/180))*1000;
    std::cout<<"缓冲计算tx:"<<tx<<"  ty:"<<ty<<" tz:"<<tz<<"yaw"<<ShootBuff.yaw<<"击打时间:"<<ShootBuff.t<<std::endl;
    return ShootBuff;
}

cv::Point3f AngleSolver::SetKF(ArmorInfo &BestArmor, CarData CarDatas,double t){
    if(BestArmor.status == pattern::FirstFind){
        //清除波形保留信息
//        DrawCurve drawCurve;
//        drawCurve.ClearSaveData();

        FirstFind(BestArmor);  //第一次发现保留位置 bykj
        KF_forecast.is_set_x = false;
        //状态协方差矩阵重新复位
        Eigen::MatrixXd P_in = Eigen::MatrixXd(6,6);
        P_in << 1.0, 0.0, 0.0, 0.0,0.0,0.0,
                0.0, 1.0, 0.0, 0.0,0.0,0.0,
                0.0, 0.0, 1.0, 0.0,0.0,0.0,
                0.0, 0.0, 0.0, 1.0,0.0,0.0,
               0.0, 0.0, 0.0, 0.0,1.0,0.0,
               0.0, 0.0, 0.0, 0.0,0.0,1.0;
      KF_forecast.P = P_in;
        return cv::Point3f(BestArmor.tx,BestArmor.ty,BestArmor.tz);
    }else if(BestArmor.status == Shoot){

        if(!KF_forecast.is_set_x){
            //第一次连续射击
            std::cout<<"首次连续滤波!!!"<<std::endl;
            FirstSetFilter(BestArmor,CarDatas);//第一次击打保留位置 速度 is_set_x = true   bykj
        }else{
            std::cout<<"连续滤波!"<<std::endl;
            ContinueSetFilter(BestArmor,CarDatas);
        }

    }else{
        std::cout<<"卡尔曼滤波状态传入出错"<<std::endl;
    }

    //时间为击打时间与弹道延迟之和
    double ShootTime = t + SHOOT_DELAY_TIME;//+ CarDatas.BeginToNowTime;//子弹飞行时间 发弹延迟  一帧的时间
    //double ShootTime = t + SHOOT_DELAY_TIME+Recive.getClock() - CarDatas.BeginToNowTime;//子弹飞行时间 发弹延迟 ？ 一帧的时间
    std::cout<<"x速度："<<KF_forecast.x_(3)<<" y速度:"<<KF_forecast.x_(4)<<" z速度:"<<KF_forecast.x_(5)<<std::endl;
    std::cout<<"时间："<<ShootTime<<std::endl;

//    t= ShootTime/1000.0;

    cv::Point3f poistion;
    poistion.x = KF_forecast.x_(0) + KF_forecast.x_(3)/1000*ShootTime;   //x = x + v*t bykj
    poistion.y = KF_forecast.x_(1) + KF_forecast.x_(4)/1000*ShootTime;
    poistion.z = KF_forecast.x_(2) + KF_forecast.x_(5)/1000*ShootTime;

    _time = ShootTime; // 好像是测距滤波(波形？)相关 bykj

     //位置预测使用加速度,使用国际单位制
    //后面再决定是否使用 bykj
//    poistion.x += (0.5*(KF_forecast.x_(3)/1000 - v_tx_old)*10*pow(ShootTime/1000.0,2))*100;
//    poistion.y += (0.5*(KF_forecast.x_(4)/1000 - v_ty_old)*10*pow(ShootTime/1000.0,2))*100;
//    poistion.z += (0.5*(KF_forecast.x_(5)/1000 - v_tz_old)*10*pow(ShootTime/1000.0,2))*100;
//    std::cout<<"加速度:"<<fabs(KF_forecast.x_(3) - v_tx_old*1000)<<endl;
//    if(fabs(KF_forecast.x_(3) - v_tx_old*1000)>12){
//        BestArmor.IsShooting = false;
//    }

//    //debug
//    char test[100];
//    sprintf(test, "a:%0.4f", fabs(KF_forecast.x_(3) - v_tx_old*1000));
//    cv::putText(Src, test, cv::Point(20, 400), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, 8);

    v_tx_old = KF_forecast.x_(3)/1000;
    v_ty_old = KF_forecast.x_(4)/1000;
    v_tz_old = KF_forecast.x_(5)/1000;

      std::cout<<"1阶滤波位置:"<<poistion.x<<"  "<<poistion.y<<"  "<<poistion.z<<std::endl;
      return poistion;
}

void AngleSolver::FirstFind(ArmorInfo BestArmor){
        std::cout<<"第一次发现目标"<<std::endl;
        //状态保留
        p_tx_old = BestArmor.tx;
        p_ty_old = BestArmor.ty;
        p_tz_old = BestArmor.tz;

    }

void AngleSolver::FirstSetFilter(ArmorInfo & BestArmor,CarData carDatas){
    //std::cout<<"第一次滤波"<<std::endl;
    double t = carDatas.BeginToNowTime - carDatasOld.BeginToNowTime;        //计算两帧经过的时间
    if(t == 0){
        std::cout<<" 未收到时间！！！"<<std::endl;
        t = 15;         //未收到数
    }
    std::cout<<"当前电控发送时间差为："<<t<<std::endl;
    //目标移动速度,单位为cm/ms
    float v_tx_now = (BestArmor.tx - p_tx_old)/t;
    float v_ty_now =( BestArmor.ty - p_ty_old)/t;
    float v_tz_now = (BestArmor.tz - p_tz_old)/t;
    std::cout<<"第一次滤波算出位置差x为："<<BestArmor.tx - p_tx_old<<"y为："<<BestArmor.ty - p_ty_old<<"z为："<<BestArmor.tz - p_tz_old;
    std::cout<<"第一次滤波算出速度为：  vx is :"<<v_tx_now<<"   vy is :"<<v_ty_now<<"   vz is :"<<v_tz_now<<std::endl;
    //pnp解算出来的点会有细微的变化，装甲板静止的时候也可能会因为有抖动而令装甲板产生速度，设置一个阈值，排除这种情况 bykj
//    if( (fabs(v_tx_now)<0.25)/*||(fabs(v_ty_now)<0.1)||(fabs(v_tz_now)<0.2) */){
//        std::cout<<"当前装甲板速度舍弃！"<<std::endl;
//        v_tx_now = 0;
//        v_ty_now = 0;
//        v_tz_now = 0;
//    }
    Eigen::VectorXd x(6,1);
    x<<BestArmor.tx,BestArmor.ty,BestArmor.tz,v_tx_now*1000,v_ty_now*1000,v_tz_now*1000;
    KF_forecast.set_x(x);               //is_set_x = true;

    //传参
    p_tx_old = BestArmor.tx;
    p_ty_old = BestArmor.ty;
    p_tz_old = BestArmor.tz;

    old_objectP.x = p_tx_old;
    old_objectP.y = p_ty_old;
    old_objectP.z = p_tz_old;

    v_tx_old = v_tx_now;
    v_ty_old = v_ty_now;
    v_tz_old = v_tz_now;

}

void AngleSolver::ContinueSetFilter(ArmorInfo & BestArmor,CarData carDatas){
    //std::cout<<"连续滤波"<<std::endl;
    double t = carDatas.BeginToNowTime - old_carDatas.BeginToNowTime;
    //不发数的时候t=0
    if(t == 0){
        std::cout<<" 未收到时间！！！"<<std::endl;
        t = 15;         //单位ms
    }
    std::cout<<"当前电控发送时间差为："<<t<<std::endl;
    //得到真实测量值
    //目标移动速度,单位为cm/ms
    float v_tx_now = (BestArmor.tx - p_tx_old)/t;
    float v_ty_now = (BestArmor.ty - p_ty_old)/t;
    float v_tz_now = (BestArmor.tz - p_tz_old)/t;
    std::cout<<"连续滤波算出位置差x为："<<BestArmor.tx - p_tx_old<<"y为："<<BestArmor.ty - p_ty_old<<"z为："<<BestArmor.tz - p_tz_old<<std::endl;
    std::cout<<"连续滤波算出速度为：  vx is :"<<v_tx_now<<"   vy is :"<<v_ty_now<<"   vz is :"<<v_tz_now<<std::endl;
//    if( (fabs(v_tx_now)<0.005)/*||(fabs(v_ty_now)<0.1)||(fabs(v_tz_now)<2)*/ ){
//        std::cout<<"当前装甲板速度舍弃！"<<std::endl;
//        v_tx_now = 0;
//        v_ty_now = 0;
//        v_tz_now = 0;
//    }
    p_tx_old = BestArmor.tx;
    p_ty_old = BestArmor.ty;
    p_tz_old = BestArmor.tz;

    Eigen::VectorXd z(6,1);//观测值 bykj
    z<<BestArmor.tx,BestArmor.ty,BestArmor.tz,v_tx_now*1000,v_ty_now*1000,v_tz_now*1000;

    //得到状态转移矩阵
    Eigen::MatrixXd F_in(6,6);
    F_in<<1.0, 0.0, 0.0, t/1000, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, t/1000, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, t/1000,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0,0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

//预测上一最佳状态值
    KF_forecast.Prediction(F_in);

//更新状态量
    KF_forecast.update(z,F_in);

//    //传参
//    p_tx_old = BestArmor.tx;
//    p_ty_old = BestArmor.ty;
//    p_tz_old = BestArmor.tz;

//    old_objectP.x = p_tx_old;
//    old_objectP.y = p_ty_old;
//    old_objectP.z = p_tz_old;

//    v_tx_old = v_tx_now;
//    v_ty_old = v_ty_now;
//    v_tz_old = v_tz_now;


    //传参
    p_tx_old = KF_forecast.x_(0);
    p_ty_old = KF_forecast.x_(1);
    p_tz_old = KF_forecast.x_(2);

    old_carDatas = carDatas;
}


















//以上为修改

double AngleSolver::getPitRef()
{
    return pit_ref;
}

double AngleSolver::getYawRef()
{
    return yaw_ref;
}

void AngleSolver::setRelationPoseCameraPTZ(const double ptz_camera_x, const double ptz_camera_y,
                                           const double ptz_camera_z, double y_offset_barrel_ptz)
{

    double r_data[] = {1, 0, 0,
                       0, 1, 0,
                       0, 0, 1};

    cv::Mat rot_camera_ptz(3, 3, CV_64FC1, r_data);
    rot_camera_ptz.copyTo(rot_camera2ptz);

    // 云台相对于相机的偏移向量和平移向量
    double t_data[] = {-ptz_camera_x, ptz_camera_y, -ptz_camera_z};
    cv::Mat trans_camera_ptz(3, 1, CV_64FC1, t_data);
    trans_camera_ptz.copyTo(trans_camera2ptz);

    offset_y_barrel_ptz = y_offset_barrel_ptz;
}



void AngleSolver::getTarget2dPoinstion(const cv::RotatedRect &rect,
    std::vector<cv::Point2f> &target2d,
    const cv::Point2f &offset)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f lu, ld, ru, rd;
    std::sort(vertices, vertices + 4, [](const cv::Point2f & p1, const cv::Point2f & p2) { return p1.x < p2.x; });
    if (vertices[0].y < vertices[1].y)
    {
        lu = vertices[0];
        ld = vertices[1];
    }
    else
    {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y)
    {
        ru = vertices[2];
        rd = vertices[3];
    }
    else
    {
        ru = vertices[3];
        rd = vertices[2];
    }

    target2d.clear();
    target2d.push_back(lu + offset);
    target2d.push_back(ru + offset);
    target2d.push_back(rd + offset);
    target2d.push_back(ld + offset);
}

void AngleSolver::rectPnpSolver(const char *camera_param_file_name)
{
    cv::FileStorage fs(camera_param_file_name, cv::FileStorage::READ);
    if(!fs.isOpened())
        std::cout << "Could not open the configuration file" << std::endl;

    fs["Camera_Matrix"] >> cam_matrix;
    fs["Distortion_Coefficients"] >> distortion_coeff;
    fs["board_Width"] >> width_target;
    fs["board_Height"] >> height_target;

    if(cam_matrix.empty() || distortion_coeff.empty())
    {
        std::cout << "cam_matrix or distortion_coeff is empty!!!" << std::endl;
        return;
    }

    //根据目标矩形的宽高设置三维坐标
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;

    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
    point3d.push_back(cv::Point3f(half_x,-half_y, 0));
}

void AngleSolver::setTargetSize(double width, double height)
{
    width_target = width;
    height_target = height;

    //根据目标矩形的宽高设置三维坐标
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;

    point3d.clear();
    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
}

bool AngleSolver::getAngle(std::vector<cv::Point2f> target2d,
                           double bullet_speed, double current_ptz_angle)
{
    //根据检测出的目标在图像中的二维坐标，算出旋转矩阵与位移向量
    solvePnP4Points(target2d, this->rot, position_in_camera);

    position_in_camera.at<double>(0, 0) = scale_z * position_in_camera.at<double>(0, 0);
    position_in_camera.at<double>(1, 0) = scale_z * position_in_camera.at<double>(1, 0);
    position_in_camera.at<double>(2, 0) = scale_z * position_in_camera.at<double>(2, 0);

    // 相机坐标转换到PTZ坐标
    tranformationCamera2PTZ(position_in_camera, position_in_ptz);
    // 根据目标在PTZ坐标中的位置，计算偏移角度，使枪管瞄准目标
    adjustPTZ2Barrel(position_in_ptz, yaw_ref,pit_ref, bullet_speed, current_ptz_angle);
    return true;
}

void AngleSolver::tranformationCamera2PTZ(const cv::Mat &pos, cv::Mat &transed_pos)
{
    transed_pos = rot_camera2ptz * pos - trans_camera2ptz;
}

void AngleSolver::adjustPTZ2Barrel(const cv::Mat &pos_in_ptz,
    double &angle_x, double &angle_y,
    double bullet_speed,
    double current_ptz_angle)
{
    const double *_xyz = (const double *)pos_in_ptz.data;
    angle_x = atan2(_xyz[0], _xyz[2]);

    double xyz[3] = { _xyz[0], _xyz[1], _xyz[2] };

    double alpha = 0.0, theta = 0.0;

    alpha = asin(offset_y_barrel_ptz / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));

    if (xyz[1] < 0)
    {
        theta = atan(-xyz[1] / xyz[2]);
        angle_y = - alpha - theta;
    }
    else
    {
        theta = atan(xyz[1] / xyz[2]);
        angle_y = -alpha + theta;
    }
    angle_x = angle_x * 180.0 / PI;
    angle_y = angle_y * 180.0 / PI;
}

void AngleSolver::solvePnP4Points(const std::vector<cv::Point2f> &points2d,
                                  cv::Mat &rot, cv::Mat &trans)
{
    cv::Mat rvec;
    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, rvec, trans/*, true, CV_P3P*/); //自瞄
    cv::Rodrigues(rvec, rot);
}
/*
void AngleSolver::getDistanceDanmu(std::vector<cv::Point2f>armor_rect, double &dist)
{
    cv::RotatedRect armor = cv::minAreaRect(armor_rect);
    float p_w = std::max(armor.size.width, armor.size.height);
    float p_h = std::min(armor.size.width, armor.size.height);

    float fx_w = cam_matrix.at<double>(0, 0) * width_target;
    float fx_h = cam_matrix.at<double>(1, 1) * height_target;

    float dist_w = fx_w / p_w;
    float dist_h = fx_h / p_h;

    dist = (dist_w + dist_h) / 2;
}
*/

/*2023赛季新单目测距*/
/*
    解决的问题：
    1.4m内测距误差5cm内
    2.5m内测距误差10cm内
    尚未解决的问题：
    1.左右倾斜不准
    2.超过6m测距不准
    3.装甲板偏移屏幕中心点不准
    4.像素点抖动导致测距不准
*/
/*pnp测距*/
void AngleSolver::getDistancePnP(const std::vector<cv::Point2f> &points2d,
                                    double &dist){
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
//    std::cout << "2d点 = " << std::endl << points2d <<std::endl;

    cv::solvePnP(point3d, points2d, cam_matrix, distortion_coeff, rvec, position_in_camera, cv::SOLVEPNP_ITERATIVE);
    double z_pos =  position_in_camera.at<double>(2, 0);
//    dist = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);
    dist = z_pos * 1.0;

    std::cout << "当前使用pnp测距" << std::endl;
    //std::cout << "旋转向量 = " << std::endl << rvec << std::endl;
//    std::cout << "x_pos = " << x_pos << std::endl;
//    std::cout << "y_pos = " << y_pos << std::endl;
//    std::cout << "z_pos = " << z_pos << std::endl;
    //std::cout << "dist = " << dist << std::endl;

//    double angle_x = std::atan(x_pos/z_pos);
//    angle_x = angle_x * 180.0 / PI;
//    std::cout << "pnp: angle_x = " << angle_x << std::endl;
}

/*小孔成像测距new*/
void AngleSolver::getDistanceDanmu(std::vector<cv::Point2f>armor_rect, double &dist)
{
    cv::RotatedRect armor = cv::minAreaRect(armor_rect);
//    std::cout << "armor_rect = " << std::endl << armor_rect <<std::endl;

    float p_w = std::max(armor.size.width, armor.size.height);
    float p_h = std::min(armor.size.width, armor.size.height);

    float fx_w = cam_matrix.at<double>(0, 0) * width_target; //* SHOW_WIDTH / 750;    //750是因为标定时大小为750（600同理）
    float fx_h = cam_matrix.at<double>(1, 1) * height_target; //* SHOW_HEIGHT / 600;

    float dist_w = fx_w / p_w;
    float dist_h = fx_h / p_h;

    //装甲板倾斜的时候dist_w偏大,此处为30度时dist_w大于dist_h的距离
    if(dist_w - dist_h > 15  )
    {
        dist = dist_h;
    }
    else
    {
        //正对装甲板时，削弱高比例（高波动大，不准）
        dist = (4*dist_w + dist_h) / 5;
    }


//    std::cout << "当前使用小孔成像测距"<<std::endl;
//    std::cout << "灯条像素宽 = " << p_w << std::endl;
//    std::cout << "灯条像素高 = " << p_h << std::endl;
//    std::cout << "fx_w = " << fx_w << std::endl;
//    std::cout << "fx_h = " << fx_h << std::endl;
//    std::cout << "dist_w = " << dist_w << std::endl;
//    std::cout << "dist_h = " << dist_h << std::endl;
//    std::cout << "dist = " << dist << std::endl;

}


