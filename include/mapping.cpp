//
// Created by echo on 23-4-19.
//

#include "mapping.h"

void
mapping::readPointCloud(std::string file_names, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai, ros::Time &cur_time,
                        mypcdCloud &xyzItimeRing, std::string LiDAR_type="VLP") {
    VLPPointCloud xyzirVLP;
    RoboPointCLoud xyzirRobo;
    if(LiDAR_type == "VLP"){//判断用的是不是vlp的,用的话进行转换

        pcl::io::loadPCDFile<VLPPoint>(file_names, xyzirVLP);
        auto _now_ms = std::chrono::high_resolution_clock ::now();

        //滤波
        VLPPointCloud::Ptr xyzirVLP_ptr(new VLPPointCloud);
        VLPPointCloud::Ptr xyzirVLP_ds_ptr(new VLPPointCloud);
        pcl::copyPointCloud(xyzirVLP,*xyzirVLP_ptr);
        //1. 執行濾波 setStddevMulThresh
        pcl::StatisticalOutlierRemoval<VLPPoint> sor;
        sor.setInputCloud (xyzirVLP_ptr);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1);
        sor.filter (*xyzirVLP_ds_ptr);
        pcl::copyPointCloud(*xyzirVLP_ds_ptr, xyzirVLP);
        xyzItimeRing.clear();
        //设置时间戳
        cur_time.fromSec(xyzirVLP[xyzirVLP.size()-1].time);

        for (int j = 0; j < xyzirVLP.size(); ++j) {
            mypcd temp;
            temp.x = xyzirVLP[j].x;
            temp.y = xyzirVLP[j].y;
            temp.z = xyzirVLP[j].z;
            temp.intensity = xyzirVLP[j].intensity;
            temp.timestamp = xyzirVLP[j].time;
            temp.ring = xyzirVLP[j].ring;
            //设置距离滤波
            double distance = sqrtf(temp.x*temp.x+temp.y*temp.y);
            if(distance>2.2){
                xyzItimeRing.push_back(temp);
            }
        }
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>
                (std::chrono::high_resolution_clock::now() - _now_ms);
        std::cout<< "filter time is :"<< duration.count()/1e9<<std::endl;
    }  else if(LiDAR_type == "Hesai"){
        pcl::io::loadPCDFile<mypcd>(file_names, xyzItimeRing);
    } else if(LiDAR_type == "robo"){
        pcl::io::loadPCDFile<RoboPoint>(file_names, xyzirRobo);
        xyzItimeRing.clear();
        for (int j = 0; j < xyzirRobo.size(); ++j) {
            mypcd temp;
            temp.x = xyzirRobo[j].x;
            temp.y = xyzirRobo[j].y;
            temp.z = xyzirRobo[j].z;
            temp.intensity = xyzirRobo[j].intensity;
            temp.timestamp = xyzirRobo[j].time;
            temp.ring = xyzirRobo[j].ring;
            xyzItimeRing.push_back(temp);
        }
    }else{
        std::cout<<"unknown pcd type"<<std::endl;
    }
    pcl::copyPointCloud(xyzItimeRing,*cloud_hesai);
}

void mapping::start_mapping() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_local_map(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>>  clouds;

    ros::Time cur_time;
    mypcdCloud xyzItimeRing;

    for (int i = 0; i < _file_names.size(); ++i) {
        xyzItimeRing.clear();
        cloud_hesai->clear();
        readPointCloud(_file_names[i],cloud_hesai,cur_time,xyzItimeRing,"VLP");

        std::cout <<"thread: "<<_thread_num<<" index:"<<i<<"  "<<_file_names[i] <<std::endl;
        if(_first_cloud){                  //1.1 第一帧 不进行计算
            pcl::copyPointCloud(*cloud_hesai,*cloud_local_map);
            poses.push_back(Eigen::Matrix4f::Identity());
            clouds.push_back(*cloud_local_map);
            g2osaver.insertPose(Eigen::Isometry3d::Identity());
            _first_cloud = false;
        } else{
        icp.transformation = Eigen::Matrix4f::Identity();

        }
    }

}
void mapping::simpleDistortion(mypcdCloud input, Eigen::Matrix4f increase, pcl::PointCloud<pcl::PointXYZI> &output){

    output.clear();
    pcl::PointCloud<PointTypeBeam>::Ptr test(new pcl::PointCloud<PointTypeBeam>);
    pcl::PointCloud<PointTypeBeam>::Ptr pcout(new pcl::PointCloud<PointTypeBeam>);

    Eigen::Isometry3d se3;
    se3 = increase.cast<double>();
    PointTypeBeam temp;
    for (int i = 0; i < input.size(); ++i) {
    temp.x = input[i].x;
    temp.y = input[i].y;
    temp.z = input[i].z;
    temp.intensity = input[i].intensity;
    temp.pctime = (input[i].timestamp - input[input.size()-1].timestamp) * 10;
    temp.beam = input[i].ring;
    test->push_back(temp);
    }
    adjustDistortion(test, pcout, se3);
    pcl::copyPointCloud(*pcout, output);
}

void mapping::adjustDistortion(pcl::PointCloud<PointTypeBeam>::Ptr pointIn,
                                         pcl::PointCloud<PointTypeBeam>::Ptr &pointOut,
                                         Eigen::Isometry3d transform) {
    pointOut->clear();
    std::vector<pcl::PointCloud<PointTypeBeam> > laserCloudScans_N(N_SCAN);
    Eigen::Matrix4d transInput,out,input;
    Eigen::Isometry3d transpc;
    input = Eigen::Matrix4d::Identity();//起始为0
    transInput.matrix() = transform.matrix();//终止为1
    pcl::PointCloud<PointTypeBeam>::Ptr temp(new pcl::PointCloud<PointTypeBeam>);
    pcl::PointCloud<PointTypeBeam>::Ptr Individual(new pcl::PointCloud<PointTypeBeam>);
    pcl::PointCloud<PointTypeBeam>::Ptr Individual_bef(new pcl::PointCloud<PointTypeBeam>);
    temp->resize(pointIn->size());
    for(int i = 0; i < pointIn->size(); i++){
        Individual->resize(1);
        Individual_bef->resize(1);
        Individual_bef->points[0] = pointIn->points[i];
        Individual->points[0] = pointIn->points[i];
        trinterp(input, transInput, pointIn->points[i].pctime, out);
        transpc.matrix() = out.matrix();
        Eigen::Matrix4d convert;
        convert = transpc.matrix();
        convert = transpc.matrix().inverse();
        pcl::transformPointCloud(*Individual_bef, *Individual, convert);
        temp->points[i] = pointIn->points[i];
        temp->points[i].x = Individual->points[0].x;
        temp->points[i].y = Individual->points[0].y;
        temp->points[i].z = Individual->points[0].z;
        if(pointIn->points[i].beam >= 0 && pointIn->points[i].beam <= N_SCAN){
            laserCloudScans_N[pointIn->points[i].beam].push_back(temp->points[i]);
        }
    }
    for (int i = 0; i < N_SCAN; i++) {
        *pointOut += laserCloudScans_N[i];
    }
}

void mapping::trinterp(Eigen::Matrix4d &T0, Eigen::Matrix4d &T1, double r, Eigen::Matrix4d &T) {
    Eigen::Vector4d q0, q1, qr;
    Eigen::Vector3d p0, p1, pr;
    Eigen::Matrix3d R, R0;
    //R 为插值之后的旋转矩阵

    rotMat2quaternion(T0, q0);  // 位姿矩阵转换为四元数
    rotMat2quaternion(T1, q1);

    p0 << T0(0, 3), T0(1, 3), T0(2, 3);       // 提取出位置矩阵
    p1 << T1(0, 3), T1(1, 3), T1(2, 3);

    qinterp(q0, q1, r, qr);      // 进行四元数插值 10.4
    pr = p0*(1 - r) + r*p1;      // 进行位置插值

    quatern2rotMat(qr, R);       // 四元数转旋转矩阵

    T(0, 0) = R(0, 0); T(0, 1) = R(0, 1); T(0, 2) = R(0, 2); T(0, 3) = pr(0);
    T(1, 0) = R(1, 0); T(1, 1) = R(1, 1); T(1, 2) = R(1, 2); T(1, 3) = pr(1);
    T(2, 0) = R(2, 0); T(2, 1) = R(2, 1); T(2, 2) = R(2, 2); T(2, 3) = pr(2);
    T(3, 0) = 0;       T(3, 1) = 0;       T(3, 2) = 0;       T(3, 3) = 1;
}
void mapping::rotMat2quaternion(Eigen::Matrix4d &T, Eigen::Vector4d &q_quaternion) {

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    double w, x, y, z;

    R(0, 0) = T(0, 0);
    R(0, 1) = T(0, 1);
    R(0, 2) = T(0, 2);
    R(1, 0) = T(1, 0);
    R(1, 1) = T(1, 1);
    R(1, 2) = T(1, 2);
    R(2, 0) = T(2, 0);
    R(2, 1) = T(2, 1);
    R(2, 2) = T(2, 2);

    double trace = R(0, 0) + R(1, 1) + R(2, 2);
    double epsilon = 1E-12;
    if (trace > epsilon)
    {
        double s = 0.5 / sqrt(trace + 1.0);
        w = 0.25 / s;
        x = -(R(2, 1) - R(1, 2)) * s;
        y = -(R(0, 2) - R(2, 0)) * s;
        z = -(R(1, 0) - R(0, 1)) * s;

    }
    else
    {
        if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2))
        {
            double s = 2.0 * sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
            w = -(R(2, 1) - R(1, 2)) / s;
            x = 0.25 * s;
            y = (R(0, 1) + R(1, 0)) / s;
            z = (R(0, 2) + R(2, 0)) / s;

        }
        else if (R(1, 1) > R(2, 2))
        {
            double s = 2.0 * sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
            w = -(R(0, 2) - R(2, 0)) / s;
            x = (R(0, 1) + R(1, 0)) / s;
            y = 0.25 * s;
            z = (R(1, 2) + R(2, 1)) / s;
        }
        else
        {
            double s = 2.0 * sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
            w = -(R(1, 0) - R(0, 1)) / s;
            x = (R(0, 2) + R(2, 0)) / s;
            y = (R(1, 2) + R(2, 1)) / s;
            z = 0.25 * s;

        }

    }
    q_quaternion(0) = w;
    q_quaternion(1) = x;
    q_quaternion(2) = y;
    q_quaternion(3) = z;
}

void mapping::qinterp(Eigen::Vector4d &Q1, Eigen::Vector4d &Q2, double r,
                                Eigen::Vector4d &q_quaternion_interpolation) {

    double k0, k1, theta, sin_theta, cos_theta;
    Eigen::Vector4d q_temp;
    if ((r<0) || (r>1)){
//		std::cout << " R out of rang : " <<r<< std::endl;
    }


    cos_theta = Q1(0)*Q2(0) + Q1(1)*Q2(1) + Q1(2)*Q2(2) + Q1(3)*Q2(3);

    if (cos_theta < 0) {
        Q2 = -Q2;
        cos_theta = -cos_theta;
    }

    if ((cos_theta) > 0.9999999999) {
        k0 = 1.00 - r;
        k1 = r;

    }
    else {
        sin_theta = sqrt(1.00 - pow(cos_theta, 2));
        theta = atan2(sin_theta, cos_theta);
        k0 = sin((1.000 - r)*theta) / sin(theta);
        k1 = sin((r)*theta) / sin(theta);
    }

    q_quaternion_interpolation = k0* Q1 + k1 * Q2;
    Eigen::Quaterniond out;
    out.x() = q_quaternion_interpolation(0);
    out.y() = q_quaternion_interpolation(1);
    out.z() = q_quaternion_interpolation(2);
    out.w() = q_quaternion_interpolation(3);
    out.normalize();
    q_quaternion_interpolation(0) = out.x();
    q_quaternion_interpolation(1) = out.y();
    q_quaternion_interpolation(2) = out.z();
    q_quaternion_interpolation(3) = out.w();
}
void mapping::quatern2rotMat(Eigen::Vector4d &q_quaternion, Eigen::Matrix3d &R) {
    Eigen::Quaterniond q;
    q.x() = q_quaternion(0);
    q.y() = q_quaternion(1);
    q.z() = q_quaternion(2);
    q.w() = q_quaternion(3);
//	R=Eigen::Matrix3d(q);
    R(0, 0) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(1), 2);
    R(0, 1) = 2.0000 * (q_quaternion(1)*q_quaternion(2) + q_quaternion(0)*q_quaternion(3));
    R(0, 2) = 2.0000 * (q_quaternion(1)*q_quaternion(3) - q_quaternion(0)*q_quaternion(2));
    R(1, 0) = 2.0000* (q_quaternion(1)*q_quaternion(2) - q_quaternion(0)*q_quaternion(3));
    R(1, 1) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(2), 2);
    R(1, 2) = 2.0000* (q_quaternion(2)*q_quaternion(3) + q_quaternion(0)*q_quaternion(1));
    R(2, 0) = 2.0000 * (q_quaternion(1)*q_quaternion(3) + q_quaternion(0)*q_quaternion(2));
    R(2, 1) = 2.0000 * (q_quaternion(2)*q_quaternion(3) - q_quaternion(0)*q_quaternion(1));
    R(2, 2) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(3), 2);

}
