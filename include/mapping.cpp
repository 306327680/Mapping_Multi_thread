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


        //滤波
     /*   VLPPointCloud::Ptr xyzirVLP_ptr(new VLPPointCloud);
        VLPPointCloud::Ptr xyzirVLP_ds_ptr(new VLPPointCloud);
        pcl::copyPointCloud(xyzirVLP,*xyzirVLP_ptr);
        //1. 執行濾波 setStddevMulThresh
        pcl::StatisticalOutlierRemoval<VLPPoint> sor;
        sor.setInputCloud (xyzirVLP_ptr);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1);
        sor.filter (*xyzirVLP_ds_ptr);
        pcl::copyPointCloud(*xyzirVLP_ds_ptr, xyzirVLP);
        xyzItimeRing.clear();*/
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
            if(distance>2.2&&distance<45){
                xyzItimeRing.push_back(temp);
            }
        }

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
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bef(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_local_map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_to_pub(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> tfed;
    std::vector<Eigen::Matrix4f> icp_result;
    std::vector<Eigen::Matrix4f>  poses_distortion;
    std::vector<Eigen::Matrix4f> poses;
    std::vector<mypcdCloud> clouds_distortion_origin;
    std::vector<pcl::PointCloud<pcl::PointXYZI>>  clouds;
    CSVio csvio;
    bool local_map_updated = true;
    ros::Time cur_time;
    mypcdCloud xyzItimeRing;

    for (int i = 0; i < _file_names.size(); ++i) {
        xyzItimeRing.clear();
        cloud_hesai->clear();
        std::cout <<"thread: "<<_thread_num<<" index:"<<i<<"  "<<_file_names[i] <<std::endl;
        readPointCloud(_file_names[i],cloud_hesai,cur_time,xyzItimeRing,"VLP");


        if(_first_cloud){                  //1.1 第一帧 不进行计算
            pcl::copyPointCloud(*cloud_hesai,*cloud_local_map);
            poses.push_back(Eigen::Matrix4f::Identity());
            clouds.push_back(*cloud_local_map);
            g2osaver.insertPose(Eigen::Isometry3d::Identity());
            _first_cloud = false;
        } else{
            auto _now_ms = std::chrono::high_resolution_clock ::now();

        icp.transformation = Eigen::Matrix4f::Identity();
        simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef); //T_l-1_l
            icp.transformation = Eigen::Matrix4f::Identity();
            icp.SetNormalICP(); //设定odom icp参数0.5+acc*0.01
            tfed = icp.normalIcpRegistration(cloud_bef,*cloud_local_map);
            icp_result.push_back(icp.increase);//T_l_l+1
            //2.3.1 再次去畸变 再次减小范围
            simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
            *cloud_local_map = lidarLocalMapDistance(poses,clouds,0.1,20 ,local_map_updated,*cloud_local_map);
            icp.SetPlaneICP();	//设定点面 mapping icp参数0.3+acc*0.01
            tfed = icp.normalIcpRegistrationlocal(cloud_bef,*cloud_local_map);
            icp_result.back() = icp_result.back()*icp.pcl_plane_plane_icp->getFinalTransformation(); //第一次结果*下一次去畸变icp结果
            //2.3.3 再次去畸变 再次减小范围
            simpleDistortion(xyzItimeRing,icp.increase.inverse(),*cloud_bef);
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>
                    (std::chrono::high_resolution_clock::now() - _now_ms);
            std::cout<< "icp time is :"<< duration.count()/1e9<<std::endl;
            *local_map_to_pub = *cloud_local_map;
            *cloud_local_map = *cloud_bef; 	//下一帧匹配的target是上帧去畸变之后的结果
            //可以用恢复出来的位姿 tf 以前的点云
//				clouds.push_back(bef_tfed_scan);
            clouds.push_back(*cloud_bef);
            Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity();
            //试一下这样恢复出来的位姿
            for (int k = 0; k < icp_result.size(); ++k) {
                current_pose *= icp_result[k];
            }
            poses_distortion.push_back(current_pose.matrix());
            poses.push_back(current_pose.matrix());
            g2osaver.insertPose(Eigen::Isometry3d(current_pose.matrix().cast<double>()));
            clouds_distortion_origin.push_back(xyzItimeRing);

            //存一下'csvio
            Eigen::Isometry3d se3_save;
            csvio.LiDARsaveOnePose(Eigen::Isometry3d(current_pose.matrix().cast<double>()),cur_time);//转csv用的
            std::cout<<"全局坐标 \n"<<current_pose.matrix()<<std::endl;
        }

    }
    std::string save_g2o_path = "/home/echo/pcd/";
    save_g2o_path += _thread_num;
    save_g2o_path += "_LiDAR_Odom.g2o";
    std::cout<<save_g2o_path<<std::endl;
    g2osaver.saveGraph(save_g2o_path);
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
pcl::PointCloud<pcl::PointXYZI> mapping::lidarLocalMapDistance(std::vector<Eigen::Matrix4f> &poses,
                                                                     std::vector<pcl::PointCloud<pcl::PointXYZI>> &clouds,
                                                                     double distiance, int buffer_size,
                                                                     bool &local_map_updated,
                                                                     pcl::PointCloud<pcl::PointXYZI> last_local_map) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> map_temp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_temp_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<Eigen::Matrix4f> poses_tmp;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> clouds_tmp;
    Eigen::Matrix4f tf_all = Eigen::Matrix4f::Identity();
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor1;
    pcl::UniformSampling<pcl::PointXYZI> filter_us;
    pcl::PointCloud<int> keypointIndices;

    //存放两帧的位姿
    Eigen::Matrix4f pose_latest;
    Eigen::Matrix4f pose_last;
    Eigen::Matrix4f pose_diff;
    float dx,dy,dz;
    double distance_calc;
    //室外的參數 //4.整体地图降采样
/*	double filter_param_x = 0.35;
	double filter_param_y = 0.35;
	double filter_param_z = 0.05;

	double final_filter_param_x = 0.25;
	double final_filter_param_y = 0.25;
	double final_filter_param_z = 0.05;*/
    //室內的參數 mod 2021/11/4 //4.整体地图降采样
    double filter_param_x = 0.15;
    double filter_param_y = 0.15;
    double filter_param_z = 0.05;

    //0. 得到这一阵位姿和上一阵位姿
    if(poses.size()> 1){
        pose_latest =  poses.back();
        pose_last = poses.at(poses.size()-2);
        dx = pose_latest(0,3) - pose_last(0,3);
        dy = pose_latest(1,3) - pose_last(1,3);
        dz = pose_latest(2,3) - pose_last(2,3);
        distance_calc = sqrtf(dx*dx + dy*dy + dz*dz);
    }else{
        distance_calc = 100;
    }

    //满足运动距离
    if(distance_calc>distiance){
        local_map_updated = true;
        //1. 最新帧降采样
        *map_temp_ptr = clouds.back();
        sor.setInputCloud(map_temp_ptr);
        sor.setLeafSize(0.05f, 0.05f, 0.05f);//室内
//		sor.setLeafSize(0.1f, 0.1f, 0.1f); //外面
        sor.filter(clouds.back());
/*		*map_temp_ptr = clouds.back();
		sor1.setInputCloud(map_temp_ptr);
		sor1.setMeanK(50);
		sor1.setStddevMulThresh(1.0);
		sor1.filter(clouds.back());*/

        //2. note 15帧64线 0.02 大概225520个点 //10帧试试
        //点云数量超过了设置的阈值
        if (poses.size() >= buffer_size) {
            //后期点云比较多的情况
            for (int i = poses.size() - buffer_size; i < poses.size(); i++) {
                pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
                //下面的if 保证了 clouds 里面都是 降采样过的点云
                *map_ptr += map_temp;
                poses_tmp.push_back(poses[i]);
                clouds_tmp.push_back(clouds[i]);
            }
            poses = poses_tmp;
            clouds = clouds_tmp;
            //3. 地图转换到当前的最后一帧的位姿
            pcl::transformPointCloud(*map_ptr, map_temp, poses.back().inverse());
            *map_ptr = map_temp;
            //4.整体地图降采样
            sor.setInputCloud(map_ptr);
            sor.setLeafSize(filter_param_x, filter_param_y, filter_param_z);
            sor.filter(map_temp);
/*			filter_us.setInputCloud(map_ptr);
			filter_us.setRadiusSearch(0.1f);
			filter_us.compute(keypointIndices);
			pcl::copyPointCloud(*map_ptr, keypointIndices.points, map_temp);*/
            return map_temp;

        } else if (poses.size() > 1) {//第一开始点云不多的情况 不要第一个帧,因为没有去畸变
            for (int i = 1; i < poses.size(); i++) {
                pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
                *map_ptr += map_temp;
            }
        } else {
            pcl::transformPointCloud(clouds[0], *map_ptr, poses[0]);
        }
        //3. 地图转换到当前的最后一帧的位姿
        pcl::transformPointCloud(*map_ptr, map_temp, poses.back().inverse());
        *map_ptr = map_temp;
        //4.整体地图降采样
        sor.setInputCloud(map_ptr);
        sor.setLeafSize(filter_param_x, filter_param_y, filter_param_z);
        sor.filter(map_temp);
        return map_temp;
    }else{ 	//不满足运动距离
        local_map_updated = false;
        //2. 这个只保留前面的点云和位姿
        if (poses.size() >= buffer_size) {
            //点云足够多的情况
            for (int i = 0; i < buffer_size; i++) {
                pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
                //下面的if 保证了 clouds 里面都是 降采样过的点云
                *map_ptr += map_temp;
                poses_tmp.push_back(poses[i]);
                clouds_tmp.push_back(clouds[i]);
            }
            poses = poses_tmp;
            clouds = clouds_tmp;
            //3. 地图转换到最新的一帧下位位置,继续计算增量 last_local_map
            sor.setInputCloud(map_ptr);
            sor.setLeafSize(filter_param_x, filter_param_y, filter_param_z);
            sor.filter(map_temp);
/*			filter_us.setInputCloud(map_ptr);
			filter_us.setRadiusSearch(0.05f);
			filter_us.compute(keypointIndices);
			pcl::copyPointCloud(*map_ptr, keypointIndices.points, map_temp);*/
            pcl::transformPointCloud(map_temp, *map_ptr, pose_latest.inverse());
            return *map_ptr;

        } else if (poses.size() > 1) {//第一开始点云不多的情况 不要第一个帧,因为没有去畸变
            for (int i = 1; i < poses.size(); i++) {
                pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
                *map_ptr += map_temp;
            }
            //3. 地图转换到最新的一帧下位位置,继续计算增量 last_local_map
            pcl::transformPointCloud(*map_ptr, map_temp, pose_latest.inverse());
            *map_ptr = map_temp;
            //4.降采样
            sor.setInputCloud(map_ptr);
            sor.setLeafSize(filter_param_x, filter_param_y, filter_param_z);
            sor.filter(map_temp);
            return map_temp;
        } else {
            pcl::transformPointCloud(clouds[0], *map_ptr, poses[0]);
            //3. 地图转换到最新的一帧下位位置,继续计算增量 last_local_map
            pcl::transformPointCloud(*map_ptr, map_temp, pose_latest.inverse());
            *map_ptr = map_temp;
            //4.降采样
            sor.setInputCloud(map_ptr);
            sor.setLeafSize(filter_param_x, filter_param_y, filter_param_z);
            sor.filter(map_temp);
            return map_temp;
        }
    }
}