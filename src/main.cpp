#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
using namespace std;
struct foot
{
    double x, y, z, roll, pitch, yaw;
    bool is_left = true;
    foot(double x_, double y_, double z_, double roll_, double pitch_, double yaw_, bool is_left_)
    {
        x = x_;
        y = y_;
        z = z_;
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;
        is_left = is_left_;
    }
};

vector<Eigen::Vector3d> stepCorners(foot f)
{
    Eigen::Vector3d c1(0.15, 0.065, 0);
    Eigen::Vector3d c2(0.15, -0.065, 0);
    Eigen::Vector3d c3(-0.11, -0.065, 0);
    Eigen::Vector3d c4(-0.11, 0.065, 0);
    Eigen::AngleAxisd ad_roll(f.roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ad_pitch(f.pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd ad_yaw(f.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d r = ad_roll.toRotationMatrix() * ad_pitch.toRotationMatrix() * ad_yaw.toRotationMatrix();
    vector<Eigen::Vector3d> return_data;
    Eigen::Vector3d center(f.x, f.y, f.z);
    return_data.emplace_back(r*c1 + center);
    return_data.emplace_back(r*c2 + center);
    return_data.emplace_back(r*c3 + center);
    return_data.emplace_back(r*c4 + center);
    return return_data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "draw_steps");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<grid_map_msgs::GridMap>("map", 1);
    ros::Publisher pub_steps = nh.advertise<visualization_msgs::MarkerArray>("steps", 1);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    tf2_ros::TransformBroadcaster broadcaster;
    std::string bag_file_path = "/home/lichao/Darwin-op/src/draw_steps/data/2024-01-28-19-28-48.bag";// 上楼梯
    std::string topic_name = "/elevation_mapping_ours_navLocalmap/local_map";

    std::string globalmap_path = "/home/lichao/Darwin-op/src/draw_steps/data/global_map.bag";
    std::string globalmap_topic = "/global_map";

    rosbag::Bag bag;
    bag.open(bag_file_path, rosbag::bagmode::Read);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    // pcl::io::loadPCDFile<pcl::PointXYZ>("/home/lichao/Darwin-op/src/draw_steps/data/scans.pcd", cloud);
    string pcd_file_path = "/home/lichao/Darwin-op/src/draw_steps/data/scans.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", pcd_file_path.c_str());
        return -1;
    }
    
    rosbag::View view(bag, rosbag::TopicQuery(topic_name));
    vector<grid_map::GridMap> maps;
    BOOST_FOREACH(rosbag::MessageInstance const& msg, view) {
        // 替换成你实际使用的消息类型
        grid_map_msgs::GridMap::ConstPtr map_mag = msg.instantiate<grid_map_msgs::GridMap>();
        if (map_mag != nullptr) {
            // 处理消息，这里仅打印消息内容
            // std::cout << "Timestamp: " << msg.getTime() << ", Data: " << string_msg->data << std::endl;
            grid_map::GridMap local_map;
            grid_map::GridMapRosConverter::fromMessage(*map_mag, local_map);
            maps.emplace_back(local_map);
        }
    }
    cout<<"map size is "<<maps.size()<<endl;
    sleep(1);
    vector<grid_map::GridMap> chooseMaps;
    chooseMaps.emplace_back(maps.at(5));
    chooseMaps.emplace_back(maps.at(5));
    chooseMaps.emplace_back(maps.at(14+1));
    chooseMaps.emplace_back(maps.at(16+1));
    chooseMaps.emplace_back(maps.at(20+1));

    // chooseMaps.emplace_back(maps.at(24));
    // chooseMaps.emplace_back(maps.at(28));
    // chooseMaps.emplace_back(maps.at(32));
    // chooseMaps.emplace_back(maps.at(36));

    chooseMaps.emplace_back(maps.at(40+1));
    chooseMaps.emplace_back(maps.at(44+1));
    chooseMaps.emplace_back(maps.at(48+1));
    chooseMaps.emplace_back(maps.at(52+1));
    chooseMaps.emplace_back(maps.at(56+1));
    chooseMaps.emplace_back(maps.at(60+1));
    chooseMaps.emplace_back(maps.at(64+1));
    chooseMaps.emplace_back(maps.at(68+1));
    chooseMaps.emplace_back(maps.at(72+1));
    chooseMaps.emplace_back(maps.at(76+1));
    chooseMaps.emplace_back(maps.at(80+1));
    chooseMaps.emplace_back(maps.at(84+1));

    

    // 初始化
    vector<vector<foot>> map_foots;
    vector<foot> foots1;
    foots1.emplace_back(foot(0.11,   0.1, 0,  0, 0,       0, true ));
    foots1.emplace_back(foot(0.11,  -0.1, 0,  0, 0,       0, false));
    foots1.emplace_back(foot(0.38,  0.1, 0.03, 0, -14/57.3, 0, true ));
    foots1.emplace_back(foot(0.38, -0.1, 0.03, 0, -14/57.3, 0, false));
    foots1.emplace_back(foot(0.55,  0.1, 0.078, 0, -14/57.3, 0, true ));
    foots1.emplace_back(foot(0.55, -0.1, 0.078, 0, -14/57.3, 0, false));
    foots1.emplace_back(foot(0.77,  0.1, 0.13, 0, 0, 0, true ));
    foots1.emplace_back(foot(0.77, -0.1, 0.13, 0, 0, 0, false));
    foots1.emplace_back(foot(1.06,  0.1, 0.25, 0, 0, 0, true ));
    foots1.emplace_back(foot(1.06, -0.1, 0.25, 0, 0, 0, false));
    map_foots.emplace_back(foots1);
    // 迈出第一步，左脚
    foots1.clear();
    // foots1.emplace_back(foot(0.11,   0.1, 0,  0, 0,       0, true ));
    foots1.emplace_back(foot(0.11,  -0.1, 0,  0, 0,       0, false));
    foots1.emplace_back(foot(0.38,  0.1, 0.03, 0, -14/57.3, 0, true ));
    foots1.emplace_back(foot(0.38, -0.1, 0.03, 0, -14/57.3, 0, false));
    foots1.emplace_back(foot(0.55,  0.1, 0.078, 0, -14/57.3, 0, true ));
    foots1.emplace_back(foot(0.55, -0.1, 0.078, 0, -14/57.3, 0, false));
    foots1.emplace_back(foot(0.77,  0.1, 0.13, 0, 0, 0, true ));
    foots1.emplace_back(foot(0.77, -0.1, 0.13, 0, 0, 0, false));
    foots1.emplace_back(foot(1.06,  0.1, 0.25, 0, 0, 0, true ));
    foots1.emplace_back(foot(1.06, -0.1, 0.25, 0, 0, 0, false));
    map_foots.emplace_back(foots1);
    // 迈出第二步，右脚
    foots1.clear();
    foots1.emplace_back(foot(0.23,   0.1, 0.035,  0, -14/57.3,       0, true ));
    foots1.emplace_back(foot(0.23,  -0.1, 0.035,  0, -14/57.3,       0, false));
    foots1.emplace_back(foot(0.42,   0.1, 0.08,  0, -14/57.3,       0, true ));
    foots1.emplace_back(foot(0.42,  -0.1, 0.08,  0, -14/57.3,       0, false));
    foots1.emplace_back(foot(0.58,   0.1, 0.135,  0, 0,       0, true ));
    foots1.emplace_back(foot(0.58,  -0.1, 0.135,  0, 0,       0, false));
    foots1.emplace_back(foot(0.87,   0.1, 0.25,  0, 0,       0, true ));
    foots1.emplace_back(foot(0.87,  -0.1, 0.25,  0, 0,       0, false));
    map_foots.emplace_back(foots1);
    // 迈出第三步，左脚，上斜坡
    foots1.clear();
    foots1.emplace_back(foot(0.20,  -0.1, 0.035,  0, -14/57.3,       0, false));
    foots1.emplace_back(foot(0.35,   0.1, 0.07,  0, -14/57.3,       0, true ));
    foots1.emplace_back(foot(0.35,  -0.1, 0.07,  0, -14/57.3,       0, false));
    foots1.emplace_back(foot(0.56,   0.1, 0.135,  0, 0,       0, true ));
    foots1.emplace_back(foot(0.56,  -0.1, 0.135,  0, 0,       0, false));
    foots1.emplace_back(foot(0.86,   0.1, 0.25,  0, 0,       0, true ));
    foots1.emplace_back(foot(0.86,  -0.1, 0.25,  0, 0,       0, false));
    map_foots.emplace_back(foots1);
    // // // 迈出第四步，右脚，上斜坡
    foots1.clear();
    foots1.emplace_back(foot(0.15,   0.1, 0.08,  0, -14/57.3,       0, true ));
    foots1.emplace_back(foot(0.15,  -0.1, 0.08,  0, -14/57.3,       0, false));
    foots1.emplace_back(foot(0.37,   0.1, 0.13,  0, 0,       0, true ));
    foots1.emplace_back(foot(0.37,  -0.1, 0.13,  0, 0,       0, false));
    foots1.emplace_back(foot(0.65,   0.1, 0.25,  0, 0,       0, true ));
    foots1.emplace_back(foot(0.65,  -0.1, 0.25,  0, 0,       0, false));
    map_foots.emplace_back(foots1);

    // 迈出第五步，斜坡上平其一脚,规划右脚
    foots1.clear();
    foots1.emplace_back(foot(0.07,  -0.1, 0.08,  0, -14/57.3,       0, false));
    foots1.emplace_back(foot(0.25,   0.1, 0.135,  0, 0,       0, true ));
    foots1.emplace_back(foot(0.25,  -0.1, 0.135,  0, 0,       0, false));
    foots1.emplace_back(foot(0.54,   0.1, 0.235,  0, 0,       -1/57.3, true ));
    foots1.emplace_back(foot(0.54,  -0.1, 0.235,  0, 0,       -1/57.3, false));
    foots1.emplace_back(foot(0.83,   0.1, 0.365,  0, 0,       -1/57.3, true ));
    foots1.emplace_back(foot(0.83,  -0.1, 0.365,  0, 0,       -1/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.14,   0.1, 0.135,  0, 0,       0, true ));
    foots1.emplace_back(foot(0.14,  -0.1, 0.135,  0, 0,       0, false));
    foots1.emplace_back(foot(0.44,   0.1, 0.245,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.44,  -0.1, 0.245,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.74,   0.1, 0.365,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.74,  -0.1, 0.365,  0, 0,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.14,  -0.1, 0.135,  0, 0,       0, false));
    foots1.emplace_back(foot(0.44,   0.1, 0.255,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.44,  -0.1, 0.255,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.71,   0.1, 0.365,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.71,  -0.1, 0.365,  0, 0,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.23,   0.1, 0.255,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.23,  -0.1, 0.255,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.51,   0.1, 0.365,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.51,  -0.1, 0.365,  0, 0,       -0/57.3, false));
    // foots1.emplace_back(foot(0.9,   0.1, 0.475,  0, 0,       -0/57.3, true ));
    // foots1.emplace_back(foot(0.9,  -0.1, 0.475,  0, 0,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.22,  -0.1, 0.235,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.48,   0.1, 0.345,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.48,  -0.1, 0.345,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.78,   0.1, 0.455,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.78,  -0.1, 0.455,  0, 0,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.25,   0.1, 0.335,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.25,  -0.1, 0.335,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.545,   0.1, 0.455,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.545,  -0.1, 0.455,  0, 0,       -0/57.3, false));
    // foots1.emplace_back(foot(0.95,   0.1, 0.475,  0, 0,       -0/57.3, true ));
    // foots1.emplace_back(foot(0.95,  -0.1, 0.475,  0, 0,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.23,  -0.1, 0.335,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.5,   0.1, 0.455,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.5,  -0.1, 0.455,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.79,   0.1, 0.575,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.79,  -0.1, 0.575,  0, 0,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.26,   0.1, 0.435,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.265,  -0.1, 0.435,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.56,   0.1, 0.555,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.565,  -0.1, 0.555,  0, 0,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.225,  -0.1, 0.435,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.49,   0.1, 0.56,  0, 0,       -0/57.3, true ));
    foots1.emplace_back(foot(0.49,  -0.1, 0.56,  0, 0,       -0/57.3, false));
    foots1.emplace_back(foot(0.80,   0.1, 0.72,  0, -1/57.3,       -0/57.3, true ));
    foots1.emplace_back(foot(0.80,  -0.1, 0.72,  0, -1/57.3,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.29,   0.1, 0.56,  0, -2/57.3,       -0/57.3, true ));
    foots1.emplace_back(foot(0.29,  -0.1, 0.56,  0, -2/57.3,       -0/57.3, false));
    foots1.emplace_back(foot(0.6,   0.1, 0.72,  0, -3/57.3,       -0/57.3, true ));
    foots1.emplace_back(foot(0.6,  -0.1, 0.72,  0, -3/57.3,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.22,  -0.1, 0.58,  0, -4/57.3,       -0/57.3, false));
    foots1.emplace_back(foot(0.5,   0.1, 0.73,  0, -5/57.3,       -0/57.3, true ));
    foots1.emplace_back(foot(0.5,  -0.1, 0.73,  0, -5/57.3,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    // foots1.emplace_back(foot(0.22,  -0.1, 0.58,  0, -4/57.3,       -0/57.3, false));
    foots1.emplace_back(foot(0.3,   0.1, 0.73,  0, -5/57.3,       -0/57.3, true ));
    foots1.emplace_back(foot(0.3,  -0.1, 0.73,  0, -5/57.3,       -0/57.3, false));
    map_foots.emplace_back(foots1);

    int index = 0;
    ros::Rate loop_rate(0.625);
    double pianyi_x = 0;
    double step_pianyi = 0;
    while (ros::ok() && index < chooseMaps.size())
    {
        grid_map_msgs::GridMap pub_msg;
        grid_map::GridMapRosConverter::toMessage(chooseMaps.at(index), pub_msg);
        pub_msg.info.header.stamp = ros::Time::now();
        pub_msg.info.header.frame_id = "map";
        pub.publish(pub_msg);

        // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        // transform.translation() << -0.01-(map_foots.at(index).begin()->x), 0, (0.7217 + 0.69);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.frame_id = "world";  // 父坐标系
        transformStamped.child_frame_id = "map";  // 子坐标系
        transformStamped.header.stamp = ros::Time::now();
        if (map_foots.at(index).begin()->is_left == true)
        {
            step_pianyi = map_foots.at(index).begin()->x/3;
            pianyi_x += map_foots.at(index).begin()->x - step_pianyi;
        }
        else
        {
            pianyi_x += step_pianyi;
        }
        
        transformStamped.transform.translation.x = 0.01 + pianyi_x;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = -(0.7217 + 0.69);
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;
        broadcaster.sendTransform(transformStamped);
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(cloud_msg);
        // visualization_msgs::MarkerArray model_markers;
        // model_markers.markers.resize(0);
        // visualization_msgs::Marker model_marker;
        // model_marker.action = visualization_msgs::Marker::DELETEALL;
        // model_marker.header.frame_id = "map";  // 设置坐标系
        // model_marker.id = 0;
        // model_marker.type = visualization_msgs::Marker::MESH_RESOURCE; 
        // model_marker.color.r = 220.0/255.0;
        // model_marker.color.g = 223.0/255.0;
        // model_marker.color.b = 227.0/255.0;
        // pub_steps.publish(model_markers);
        // model_markers.markers.clear();
        visualization_msgs::MarkerArray new_model_markers;
        // model_markers.markers.at(0).action = visualization_msgs::Marker::DELETEALL;
        for (int i = 0; i < map_foots.at(index).size(); i++)
        {
            visualization_msgs::Marker model_marker;
            model_marker.action = visualization_msgs::Marker::ADD;
            model_marker.header.frame_id = "map";  // 设置坐标系
            model_marker.id = i;
            model_marker.type = visualization_msgs::Marker::MESH_RESOURCE;  // 表示网格模型
            Eigen::AngleAxisd ad_roll(map_foots.at(index).at(i).roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd ad_pitch(map_foots.at(index).at(i).pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd ad_yaw(map_foots.at(index).at(i).yaw, Eigen::Vector3d::UnitZ());
            Eigen::Matrix3d r = ad_roll.toRotationMatrix() * ad_pitch.toRotationMatrix() * ad_yaw.toRotationMatrix();
            // Eigen::Matrix3d r_stl;
            // r_stl<<1, 0, 0, 0, -1, 0, 0, 0, -1;
            // Eigen::Quaterniond qd(r_stl * r);
            Eigen::Quaterniond qd(r);
            model_marker.pose.orientation.w = qd.w();
            model_marker.pose.orientation.x = qd.x();
            model_marker.pose.orientation.y = qd.y();
            model_marker.pose.orientation.z = qd.z();
            model_marker.pose.position.x = map_foots.at(index).at(i).x;
            model_marker.pose.position.y = map_foots.at(index).at(i).y - 0.076;
            model_marker.pose.position.z = map_foots.at(index).at(i).z;

            model_marker.scale.x = 1.0;  // 调整模型大小
            model_marker.scale.y = 1.0;
            model_marker.scale.z = 1.0;
            model_marker.color.a = 1.0;
            // 获取包的路径
            std::string package_path = ros::package::getPath("draw_steps");

            // 设置模型的相对路径
            // mesh_resource_marker.mesh_resource = "file://" + package_path + "/path/to/your/model.stl";
            if (map_foots.at(index).at(i).is_left)
            {
                model_marker.mesh_resource = "file://" + package_path + "/data/leftfoot4.STL";  // 设置STL文件路径
                // model_marker.mesh_resource = "file://" + package_path + "/data/left_foot.STL";  // 设置STL文件路径
            }
            else
            {
                model_marker.mesh_resource = "file://" + package_path + "/data/rightfoot4.STL";  // 设置STL文件路径
                // model_marker.mesh_resource = "file://" + package_path + "/data/right_foot.STL";  // 设置STL文件路径
            }
            // 220,223,227
            model_marker.color.r = 220.0/255.0;
            model_marker.color.g = 223.0/255.0;
            model_marker.color.b = 227.0/255.0;
            new_model_markers.markers.emplace_back(model_marker);
        }
        pub_steps.publish(new_model_markers);
        loop_rate.sleep();
        new_model_markers.markers.clear();
        for (int i = 0; i < map_foots.at(index).size(); i++)
        {
            visualization_msgs::Marker model_marker;
            model_marker.action = visualization_msgs::Marker::ADD;
            model_marker.header.frame_id = "map";  // 设置坐标系
            model_marker.id = i;
            model_marker.type = visualization_msgs::Marker::MESH_RESOURCE;  // 表示网格模型
            model_marker.header.stamp = ros::Time::now();
            Eigen::AngleAxisd ad_roll(map_foots.at(index).at(i).roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd ad_pitch(map_foots.at(index).at(i).pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd ad_yaw(map_foots.at(index).at(i).yaw, Eigen::Vector3d::UnitZ());
            Eigen::Matrix3d r = ad_roll.toRotationMatrix() * ad_pitch.toRotationMatrix() * ad_yaw.toRotationMatrix();
            // Eigen::Matrix3d r_stl;
            // r_stl<<1, 0, 0, 0, -1, 0, 0, 0, -1;
            // Eigen::Quaterniond qd(r_stl * r);
            Eigen::Quaterniond qd(r);
            model_marker.pose.orientation.w = qd.w();
            model_marker.pose.orientation.x = qd.x();
            model_marker.pose.orientation.y = qd.y();
            model_marker.pose.orientation.z = qd.z();
            model_marker.pose.position.x = map_foots.at(index).at(i).x;
            model_marker.pose.position.y = map_foots.at(index).at(i).y - 0.076;
            model_marker.pose.position.z = map_foots.at(index).at(i).z;

            model_marker.scale.x = 1.0;  // 调整模型大小
            model_marker.scale.y = 1.0;
            model_marker.scale.z = 1.0;
            model_marker.color.a = 0;
            // 获取包的路径
            std::string package_path = ros::package::getPath("draw_steps");

            // 设置模型的相对路径
            // mesh_resource_marker.mesh_resource = "file://" + package_path + "/path/to/your/model.stl";
            if (map_foots.at(index).at(i).is_left)
            {
                model_marker.mesh_resource = "file://" + package_path + "/data/leftfoot4.STL";  // 设置STL文件路径
                // model_marker.mesh_resource = "file://" + package_path + "/data/left_foot.STL";  // 设置STL文件路径
            }
            else
            {
                model_marker.mesh_resource = "file://" + package_path + "/data/rightfoot4.STL";  // 设置STL文件路径
                // model_marker.mesh_resource = "file://" + package_path + "/data/right_foot.STL";  // 设置STL文件路径
            }
            // 220,223,227
            model_marker.color.r = 220.0/255.0;
            model_marker.color.g = 223.0/255.0;
            model_marker.color.b = 227.0/255.0;
            new_model_markers.markers.emplace_back(model_marker);
        }
        pub_steps.publish(new_model_markers);
        


        index++;
    }
    bag.close();
    return 0;

}