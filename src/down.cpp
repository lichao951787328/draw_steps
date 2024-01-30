#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
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
int main(int argc, char** argv)
{
    ros::init(argc, argv, "down");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<grid_map_msgs::GridMap>("globalmap", 1);
    ros::Publisher pub_submap = nh.advertise<grid_map_msgs::GridMap>("submap", 1);
    ros::Publisher pub_steps = nh.advertise<visualization_msgs::MarkerArray>("steps", 1);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    tf2_ros::TransformBroadcaster broadcaster;
    std::string globalmap_path = "/home/lichao/Darwin-op/src/draw_steps/data/global_map.bag";
    std::string globalmap_topic = "/global_map";

    rosbag::Bag bag;
    bag.open(globalmap_path, rosbag::bagmode::Read);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    // pcl::io::loadPCDFile<pcl::PointXYZ>("/home/lichao/Darwin-op/src/draw_steps/data/scans.pcd", cloud);
    string pcd_file_path = "/home/lichao/Darwin-op/src/draw_steps/data/scans.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s\n", pcd_file_path.c_str());
        return -1;
    }
    rosbag::View view(bag, rosbag::TopicQuery(globalmap_topic));
    grid_map::GridMap global_map;
    BOOST_FOREACH(rosbag::MessageInstance const& msg, view) {
        // 替换成你实际使用的消息类型
        grid_map_msgs::GridMap::ConstPtr map_mag = msg.instantiate<grid_map_msgs::GridMap>();
        if (map_mag != nullptr) {
            // 处理消息，这里仅打印消息内容
            // std::cout << "Timestamp: " << msg.getTime() << ", Data: " << string_msg->data << std::endl;
            grid_map::GridMapRosConverter::fromMessage(*map_mag, global_map);
        }
    }
    bool get_submap_flag;
    vector<grid_map::GridMap> submaps;
    grid_map::GridMap submap = global_map.getSubmap(grid_map::Position(1.9, 0), grid_map::Length(1, 1), get_submap_flag);
    submaps.emplace_back(submap);

    submaps.emplace_back(global_map.getSubmap(grid_map::Position(1.7, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(1.6, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(1.4, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(1.31, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(1.11, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(1, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(0.8, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(0.73, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(0.53, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(0.46, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(0.26, 0), grid_map::Length(1, 1), get_submap_flag));
    submaps.emplace_back(global_map.getSubmap(grid_map::Position(0.16, 0), grid_map::Length(1, 1), get_submap_flag));

    vector<vector<foot>> map_foots;
    vector<foot> foots1;
    foots1.emplace_back(foot(2.1,   0.1, 0.53,  0, 0,       180/57.3, true ));
    foots1.emplace_back(foot(2.1,  -0.1, 0.53,  0, 0,       180/57.3, false));
    foots1.emplace_back(foot(1.8,  0.1, 0.42, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.8, -0.1, 0.42, 0,  0, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(2.1,  -0.1, 0.53,  0, 0,       180/57.3, false));
    foots1.emplace_back(foot(1.81,  0.1, 0.42, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.81, -0.1, 0.42, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(1.52,  0.1, 0.31, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.52, -0.1, 0.31, 0,  0, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(1.8,  0.1, 0.42, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.8, -0.1, 0.42, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(1.52,  0.1, 0.31, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.52, -0.1, 0.31, 0,  0, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(1.8, -0.1, 0.42, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(1.52,  0.1, 0.31, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.52, -0.1, 0.31, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(1.26,  0.1, 0.2, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.26, -0.1, 0.2, 0,  0, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(1.52,  0.1, 0.31, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.52, -0.1, 0.31, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(1.26,  0.1, 0.2, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.26, -0.1, 0.2, 0,  0, 180/57.3, false));
    // foots1.emplace_back(foot(0.94,  0.1, 0.09, 0,  0, 180/57.3, true ));
    // foots1.emplace_back(foot(0.94, -0.1, 0.09, 0,  0, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(1.52, -0.1, 0.31, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(1.24,  0.1, 0.2, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.24, -0.1, 0.2, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(0.97,  0.1, 0.09, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(0.97, -0.1, 0.09, 0,  0, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(1.24,  0.1, 0.2, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(1.24, -0.1, 0.2, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(0.97,  0.1, 0.09, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(0.97, -0.1, 0.09, 0,  0, 180/57.3, false));
    // foots1.emplace_back(foot(0.74,  0.1, 0.09, 0,  -14/57.3, 180/57.3, true ));
    // foots1.emplace_back(foot(0.74, -0.1, 0.09, 0,  -14/57.3, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(1.24, -0.1, 0.2, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(0.97,  0.1, 0.09, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(0.97, -0.1, 0.09, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(0.74,  0.1, 0.09, 0,  -14/57.3, 180/57.3, true ));
    foots1.emplace_back(foot(0.74, -0.1, 0.09, 0,  -14/57.3, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.97,  0.1, 0.09, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(0.97, -0.1, 0.09, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(0.74,  0.1, 0.09, 0,  -14/57.3, 180/57.3, true ));
    foots1.emplace_back(foot(0.74, -0.1, 0.09, 0,  -14/57.3, 180/57.3, false));
    foots1.emplace_back(foot(0.61,  0.1, 0.06, 0,  -14/57.3, 180/57.3, true ));
    foots1.emplace_back(foot(0.61, -0.1, 0.06, 0,  -14/57.3, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.97, -0.1, 0.09, 0,  0, 180/57.3, false));
    foots1.emplace_back(foot(0.74,  0.1, 0.09, 0,  -14/57.3, 180/57.3, true ));
    foots1.emplace_back(foot(0.74, -0.1, 0.09, 0,  -14/57.3, 180/57.3, false));
    foots1.emplace_back(foot(0.61,  0.1, 0.06, 0,  -14/57.3, 180/57.3, true ));
    foots1.emplace_back(foot(0.61, -0.1, 0.06, 0,  -14/57.3, 180/57.3, false));
    foots1.emplace_back(foot(0.34,  0.1, 0.0, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(0.34, -0.1, 0.0, 0,  0, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    foots1.emplace_back(foot(0.74,  0.1, 0.09, 0,  -14/57.3, 180/57.3, true ));
    foots1.emplace_back(foot(0.74, -0.1, 0.09, 0,  -14/57.3, 180/57.3, false));
    foots1.emplace_back(foot(0.61,  0.1, 0.06, 0,  -14/57.3, 180/57.3, true ));
    foots1.emplace_back(foot(0.61, -0.1, 0.06, 0,  -14/57.3, 180/57.3, false));
    foots1.emplace_back(foot(0.34,  0.1, 0.0, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(0.34, -0.1, 0.0, 0,  0, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    // foots1.emplace_back(foot(0.74,  0.1, 0.09, 0,  -14/57.3, 180/57.3, true ));
    foots1.emplace_back(foot(0.74, -0.1, 0.09, 0,  -14/57.3, 180/57.3, false));
    foots1.emplace_back(foot(0.61,  0.1, 0.06, 0,  -14/57.3, 180/57.3, true ));
    foots1.emplace_back(foot(0.61, -0.1, 0.06, 0,  -14/57.3, 180/57.3, false));
    foots1.emplace_back(foot(0.34,  0.1, 0.0, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(0.34, -0.1, 0.0, 0,  0, 180/57.3, false));
    map_foots.emplace_back(foots1);

    foots1.clear();
    // foots1.emplace_back(foot(0.74,  0.1, 0.09, 0,  -14/57.3, 180/57.3, true ));
    // foots1.emplace_back(foot(0.74, -0.1, 0.09, 0,  -14/57.3, 180/57.3, false));
    foots1.emplace_back(foot(0.61,  0.1, 0.06, 0,  -14/57.3, 180/57.3, true ));
    foots1.emplace_back(foot(0.61, -0.1, 0.06, 0,  -14/57.3, 180/57.3, false));
    foots1.emplace_back(foot(0.34,  0.1, 0.0, 0,  0, 180/57.3, true ));
    foots1.emplace_back(foot(0.34, -0.1, 0.0, 0,  0, 180/57.3, false));
    map_foots.emplace_back(foots1);

    sleep(3);
    ros::Rate loop_rate(0.4);
    int index = 0;
    double pianyi_x = 0;
    double step_pianyi = 0;
    vector<double> pinayis;
    pinayis.emplace_back(2.9);
    pinayis.emplace_back(2.7);
    pinayis.emplace_back(2.6);
    pinayis.emplace_back(2.4);
    pinayis.emplace_back(2.31-0.05);
    pinayis.emplace_back(2.11-0.05);
    pinayis.emplace_back(2-0.1);
    pinayis.emplace_back(1.8-0.1);
    pinayis.emplace_back(1.73-0.1);
    pinayis.emplace_back(1.53-0.1);
    pinayis.emplace_back(1.46-0.1);
    pinayis.emplace_back(1.26-0.1);
    pinayis.emplace_back(1.16-0.1);

    vector<double> pinayis_map;
    pinayis_map.emplace_back(0.6);
    pinayis_map.emplace_back(0.5);
    pinayis_map.emplace_back(0.48);
    pinayis_map.emplace_back(0.46);
    pinayis_map.emplace_back(0.45);
    pinayis_map.emplace_back(0.43);
    pinayis_map.emplace_back(0.41);
    pinayis_map.emplace_back(0.39);
    pinayis_map.emplace_back(0.4);
    pinayis_map.emplace_back(0.4);
    pinayis_map.emplace_back(0.4);
    pinayis_map.emplace_back(0.4);
    pinayis_map.emplace_back(0.4);

    while (ros::ok() && index < submaps.size())
    {
        grid_map_msgs::GridMap global_map_msg;
        grid_map::GridMapRosConverter::toMessage(global_map, global_map_msg);
        global_map_msg.info.header.frame_id = "map";

        grid_map_msgs::GridMap submap_msg;
        grid_map::GridMapRosConverter::toMessage(submaps.at(index), submap_msg);
        submap_msg.info.header.frame_id = "map";
        submap_msg.info.header.stamp = ros::Time::now();

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.frame_id = "world";  // 父坐标系
        transformStamped.child_frame_id = "map";  // 子坐标系
        transformStamped.header.stamp = ros::Time::now();
        // if (map_foots.at(index).begin()->is_left == true)
        // {
        //     step_pianyi = map_foots.at(index).begin()->x/3;
        //     pianyi_x += map_foots.at(index).begin()->x - step_pianyi;
        // }
        // else
        // {
        //     pianyi_x += step_pianyi;
        // }
        // Eigen::AngleAxisd ad(3.1415926, Eigen::Vector3d::UnitZ());
        // Eigen::Quaterniond qd(ad.toRotationMatrix());
        transformStamped.transform.translation.x = pinayis_map.at(index);
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = -(0.7217 + 0.69);
        // transformStamped.transform.rotation.x = qd.x();
        // transformStamped.transform.rotation.y = qd.y();
        // transformStamped.transform.rotation.z = qd.z();
        // transformStamped.transform.rotation.w = qd.w();
        transformStamped.transform.rotation.x = 0;
        transformStamped.transform.rotation.y = 0;
        transformStamped.transform.rotation.z = 0;
        transformStamped.transform.rotation.w = 1;
        broadcaster.sendTransform(transformStamped);

        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "id";
        Eigen::AngleAxisd ad(3.1415926, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond qd(ad.toRotationMatrix());
        transformStamped.transform.rotation.x = qd.x();
        transformStamped.transform.rotation.y = qd.y();
        transformStamped.transform.rotation.z = qd.z();
        transformStamped.transform.rotation.w = qd.w();
        transformStamped.transform.translation.x = pinayis.at(index);
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = -(0.7217 + 0.69);
        broadcaster.sendTransform(transformStamped);

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);
        cloud_msg.header.frame_id = "world";
        cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(cloud_msg);

        visualization_msgs::MarkerArray new_model_markers;
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
                model_marker.mesh_resource = "file://" + package_path + "/data/rightfoot4.STL";  // 设置STL文件路径
                // model_marker.mesh_resource = "file://" + package_path + "/data/left_foot.STL";  // 设置STL文件路径
            }
            else
            {
                model_marker.mesh_resource = "file://" + package_path + "/data/leftfoot4.STL";  // 设置STL文件路径
                // model_marker.mesh_resource = "file://" + package_path + "/data/right_foot.STL";  // 设置STL文件路径
            }
            // 220,223,227
            model_marker.color.r = 220.0/255.0;
            model_marker.color.g = 223.0/255.0;
            model_marker.color.b = 227.0/255.0;
            new_model_markers.markers.emplace_back(model_marker);
        }
        pub_steps.publish(new_model_markers);

        pub.publish(global_map_msg);
        pub_submap.publish(submap_msg);
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
    
    
    return 0;
}
