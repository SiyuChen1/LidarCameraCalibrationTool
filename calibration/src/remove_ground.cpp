#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 
//pcl 常用头文件


class RemoveGroundNode{
public:
    RemoveGroundNode(ros::NodeHandle& nh_ ):nh(nh_){
        ros::param::param<std::string>("~topic_name", topic_name_, "/velodyne64/PointCloud2");
        pub_points = nh.advertise<sensor_msgs::PointCloud2>("points_no_ground", 2);
        sub_points = nh.subscribe(topic_name_, 2 , &RemoveGroundNode::pointsCallback,this);
    }
    ~RemoveGroundNode(){}
    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
        pcl::PointCloud<pcl::PointXYZI> cloud_in;
        pcl::PointCloud<pcl::PointXYZI> cloud_out;
        pcl::fromROSMsg(*msg, cloud_in);

        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud_in.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1.8,10.0);
        pass.filter(cloud_in);

        pass.setInputCloud(cloud_in.makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0,17);
        pass.filter(cloud_out);

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud_out, cloud_msg);
        cloud_msg.header = msg->header;
        pub_points.publish(cloud_msg);
    } 
private:
    ros::NodeHandle nh;
    ros::Publisher pub_points;
    ros::Subscriber sub_points;
    std::string topic_name_;
};

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "remove_groud");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  RemoveGroundNode remove_ground_node(n);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}