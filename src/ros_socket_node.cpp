#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros_socket/ros_socket.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_subscriber_node");
    ros::NodeHandle nh;
    std::string port;
    ros::param::get("/ros_socket_node/port", port);
    MessageSender sender(port, ros::NodeHandlePtr(&nh));
    sender.thread.join();
    return 0;
}
