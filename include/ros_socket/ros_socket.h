#include "Socket.h"
#include "TCPServer.h"
#include "ros/ros.h"
#include <deque>

#include <thread>
#include <sensor_msgs/PointCloud2.h>
#define PRINT_LOG [](const std::string &strLogMsg) { std::cout << strLogMsg << std::endl; }
struct RGBpt
{
    float x, y, z;
    char r, g, b;
};
struct FrameStamp
{
    std::size_t time;
    std::size_t frame_id;
    char topic_name; // to determine how to parse data
    char instance;   // the instance id
    std::size_t dataLength;
};

class MessageSender
{
private:
    std::unique_ptr<CTCPServer> server_ptr;
    std::unique_ptr<ASocket::Socket> socket_ptr;
    ros::NodeHandlePtr nh_ptr;
    ros::Subscriber pcd_sub;

public:
    std::thread thread;
    std::mutex mtx;
    std::deque<std::pair<FrameStamp, std::vector<char>>> dataQueue;
    MessageSender(std::string port, ros::NodeHandlePtr nh_ptr) : server_ptr(std::make_unique<CTCPServer>(PRINT_LOG, port)), socket_ptr(std::make_unique<ASocket::Socket>()),
                                                                 nh_ptr(nh_ptr)
    {

        std::string pointCloudTopic;
        // parameter server
        ros::param::get("/ros_socket_node/point_cloud_topic", pointCloudTopic);
        // 订阅PointCloud2消息
        std::cout << "pointCloudTopic: " << pointCloudTopic << std::endl;
        pcd_sub = nh_ptr->subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &MessageSender::pointCloudCallback, this);
        server_ptr->Listen(*socket_ptr, 10000);
        ROS_INFO("Server is listening on port %s", port.c_str());
        thread = std::thread(&MessageSender::send, this);
    }
    void send()
    {
        while (ros::ok())
        {
            ros::spinOnce();
            std::pair<FrameStamp, std::vector<char>> data;
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (dataQueue.empty())
                {
                    continue;
                }
                else
                {
                    data = dataQueue.front();
                    dataQueue.pop_front();
                    if (!server_ptr->Send(*socket_ptr, reinterpret_cast<char *>(&data.first.topic_name), sizeof(data.first.topic_name)))
                    {
                        continue;
                    }
                    server_ptr->Send(*socket_ptr, reinterpret_cast<char *>(&data.first.time), sizeof(data.first.time));
                    server_ptr->Send(*socket_ptr, reinterpret_cast<char *>(&data.first.frame_id), sizeof(data.first.frame_id));
                    server_ptr->Send(*socket_ptr, reinterpret_cast<char *>(&data.first.dataLength), sizeof(data.first.dataLength));
                    server_ptr->Send(*socket_ptr, data.second.data(), data.second.size());
                }
            }
        }
    }
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) // x y z rgb 4*8bytes
    {
        // 在此处添加处理点云消息的代码
        // 例如：打印点云数据的一些信息
        std::cout << "frame_id: " << msg->header.seq << std::endl;
        ROS_INFO("Received a point cloud message with %d points.", msg->width * msg->height);

        FrameStamp frameStamp;
        frameStamp.time = msg->header.stamp.toNSec();
        frameStamp.frame_id = msg->header.seq;
        frameStamp.topic_name = 'p';
        frameStamp.instance = '0';

        std::vector<char> data(msg->data.begin(), msg->data.end());
        frameStamp.dataLength = data.size();
        std::lock_guard<std::mutex> lock(mtx);
        dataQueue.push_back(std::make_pair(frameStamp, data));
    }
};