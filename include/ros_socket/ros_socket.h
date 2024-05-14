#include "Socket.h"
#include "TCPServer.h"
#include "ros/ros.h"
#include <deque>

#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "draco/compression/config/compression_shared.h"
#include "draco/compression/encode.h"
#include "draco/compression/expert_encode.h"
#include "draco/core/cycle_timer.h"
#include "draco/io/file_utils.h"
#include "draco/io/mesh_io.h"
#include "draco/io/point_cloud_io.h"
#include "draco/point_cloud/point_cloud_builder.h"
#define PRINT_LOG [](const std::string &strLogMsg) { std::cout << strLogMsg << std::endl; }
struct Options
{
    Options();

    bool is_point_cloud;
    int pos_quantization_bits;
    int tex_coords_quantization_bits;
    bool tex_coords_deleted;
    int normals_quantization_bits;
    bool normals_deleted;
    int generic_quantization_bits;
    bool generic_deleted;
    int compression_level;
    bool preserve_polygons;
    bool use_metadata;
    std::string input;
    std::string output;
};

Options::Options()
    : is_point_cloud(false),
      pos_quantization_bits(11),
      tex_coords_quantization_bits(10),
      tex_coords_deleted(false),
      normals_quantization_bits(8),
      normals_deleted(false),
      generic_quantization_bits(8),
      generic_deleted(false),
      compression_level(7),
      preserve_polygons(false),
      use_metadata(false) {}
struct RGBpt
{
    float x, y, z;
    char r, g, b;
};
struct FrameStamp
{
    std::uint64_t time;
    std::uint64_t frame_id;
    char topic_name; // to determine how to parse data 'p':point cloud 'o':odometry 'd':draco binary
    char instance;   // the instance id
    std::uint64_t dataLength;
    std::vector<char> data_binary;
    FrameStamp() : time(0), frame_id(0), topic_name(0), instance(0), dataLength(0)
    {
    }
    FrameStamp(std::uint64_t time, std::uint64_t frame_id, char topic_name, char instance, std::uint64_t dataLength) : time(time), frame_id(frame_id), topic_name(topic_name), instance(instance), dataLength(dataLength)
    {
        data_binary.reserve(dataLength);
    }
};

class MessageSender
{
private:
    std::unique_ptr<CTCPServer> server_ptr;
    std::unique_ptr<ASocket::Socket> socket_ptr;
    ros::NodeHandlePtr nh_ptr;
    ros::Subscriber pcd_sub;
    ros::Subscriber odo_sub;

public:
    Options options;
    std::thread thread;
    std::mutex mtx;
    std::deque<FrameStamp> dataQueue;
    draco::Encoder encoder;
    MessageSender(std::string port, ros::NodeHandlePtr nh_ptr) : server_ptr(std::make_unique<CTCPServer>(PRINT_LOG, port)), socket_ptr(std::make_unique<ASocket::Socket>()),
                                                                 nh_ptr(nh_ptr)
    {

        std::string pointCloudTopic;
        std::string odometryTopic;
        // parameter server
        ros::param::get("/ros_socket_node/point_cloud_topic", pointCloudTopic);
        ros::param::get("/ros_socket_node/odometry_topic", odometryTopic);
        // 订阅PointCloud2消息
        std::cout << "pointCloudTopic: " << pointCloudTopic << std::endl;
        pcd_sub = nh_ptr->subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &MessageSender::DracoPcCallback, this);
        odo_sub = nh_ptr->subscribe<nav_msgs::Odometry>(odometryTopic, 1, &MessageSender::odometryCallback, this);
        server_ptr->Listen(*socket_ptr, 10000);
        ROS_INFO("Server is listening on port %s", port.c_str());
        thread = std::thread(&MessageSender::send, this);

        const int speed = 10 - options.compression_level;
        // Setup encoder options.
        if (options.pos_quantization_bits > 0)
        {
            encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,
                                             options.pos_quantization_bits);
        }
        if (options.tex_coords_quantization_bits > 0)
        {
            encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD,
                                             options.tex_coords_quantization_bits);
        }
        if (options.normals_quantization_bits > 0)
        {
            encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL,
                                             options.normals_quantization_bits);
        }
        if (options.generic_quantization_bits > 0)
        {
            encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC,
                                             options.generic_quantization_bits);
        }
        encoder.SetSpeedOptions(speed, speed);
    }
    void send()
    {
        while (ros::ok())
        {
            ros::spinOnce();
            FrameStamp data;
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
                    if (!server_ptr->Send(*socket_ptr, reinterpret_cast<char *>(&data.topic_name), sizeof(data.topic_name)))
                    {
                        continue;
                    }
                    server_ptr->Send(*socket_ptr, reinterpret_cast<char *>(&data.time), sizeof(data.time));
                    server_ptr->Send(*socket_ptr, reinterpret_cast<char *>(&data.frame_id), sizeof(data.frame_id));
                    server_ptr->Send(*socket_ptr, reinterpret_cast<char *>(&data.dataLength), sizeof(data.dataLength));
                    server_ptr->Send(*socket_ptr, data.data_binary.data(), data.data_binary.size());
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

        FrameStamp frameStamp(msg->header.stamp.toNSec(), msg->header.seq, 'p', '0', msg->data.size());
        frameStamp.data_binary = std::vector<char>(msg->data.begin(), msg->data.end());
        std::lock_guard<std::mutex> lock(mtx);
        dataQueue.push_back(frameStamp);
    }
    void DracoPcCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {

        std::cout << "frame_id: " << msg->header.seq << std::endl;
        ROS_INFO("Received a point cloud message with %d points.", msg->width * msg->height);
        // construct draco format pointcloud
        std::vector<std::vector<float>> pointCloudData = deserializePointCloud(msg->data);
        std::vector<RGBpt> rgbpt_vector;
        for (auto &point : pointCloudData)
        {
            int32_t rgb = *(reinterpret_cast<uint32_t *>(&point[3]));
            uint8_t r = (rgb >> 16) & 0xff;
            uint8_t g = (rgb >> 8) & 0xff;
            uint8_t b = rgb & 0xff;
            RGBpt rgbpt = {point[0], point[1], point[2], r, g, b};
            rgbpt_vector.push_back(rgbpt);
        }
        draco::PointCloudBuilder builder;
        builder.Start(pointCloudData.size());
        const int pos_att_id =
            builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
        const int color_att_id =
            builder.AddAttribute(draco::GeometryAttribute::COLOR, 3, draco::DT_UINT8); // anchor
        for (draco::PointIndex i(0); i < pointCloudData.size(); ++i)
        {
            builder.SetAttributeValueForPoint(pos_att_id, i,
                                              &rgbpt_vector[i.value()].x);
            builder.SetAttributeValueForPoint(color_att_id, i,
                                              &rgbpt_vector[i.value()].r);
        }
        std::unique_ptr<draco::PointCloud> pc = builder.Finalize(false);
        // encode the pointcloud to the draco binary drc

        std::unique_ptr<draco::ExpertEncoder> expert_encoder;
        expert_encoder.reset(new draco::ExpertEncoder(*pc));
        expert_encoder->Reset(encoder.CreateExpertEncoderOptions(*pc));
        int ret = -1;
        draco::EncoderBuffer buffer;
        const draco::Status status = expert_encoder->EncodeToBuffer(&buffer);
        if (!status.ok())
        {
            printf("Failed to encode the point cloud.\n");
            printf("%s\n", status.error_msg());
        }

        std::vector<char> data_binary(buffer.data(), buffer.data() + buffer.size());
        FrameStamp frameStamp(msg->header.stamp.toNSec(), msg->header.seq, 'd', '0', data_binary.size() * sizeof(char));

        std::unique_ptr<draco::PointCloud> decpc;
        draco::DecoderBuffer decbuffer;
        decbuffer.Init(data_binary.data(), data_binary.size());
        auto type_statusor = draco::Decoder::GetEncodedGeometryType(&decbuffer);
        draco::Decoder decoder;
        auto statusor = decoder.DecodePointCloudFromBuffer(&decbuffer);

        if (!statusor.ok())
        {
            printf("error!");
        }else
        {
            
        }

        frameStamp.data_binary = std::vector<char>(data_binary.begin(), data_binary.end());
        std::lock_guard<std::mutex> lock(mtx);
        dataQueue.push_back(frameStamp);
        std::cout << "dataQueue size: " << dataQueue.size() << std::endl;
    }
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {

        std::cout << "frame_id: " << msg->header.seq << std::endl;
        ROS_INFO("Received Odometry messages.");
        // 提取 pose 中的 position 和 orientation
        geometry_msgs::Point position = msg->pose.pose.position;
        geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;

        // 将 position 和 orientation 存放在一个 vector<float> 中
        std::vector<float> data;
        data.push_back(position.x);
        data.push_back(position.y);
        data.push_back(position.z);
        data.push_back(orientation.x);
        data.push_back(orientation.y);
        data.push_back(orientation.z);
        data.push_back(orientation.w);

        // 将 vector<float> 转换为 vector<char>
        std::vector<char> char_data(data.size() * sizeof(float));
        char *char_ptr = reinterpret_cast<char *>(data.data());
        std::copy(char_ptr, char_ptr + char_data.size(), char_data.data());
        FrameStamp frameStamp(msg->header.stamp.toNSec(), msg->header.seq, 'o', '0', data.size() * sizeof(float));

        frameStamp.data_binary = std::vector<char>(char_data.begin(), char_data.end());
        std::lock_guard<std::mutex> lock(mtx);
        dataQueue.push_back(frameStamp);
    }

    std::vector<std::vector<float>> deserializePointCloud(const std::vector<uint8_t> &serializedData) // only for [xyzrgb] pc
    {
        std::vector<std::vector<float>> pointCloudData;

        std::size_t pointSize = sizeof(float) * 8; // 每个点的大小为3个float值
        for (std::size_t i = 0; i < serializedData.size(); i += pointSize)
        {
            const float *pointData = reinterpret_cast<const float *>(&serializedData[i]);
            std::vector<float> point(pointData, pointData + 3);                         // 每个点包含3个float值
            point.push_back(*reinterpret_cast<const float *>(&serializedData[i] + 16)); // r
            pointCloudData.push_back(point);
        }

        return pointCloudData;
    }
};