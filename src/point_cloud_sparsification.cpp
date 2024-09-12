#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

class PointCloudSparsificationNode : public rclcpp::Node
{
public:
    PointCloudSparsificationNode() : Node("point_cloud_sparsification_node")
    {
        // 订阅点云话题
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/nebula200/stof_points2", 10,
            std::bind(&PointCloudSparsificationNode::pointCloudCallback, this, std::placeholders::_1));

        // 发布稀疏化后的点云话题
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output/sparse_point_cloud", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 将ROS点云消息转换为PCL点云对象
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // 创建VoxelGrid滤波器对象
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.05f, 0.05f, 0.05f); // 设置体素网格叶子大小，即稀疏化参数

        // 输出稀疏化后的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*cloud_filtered);
        
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud_filtered);
        outrem.setRadiusSearch(0.1);  // 搜索半径
        outrem.setMinNeighborsInRadius(5);  // 阈值内至少5个邻居点
        outrem.filter(*cloud_filtered);

        // 将PCL点云对象转换为ROS点云消息
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_filtered, output);
        output.header = msg->header;

        // 发布稀疏化后的点云消息
        point_cloud_pub_->publish(output);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSparsificationNode>());
    rclcpp::shutdown();
    return 0;
}

