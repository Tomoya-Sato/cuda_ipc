#include <ipc_msgs/gpu_handle.h>
#include <ipc_msgs/data_size.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <ndt_node/ndt.hpp>

static ros::Publisher ack_pub;
static ros::Publisher points_pub;

static ros::Subscriber sub;

NdtNode node;

std_msgs::Bool bl1, bl2;

void handle_callback(const ipc_msgs::gpu_handle msg)
{
  ROS_INFO("Handle_callback");
  unsigned char handle[65];
  for (int i = 0; i < 64; i++)
  {
    handle[i] = msg.data[i];
  }

  ROS_INFO("Received handle: %s", handle);
  
  node.getHandle(handle);

  sleep(1);

  ack_pub.publish(bl2);
  ROS_INFO("Published ready");

  sub.shutdown();
}

void data_callback(const ipc_msgs::data_size msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());

  node.publishPoints(msg.data, filtered_scan_ptr);

  sensor_msgs::PointCloud2 filtered_msg;

  pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
  filtered_msg.header.frame_id = "velodyne";

  points_pub.publish(filtered_msg);
  ROS_INFO("Received size: %d", msg.data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ndt_node");
  ros::NodeHandle n;

  bl1.data = true;
  bl2.data = false;

  ack_pub = n.advertise<std_msgs::Bool>("filtered_ack", 10);
  points_pub = n.advertise<sensor_msgs::PointCloud2>("filtered_points", 10);
  
  sleep(1);

  ack_pub.publish(bl1);
  ROS_INFO("Published filtered_ack");

  sub = n.subscribe("filtered_gpu_handle", 10, handle_callback);
  ros::Subscriber data_sub = n.subscribe("filtered_size", 10, data_callback);

  ros::spin();

  return 0;
}
