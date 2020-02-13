#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <gpu_manager/gpu_manager_cuda.hpp>

static ros::Publisher handle_pub;
static ros::Publisher data_pub;

static GpuIpc gmng;

void setHandle()
{
  handle_msg.data.clear();
  unsigned char *buf = gmng.initGpuMemory();

  for (int i = 0; i < 64; i++)
  {
    handle_msg.data.push_back(buf[i]);
  }

  free(buf);
}

void ack_callback(std_msgs::Bool msg)
{
  if (msg.data)
  {
    ROS_INFO("Recived points_raw_ack");
    gpu_pub.publish(handle_msg);
    ROS_INFO("Published points_raw_handle");
  }
  else
  {
    ROS_INFO("Recieved ready");
  }
}

void points_callback(sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  pcl::PointXYZ *input = cloud.points.data();
  gmng.storeBuffer(input, (int)cloud.size());

  data_msg.data = gmng.updateData();
  data_pub.publish(data_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpu_manager");
  ros::NodeHandle n;

  handle_pub = n.advertise<ipc::gpu_handle>("points_raw_handle", 10);
  data_pub = n.advertise<std_msgs::Int32>("points_raw_size", 10);

  sleep(1);

  handle_pub.publish(handle_msg);
  ROS_INFO("Pulished points_raw_handle");

  ros::Subscriber ack_sub = n.subscribe("points_raw_ack", 10, ack_callback);
  ros::Subscriber points_sub = n.subscribe("points_raw", 10, points_callback);

  ros::spin();

  ins.freeResource();

  return 0;
}