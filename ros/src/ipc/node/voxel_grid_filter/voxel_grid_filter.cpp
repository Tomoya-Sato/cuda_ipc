#include <std_msgs/Bool.h>
#include <ipc_msgs/gpu_handle.h>
#include <ipc_msgs/data_size.h>

#include <voxel_grid_filter/voxel_grid_filter.hpp>

static ros::Publisher ack_pub;
static ros::Publisher handle_pub;
static ros::Publisher data_pub;

static ros::Subscriber sub;

VoxelGridFilter vgrid;

ipc_msgs::gpu_handle handle_msg;
ipc_msgs::data_size data_msg;

std_msgs::Bool bl1, bl2;

void setHandle()
{
  handle_msg.data.clear();
  unsigned char *buf = vgrid.initGpuMemory();

  ROS_INFO("filtered_handle: %s", buf);

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
    ROS_INFO("Recived filtered_ack");
    sleep(1);
    handle_pub.publish(handle_msg);
    ROS_INFO("Published filtered_handle");
  }
  else
  {
    ROS_INFO("Recieved ready");
  }
}

void handle_callback(const ipc_msgs::gpu_handle msg)
{
  ROS_INFO("Handle_callback");
  unsigned char handle[65];
  for (int i = 0; i < 64; i++)
  {
    handle[i] = msg.data[i];
  }

  ROS_INFO("Received handle: %s", handle);

  vgrid.getHandle(handle);
  vgrid.setLeafsize(2.0);

  sleep(1);

  ack_pub.publish(bl2);
  ROS_INFO("Published ready");

  sub.shutdown();
}

void data_callback(const ipc_msgs::data_size msg)
{
  int ret_size = vgrid.filterPoints(msg.data);
  data_msg.data = ret_size;
  data_pub.publish(data_msg);

  ROS_INFO("Published size: %d", ret_size);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "GPU_voxelgridfilter");
  ros::NodeHandle n;

  bl1.data = true;
  bl2.data = false;

  ack_pub = n.advertise<std_msgs::Bool>("points_raw_ack", 10);
  handle_pub = n.advertise<ipc_msgs::gpu_handle>("filtered_gpu_handle", 10);
  data_pub = n.advertise<ipc_msgs::data_size>("filtered_size", 10);

  vgrid.setMaxSize(100000);

  setHandle();

  sleep(1);

  handle_pub.publish(handle_msg);
  ack_pub.publish(bl1);
  ROS_INFO("Published pointns_raw_ack");

  sub = n.subscribe("points_raw_handle", 10, handle_callback);
  ros::Subscriber data_sub = n.subscribe("points_raw_size", 10, data_callback);
  ros::Subscriber ack_sub = n.subscribe("filtered_ack", 10, ack_callback);

  ros::spin();

  return 0;
}
