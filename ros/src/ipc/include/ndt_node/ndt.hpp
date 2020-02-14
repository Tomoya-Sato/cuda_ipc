#ifndef NDT_NODE_H
#define NDT_NODE_H

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class NdtNode
{
  public:
    NdtNode();
    ~NdtNode();
    void getHandle(unsigned char *handle);
    void publishPoints(int size, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr);

  private:
    bool empty_handle_;
    pcl::PointXYZ *input_;
};

#endif
