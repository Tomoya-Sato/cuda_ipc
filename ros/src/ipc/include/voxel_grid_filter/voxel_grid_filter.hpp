#ifndef VOXELGRID_H
#define VOXELGRID_H

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define BUF_SIZE 10

typedef struct points_average
{
    float x, y, z;
    int cnt;
} Ave;

class VoxelGridFilter
{
    public:

        VoxelGridFilter();
        int filterPoints(int size);
        void freeFiltered();

        void getHandle(unsigned char *data);
        unsigned char* initGpuMemory();
        int updateData();
        void rotateBuffer();
        void storeBuffer(pcl::PointXYZ *tmp, size_t size);
        void freeHandleBuffer();

        void setLeafsize(float x);
        void setMaxSize(int max_size);
    
    private:

        float leafsize_;
        int max_size_;
        int size_;

        pcl::PointXYZ *input_;
        pcl::PointXYZ *filtered_;
        pcl::PointXYZ *tmp_input_;

        bool empty_handle_;
        unsigned char *handle_buffer_;
        pcl::PointXYZ *buf_[BUF_SIZE];
        int size_buffer_[BUF_SIZE];
        int buf_counter_;
};

#endif
