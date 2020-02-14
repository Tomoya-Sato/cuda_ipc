#include <iostream>

#include <thrust/device_ptr.h>
#include <thrust/scan.h>

#include <voxel_grid_filter/voxel_grid_filter.hpp>

#define BLOCK_SIZE 512
#define DSIZE 300000

// Error check macro for cuda
inline void gassert(cudaError_t err_code, const char *file, int line)
{
    if (err_code != cudaSuccess)
    {
        fprintf(stderr, "Error: %s %s %d\n", cudaGetErrorString(err_code), file, line);
        sleep(1);
        exit(EXIT_FAILURE);
    }
}

#define checkCudaErrors(val) gassert(val, __FILE__, __LINE__)


VoxelGridFilter::VoxelGridFilter()
{
    empty_handle_ = true;
    buf_counter_ = 0;
    checkCudaErrors(cudaMalloc((void**)&tmp_input_, sizeof(pcl::PointXYZ)*DSIZE));
}


// CUDA IPC part
void VoxelGridFilter::getHandle(unsigned char *data)
{
    if (empty_handle_)
    {
        cudaIpcMemHandle_t my_handle;

        memcpy((unsigned char*)&my_handle, data, sizeof(my_handle));

        checkCudaErrors(cudaIpcOpenMemHandle((void**)&input_, my_handle, cudaIpcMemLazyEnablePeerAccess));

        empty_handle_ = false;
    }

    return;
}


unsigned char* VoxelGridFilter::initGpuMemory()
{
    // checkCudaErrors(cudaMalloc((void**)&filtered_, sizeof(pcl::PointXYZ)*max_size_));

    cudaIpcMemHandle_t my_handle;
    handle_buffer_ = (unsigned char*)malloc(sizeof(my_handle)+1);

    checkCudaErrors(cudaIpcGetMemHandle(&my_handle, filtered_));

    memset(handle_buffer_, 0, sizeof(my_handle)+1);
    memcpy(handle_buffer_, (unsigned char*)(&my_handle), sizeof(my_handle));

    return handle_buffer_;
}


int VoxelGridFilter::updateData()
{
    size_t ret = size_buffer_[0];

    checkCudaErrors(cudaMemcpy(input_, buf_[0], sizeof(pcl::PointXYZ)*ret, cudaMemcpyHostToDevice));

    free(buf_[0]);
    for (int i = 0; i < buf_counter_; i++)
    {
        buf_[i] = buf_[i+1];
        size_buffer_[i] = size_buffer_[i+1];
    }

    buf_counter_--;

    return ret;
}


void VoxelGridFilter::rotateBuffer()
{
    free(buf_[0]);

    for (int i = 0; i < BUF_SIZE-1; i++) 
    {
        buf_[i] = buf_[i+1];
        size_buffer_[i] = size_buffer_[i+1];
    }

    buf_counter_--;

    ROS_INFO("The most old data in the data queue has been discarded.");

    return;
}


void VoxelGridFilter::storeBuffer(pcl::PointXYZ *tmp, size_t size)
{
    if (buf_counter_ == BUF_SIZE) rotateBuffer();

    buf_[buf_counter_] = (pcl::PointXYZ*)malloc(sizeof(pcl::PointXYZ)*size);

    memcpy(buf_[buf_counter_], tmp, sizeof(pcl::PointXYZ)*size);

    size_buffer_[buf_counter_] = size;

    buf_counter_++;

    return;
}


void VoxelGridFilter::freeHandleBuffer()
{
    free(handle_buffer_);

    return;
}



// Set voxel grid parameter
void VoxelGridFilter::setMaxSize(int max_size)
{
    max_size_ = max_size;
    checkCudaErrors(cudaMalloc((void**)&filtered_, sizeof(pcl::PointXYZ)*max_size_));

    return;
}


void VoxelGridFilter::setLeafsize(float x)
{
    leafsize_ = x;

    return;
}


// Move the maximum value to the anterior half of array
// and the minmum value to the posterior half
__global__ void separateMaxMin(pcl::PointXYZ *data, int full_size, int half_size)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;
    float tmp;
    int post_id;

    for (int i = id; i < half_size; i+= stride)
    {
        if (i + half_size >= full_size) break;

        post_id = i + half_size;
        
        if (data[i].x < data[post_id].x)
        {
            tmp = data[i].x;
            data[i].x = data[post_id].x;
            data[post_id].x = tmp;
        }

        if (data[i].y < data[post_id].y)
        {
            tmp = data[i].y;
            data[i].y = data[post_id].y;
            data[post_id].y = tmp;
        }

        if (data[i].z < data[post_id].z)
        {
            tmp = data[i].z;
            data[i].z = data[post_id].z;
            data[post_id].z = tmp;
        }
    }
}


__global__ void findMaxMin(pcl::PointXYZ *data, pcl::PointXYZ *min_data, int full_size, int min_full_size, int half_size, int min_half_size)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;

    for (int i = id; i < (half_size + min_half_size); i+=stride)
    {
        if (i < half_size)
        {
            int ihalf = i + half_size;
            data[i].x = (i + half_size < full_size) ? ((data[i].x >= data[ihalf].x) ? data[i].x : data[ihalf].x) : data[i].x;
            data[i].y = (i + half_size < full_size) ? ((data[i].y >= data[ihalf].y) ? data[i].y : data[ihalf].y) : data[i].y;
            data[i].z = (i + half_size < full_size) ? ((data[i].z >= data[ihalf].z) ? data[i].z : data[ihalf].z) : data[i].z;
        }
        else
        {
            int tmpi = i - half_size;
            int mihalf = tmpi + min_half_size;
            min_data[tmpi].x = (tmpi + min_half_size < min_full_size) ? ((min_data[tmpi].x <= min_data[mihalf].x) ? min_data[tmpi].x : min_data[mihalf].x) : min_data[tmpi].x;
            min_data[tmpi].y = (tmpi + min_half_size < min_full_size) ? ((min_data[tmpi].y <= min_data[mihalf].y) ? min_data[tmpi].y : min_data[mihalf].y) : min_data[tmpi].y;
            min_data[tmpi].z = (tmpi + min_half_size < min_full_size) ? ((min_data[tmpi].z <= min_data[mihalf].z) ? min_data[tmpi].z : min_data[mihalf].z) : min_data[tmpi].z;
        }
    }
}

// Get minimum and maximum coordinates
void getMinMax3D(pcl::PointXYZ *input, int points_num, float min[3], float max[3])
{
    int half_points_num = (points_num - 1) / 2 + 1;
    int block_x = (half_points_num > BLOCK_SIZE) ? BLOCK_SIZE : half_points_num;
    int grid_x = (half_points_num - 1) / block_x + 1;

    separateMaxMin<<<grid_x, block_x>>>(input, points_num, half_points_num);
    checkCudaErrors(cudaGetLastError());

    int min_points_num = half_points_num;
    int min_half_points_num;

    pcl::PointXYZ *min_input = input + (points_num / 2);

    points_num = half_points_num;

    while (points_num > 1) 
    {
        half_points_num = (points_num - 1) / 2 + 1;
        min_half_points_num = (min_points_num - 1) / 2 + 1;
        block_x = ((half_points_num + min_half_points_num) > BLOCK_SIZE) ? BLOCK_SIZE : half_points_num;
        grid_x = (half_points_num + min_half_points_num - 1) / block_x + 1;

        findMaxMin<<<block_x, grid_x>>>(input, min_input, points_num, min_points_num, half_points_num, min_half_points_num);

        points_num = half_points_num;
        min_points_num = min_half_points_num;
    }

    checkCudaErrors(cudaDeviceSynchronize());

    pcl::PointXYZ ret_min, ret_max;
    checkCudaErrors(cudaMemcpy(&ret_max, input, sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost));
    checkCudaErrors(cudaMemcpy(&ret_min, min_input, sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost));

    checkCudaErrors(cudaGetLastError());

    min[0] = ret_min.x;
    min[1] = ret_min.y;
    min[2] = ret_min.z;

    max[0] = ret_max.x;
    max[1] = ret_max.y;
    max[2] = ret_max.z;

    return;
}


__global__ void makeIndices(pcl::PointXYZ *input, int size, int *indices, float min_x, float min_y, float min_z, int num_x, int num_y, float leafsize)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = gridDim.x * blockDim.x;
    int id_x, id_y, id_z;

    for (int i = id; i < size; i+= stride)
    {
        id_x = (int)((input[i].x - min_x) / leafsize);
        id_y = (int)((input[i].y - min_y) / leafsize);
        id_z = (int)((input[i].z - min_z) / leafsize);

        indices[i] = id_x + id_y * num_x + id_z * (num_x * num_y);
    }

    return;
}


__global__ void countIndices(int *indices, unsigned int *histo, int size)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;

    int min = blockIdx.x * blockDim.x;
    int max = (blockIdx.x + 1) * blockDim.x;

    extern __shared__ unsigned int tmp[];

    tmp[threadIdx.x] = 0;

    __syncthreads();

    for (int i = threadIdx.x; i < size; i+=blockDim.x)
    {
        if (min <= indices[i] && indices[i] < max)
        {
            atomicAdd(&tmp[indices[i] - min], 1);
        }
    }

    __syncthreads();

    histo[id] = tmp[threadIdx.x];

    return;
}


__global__ void averagePoints(int *indices, pcl::PointXYZ *input, pcl::PointXYZ *output, int size, int indiceSize, int *red)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;

    int min = blockIdx.x * blockDim.x;
    int max = (blockIdx.x + 1) * blockDim.x;

    extern __shared__ Ave tmp1[];
    tmp1[threadIdx.x].x = 0.0;
    tmp1[threadIdx.x].y = 0.0;
    tmp1[threadIdx.x].z = 0.0;
    tmp1[threadIdx.x].cnt = 0;

    __syncthreads();

    for (int i = threadIdx.x; i < size; i+=blockDim.x)
    {
        if (min <= indices[i] && indices[i] < max)
        {
            atomicAdd(&tmp1[indices[i] - min].x, input[i].x);
            atomicAdd(&tmp1[indices[i] - min].y, input[i].y);
            atomicAdd(&tmp1[indices[i] - min].z, input[i].z);
            atomicAdd(&tmp1[indices[i] - min].cnt, 1);
        }
    }

    __syncthreads();


    if (id < indiceSize)
    {
        output[id].x = 0.0;
        output[id].y = 0.0;
        output[id].z = 0.0;
        red[id] = 0;
    }

    if (id < indiceSize && tmp1[threadIdx.x].cnt != 0)
    {
        output[id].x = tmp1[threadIdx.x].x / (float)tmp1[threadIdx.x].cnt;
        output[id].y = tmp1[threadIdx.x].y / (float)tmp1[threadIdx.x].cnt;
        output[id].z = tmp1[threadIdx.x].z / (float)tmp1[threadIdx.x].cnt;
        red[id] = 1;
    }

    return;
}


__global__ void reducePoints(pcl::PointXYZ *averaged, pcl::PointXYZ *filtered, int indiceSize, int *red, int *c_red)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;

    int index;
    for (int i = id; i < indiceSize; i+=stride)
    {
        if (red[i] == 1)
        {
            index = c_red[i];
            filtered[index].x = averaged[i].x;
            filtered[index].y = averaged[i].y;
            filtered[index].z = averaged[i].z;
        }
    }

    return;
}


__global__ void fillZero(int *c_red, int size)
{
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    int stride = blockDim.x * gridDim.x;

    for (int i = id; i < size; i+=stride)
    {
        c_red[i] = 0;
    }
}


int VoxelGridFilter::filterPoints(int size)
{
    float min[3], max[3];
    // std::cout << size << std::endl;


    checkCudaErrors(cudaMemcpy(tmp_input_, input_, sizeof(pcl::PointXYZ)*size, cudaMemcpyDeviceToDevice));
    // pcl::PointXYZ output;
    // checkCudaErrors(cudaMemcpy(&output, tmp_input_, sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost));
    // std::cout << "Input: " << output.x << ", " << output.y << ", " << output.z << std::endl;

    getMinMax3D(tmp_input_, size, min, max);

    checkCudaErrors(cudaThreadSynchronize());

    int num_x, num_y, num_z;
    num_x = (max[0] - min[0]) / leafsize_ + 1;
    num_y = (max[1] - min[1]) / leafsize_ + 1;
    num_z = (max[2] - min[2]) / leafsize_ + 1;

    int *indices;
    checkCudaErrors(cudaMalloc((void**)&indices, sizeof(int)*size));

    int block_x = (size > BLOCK_SIZE) ? BLOCK_SIZE : size;
    int grid_x = (size - 1) / block_x + 1;

    makeIndices<<<grid_x, block_x>>>(input_, size, indices, min[0], min[1], min[2], num_x, num_y, leafsize_);


    int indiceSize = num_x * num_y * num_z;


    pcl::PointXYZ *averaged;
    checkCudaErrors(cudaMalloc((void**)&averaged, sizeof(pcl::PointXYZ)*indiceSize));

    int *red;
    checkCudaErrors(cudaMalloc((void**)&red, sizeof(int)*indiceSize));
    checkCudaErrors(cudaMemset(red, 0, sizeof(int)*indiceSize));

    block_x = (indiceSize > BLOCK_SIZE) ? BLOCK_SIZE : indiceSize;
    grid_x = (indiceSize - 1) / block_x + 1;

    fillZero<<<grid_x, block_x>>>(red, indiceSize);

    block_x = (size > BLOCK_SIZE) ? BLOCK_SIZE : size;
    grid_x = (size - 1) / block_x + 1;

    averagePoints<<<grid_x, block_x, block_x * sizeof(Ave)>>>(indices, input_, averaged, size, indiceSize, red);


    thrust::device_ptr<int> dev_ptr(red);

    int *c_red;
    checkCudaErrors(cudaMalloc((void**)&c_red, sizeof(int)*(indiceSize + 1)));
    checkCudaErrors(cudaMemset(c_red, 0, sizeof(int)*(indiceSize + 1)));
    
    thrust::device_ptr<int> dev_c(c_red);

    thrust::exclusive_scan(dev_ptr, dev_ptr + indiceSize + 1, dev_c);
    checkCudaErrors(cudaDeviceSynchronize());


    int valid_num = *(dev_c + indiceSize);

    block_x = (indiceSize > BLOCK_SIZE) ? BLOCK_SIZE : indiceSize;
    grid_x = (indiceSize - 1) / block_x + 1;

    checkCudaErrors(cudaDeviceSynchronize());

    reducePoints<<<grid_x, block_x>>>(averaged, filtered_, indiceSize, red, c_red);

    checkCudaErrors(cudaDeviceSynchronize());

    // checkCudaErrors(cudaMemcpy(&output, filtered_, sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost));
    // std::cout << "Output: " << output.x << ", " << output.y << ", " << output.z << std::endl;

    checkCudaErrors(cudaFree(indices));
    checkCudaErrors(cudaFree(averaged));
    checkCudaErrors(cudaFree(red));
    checkCudaErrors(cudaFree(c_red));

    return valid_num; 
}


void VoxelGridFilter::freeFiltered()
{
    checkCudaErrors(cudaFree(filtered_));

    return;
}
