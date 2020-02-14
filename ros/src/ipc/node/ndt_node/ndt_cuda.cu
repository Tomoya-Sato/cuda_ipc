#include <ndt_node/ndt.hpp>


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


NdtNode::NdtNode()
{
  empty_handle_ = true;
}

NdtNode::~NdtNode()
{
}

void NdtNode::getHandle(unsigned char *handle)
{
  if (empty_handle_)
  {
    cudaIpcMemHandle_t my_handle;

    memcpy((unsigned char*)&my_handle, handle, sizeof(my_handle));
    checkCudaErrors(cudaIpcOpenMemHandle((void**)&input_, my_handle, cudaIpcMemLazyEnablePeerAccess));

    empty_handle_ = false;
  }
}

void NdtNode::publishPoints(int size, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr)
{
  pcl::PointXYZ *data = (pcl::PointXYZ*)malloc(sizeof(pcl::PointXYZ)*size);

  checkCudaErrors(cudaMemcpy(data, input_, sizeof(pcl::PointXYZ)*size, cudaMemcpyDeviceToHost));

  for (int i = 0; i < size; i++)
  {
    filtered_scan_ptr->points.push_back(data[i]);
  }
}
