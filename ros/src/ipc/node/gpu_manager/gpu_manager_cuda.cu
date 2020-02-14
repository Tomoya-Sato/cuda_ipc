#include <gpu_manager/gpu_manager_cuda.hpp>

#define DSIZE 100000

inline void gassert(cudaError_t err_code, const char *file, int line)
{
    if (err_code != cudaSuccess) {
        fprintf(stderr, "Error: %s %s %d\n", cudaGetErrorString(err_code), file, line);
        cudaDeviceReset();
        exit(EXIT_FAILURE);
    }
}

#define checkCudaErrors(val) gassert(val, __FILE__, __LINE__)

GpuIpc::GpuIpc()
{
  bufCounter_ = 0;
}

GpuIpc::~GpuIpc()
{
}

unsigned char* GpuIpc::initGpuMemory()
{
  checkCudaErrors(cudaMalloc((void**)&data_, DSIZE*sizeof(pcl::PointXYZ)));

  cudaIpcMemHandle_t my_handle;
  unsigned char *handle_buffer;
  handle_buffer = (unsigned char*)malloc(sizeof(my_handle)+1);

  checkCudaErrors(cudaIpcGetMemHandle(&my_handle, data_));

  memset(handle_buffer, 0, sizeof(my_handle)+1);
  memcpy(handle_buffer, (unsigned char*)(&my_handle), sizeof(my_handle));

  return handle_buffer;
}

int GpuIpc::updateData()
{
  int ret = sizeBuffer_[0];

  checkCudaErrors(cudaMemcpy(data_, buf_[0], sizeBuffer_[0]*sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice));
  
  rotateBuffer();

  checkCudaErrors(cudaThreadSynchronize());

  return ret;
}

void GpuIpc::rotateBuffer()
{
  free(buf_[0]);

  for (int i = 0; i < BUF_SIZE-1; i++) {
      buf_[i] = buf_[i+1];
      sizeBuffer_[i] = sizeBuffer_[i+1];
  }
  bufCounter_ = bufCounter_ - 1;

  return;
}

void GpuIpc::storeBuffer(pcl::PointXYZ *tmp, int size)
{
 if (bufCounter_ == BUF_SIZE) rotateBuffer();
 
  buf_[bufCounter_] = (pcl::PointXYZ*)malloc(sizeof(pcl::PointXYZ)*size);

  memcpy(buf_[bufCounter_], tmp, sizeof(pcl::PointXYZ)*size);

  sizeBuffer_[bufCounter_] = size;

  bufCounter_++;

  return;
}

void GpuIpc::freeResource()
{
  checkCudaErrors(cudaFree(data_));
  
  return;
}
