# hitachi_ipc

## 動作環境
* Ubuntu 16.04
* CUDA 8.0
* Intel Core i7-8700
* GeForce GTX 1080
* RAM 32GB

## 手順
### ビルド
```
$ cd hitachi_ipc/ros
$ rosdep update
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 実行
#### Voxel Grid Filterのみ動かす場合
1. ターミナルを３つ起動する
2. 全ての端末で`hitachi_ipc/ros/devel/setup.bash`を`source`で読み込む
```
$ source /path/to/hitachi_ipc/ros/devel/setup.bash
```
3. １つ目の端末(端末１)で`gpu_manager`を起動する。生点群をGPUに転送し、生点群を必要とするノードと共有する
```
$ rosrun ipc gpu_manager
```
4. ２つ目の端末(端末２)で`voxelgridfilter`を起動する。GPU上の生点群をダウンサンプルし、ダウンサンプルされた点群を必要とするノードと共有する
```
$ rosrun ipc voxelgridfilter
```
5. この段階で`gpu_manager`と`voxelgridfilter`が通信を行いGPUメモリを共有する。端末１では`Recieved ready`、端末２では`Published ready`と表示されていれば成功
6. ３つ目の端末(端末３)で`ndt_node`を起動する。GPU上のダウンサンプルされた点群をCPUに転送し`sensor_msgs::PointCloud2`型のトピック`/filtered_points`としてPublishする
7. 端末２では`Recieved ready`、端末３では`Published ready`と出力されていれば成功

#### Autowareと連携させ自己位置推定を行う場合
手順5までは同様
6. [distributed-autoware](https://github.com/pflab-ut/distributed-autoware)の`feature/cuda_ipc`ブランチをビルドする
```
$ git clone https://github.com/pflab-ut/distributed-autoware
$ cd distributed-autoware
$ git checkout feature/cuda_ipc
$ cd ros
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$ ./catkin_make_release
```
7. distributed-autowareを起動する
```
$ ./run
```
8. voxelgridfilterを除いて通常のndt_matchingを行う際と同様にノードを起動する。ndt_matchingの`app`で`pcl_anh_gpu`を選択する。
9. ndt_matchingを起動した段階で端末２に`Recieved ready`、runtime_managerの出力端末に`Published ready`と出力されていれば成功
10. 以降は通常のndt_matchingを実行する際と同様
