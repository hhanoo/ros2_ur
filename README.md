# SFSC_vision

## Environment

### Docker Environment
- ROS2 Humble
- Python 3.10
- CUDA 12.1 or higher

### Main Libraries
- PyTorch
- OpenCV
- PyRealSense2

### Hardware Requirements
- NVIDIA GPU (CUDA supported)
- Intel RealSense Camera


## Docker

### Image build
```bash
docker build -t ros2_ur_img -f Dockerfile.ros2 .
```
### Run
```bash
xhost +local:  # GUI 공유 허용 (한 번만 실행하면 됨)

docker run -it \
    --net=host \
    --gpus all \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v /mnt/ssd1/00_Project/ROS2/ros2_ur:/code \
    --name ros2_ur \
    ros2_ur_img
```