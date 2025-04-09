## Environment

### Docker Environment
- Ubuntu 22.04
- CUDA 12.5.1
- Python 3.10
- ROS2 Humble
- Gazebo Harmonic

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
docker build -t ros2:cuda12.5.1-cudnn-ubuntu22.04 -f Dockerfile.ros2 .
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
    -v ~/ros2_ur:/ros2_ws \
    --name ros2_ur \
    ros2:cuda12.5.1-cudnn-ubuntu22.04
```