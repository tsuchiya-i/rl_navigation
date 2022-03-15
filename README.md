# rl_navigation
 ROS navigation system based on deep reinforcement learning
 
## Python
Python version 2.7.
## Requirement
- tensorflow
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)
- [rtabmap_ros](https://github.com/introlab/rtabmap_ros)

## Installing
```
git clone 
```
or
```
git clone https://github.com/openai/gym
cd gym
pip install -e .
```

## Installing OpenCV
(recommended)
```
$ sudo apt-get install python3-opencv
```
or
```
$ pip3 install opencv-python
```

## Installing TensorFlow
```
$ pip3 install tensorflow
```
If you want to use a GPU, please check these versions of TensorFlow, CUDA, and NVIDIA-Drivers and install them.([TensorFlow & CUDA](https://www.tensorflow.org/install/source?hl=ja#tested_build_configurations))([CUDA & Driver](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html))

## Installing SS2D
```
$ git clone https://github.com/tsuchiya-i/SS2D.git
$ cd SS2D
$ pip3 install -e .
```
# Demo
![demo](https://github.com/tsuchiya-i/SS2D/blob/main/appendix/navigation_sample.gif)

# Note
#### Error code1:
```
ModuleNotFoundError: No module named 'skbuild'
```
Then run this command
```
pip3 install -U pip
```
#### Error code2:
```
ModuleNotFoundError: No module named 'tkinter'
```
Then run this command
```
sudo apt-get install python3-tk
```
#### error code3:
```
ModuleNotFoundError: No module named 'PIL.ImageTk'
```
Then run this command
```
sudo apt install python3-pil.imagetk
```
