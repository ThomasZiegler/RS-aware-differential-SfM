Rolling-Shutter Aware Differential Structure from Motion and Image Rectification
=============
![alt text](https://github.com/ThomasZiegler/RS-aware-differential-SfM/blob/master/images/algorithm_overview.png)

C++ implementation of the work proposed by Zhuang et al. [1]. This work was done as part of the ETH 3D Vision course 252-0579-00L in 2018.

The image above shows an overview of the algorithm.
1. Starting from two consecutive rolling shutter frames.
2. The flow gets extracted using Deep Flow [2].
3. Using the flow, an RS-aware depth map and relative camera poses are estimated. 
4. With the known poses for each scanline the image can reprojected into a global shutter image frame for rectification.


## Authors
* Manuel Fritsche [manuelf at ethz.ch]
* Felix Graule [graulef at ethz.ch]
* Thomas Ziegler [zieglert at ethz.ch]

#### Supervisor
* Dr. Oliver Saurer [saurero at ethz.ch]


## Dependencies
The code has been developed and tested with the following versions of packages. Ohter versions might work as well, but have not been tested! 

### Boost - Version 1.58.0 
Boost is only used to automatically create a new folder for each test case. On Ubuntu use:
```
$ sudo apt install libboost-all-dev
```

### Ceres Solver - Version 1.14.0
Ceres Solver is used for depth estimation and nonlinear refinement. 

An installation guide can be found here: http://ceres-solver.org/installation.html.

### Eigen - Version 3.3.4
On Ubuntu use: 
```
$ sudo apt install libeigen3-dev
```
Pay attention if you use Ubuntu 16.04 LTS and the libeigen3-dev package. Eigen version 3.3~beta1-2 can create segmentaion faults when using Ceres. In this case you need to build it by yourself.

### Open CV - Version 3.4.0
For the use of Deep-Flow it is important that beside the main package also the extra modules (https://github.com/opencv/opencv_contrib) are installed. 

It is best to build OpenCV yourself to avoid mismatches between main package and extra modules.
```
$ cd /path/to/your/workspace
$ git clone https://github.com/opencv/opencv.git
$ git clone https://github.com/opencv/opencv_contrib.git
$ cd opencv_contrib/
$ git checkout origin/3.4
$ cd ../opencv/
$ git checkout origin/3.4
$ mkdir build
$ cd build/
$ cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ../ 
$ make -j4
$ sudo make install
```

## Reference
[1] B. Zhuang, L. F. Cheong, and G. H. Lee, “Rolling-Shutter-Aware Differential SfM and Image Rectification,” in 2017 IEEE International Conference on Computer Vision (ICCV), 2017, pp. 948–956.

[2] P. Weinzaepfel, J. Revaud, Z. Harchaoui, and C. Schmid, “DeepFlow: Large Displacement Optical Flow with Deep Matching,” in 2013 IEEE International Conference on Computer Vision, 2013, pp. 1385–1392.

## License
The source code is released under the [GNU General Public License](./LICENSE).
