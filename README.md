# 3D_Multi_Object_Dection_Dissertation

This work is part of Disseration work of Trinity College Dublin.It is used to detect 3D Objects from a LIDAR Input.

This work is inspired from the works of Bogoslavskyi and is forked from the open-source code found in the following link:

https://github.com/PRBonn/depth_clustering

Some improvements are done in the code and the results are compared with base standards to see the performance.


## Prerequisites ##

This requires Linux Operating system to run the code.
Desired versin is Ubuntu 16.04 or higher.

Catkin Workspace needs to be installed in Robot Operating system(ROS).

Installation of Robot Operating system (ROS) can be found in the folowing link:

http://wiki.ros.org/melodic/Installation/Ubuntu

There are various version of ROS avaialble for different Ubuntu packages.I suggest using melodic package with Ubuntu 16.04.
This software is developed and tested in the above mentioned version.

Installation of Catkin can be found in the following link:

http://wiki.ros.org/catkin

### Set up workspace and catkin ###

cd <catkin_ws>            # navigate to the workspace
pipenv shell --Obj_Det      # start a virtual environment(Optional)
pip install catkin-tools  # install catkin-tools for building
mkdir src                 # create src dir if you don't have it already
cd src			# Navigate to src
#Clone the repository
git clone https://github.com/ASHWIN2605/3D-Multi-Object-Dection_Dissetation.git

### System requirements ###

We will need OpenCV,Point cloud library(PCL),Eigen,QT4 or QT5 for building the application, QGLViewer for visualising the application and it can be installed with
the follwoing command in Ubuntu 16.04 version

```bash
sudo apt install libopencv-dev libqglviewer-dev libqt5-dev libeigen3-dev
```

PCL is optional in the build.If we have PCL instlled it will execute code to generate binary files for 
Euclidean measurement in clustering processing.


## How to run? ##
Since it is a catkin package.Once,the catkin workspace is set please follow the steps below to
build the application.

```bash
mkdir build
cd build
cmake ..
make -j4
```
## Run on real world data ##

Go to folder with binaries:
```
cd <path_to_project>/build/devel/lib/3D-Multi-Object-Dection_Dissetation
```

#### Frank Moosmann's "Velodyne SLAM" Dataset ####
Get the data:
```
mkdir data/; wget http://www.mrt.kit.edu/z/publ/download/velodyneslam/data/scenario1.zip -O data/moosmann.zip; unzip data/moosmann.zip -d data/; rm data/moosmann.zip
```

Run a binary to show detected objects:
```
./show_objects_moosmann --path data/scenario1/
```

This will run the application and the output will be dispalyed in a seperate terminal with object detected in each frame.









