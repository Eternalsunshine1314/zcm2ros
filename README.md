# zcm2ros converter #

DEPENDENCIES:

1) ROS
2) OpenCV
3) ZCM
4) GeographicLib
5) minIni
 
HOW TO RUN:

1) ./build.sh
2) source devel/setup.bash
3) roslaunch zcm2ros zcm2ros.launch

TROUBLESHOOTING TIPS:

1) _Problems with dependencies installation_  
   Try to install them manually one by on with use of scripts in `zcm2ros/dep/*_buils.sh`.  
   The sequence of installation you may see in `zcm2ros/dep/build.sh`  
   
2) _Problems with ROS packages_  
   In case of errors with "Missing network_interface/rviz/tf_conversions" and etc. try to install them by `sudo apt-get install <your-ros-version>-<package>`.  
   For example for rviz installation with ROS Melodic run: `sudo apt-get install ros-melodic-rviz`  
   
3) _Problems with CV_LOAD_IMAGE_COLOR_  
   If there is a problem kind of "Invalid symbol: `CV_LOAD_IMAGE_COLOR`" or etc, you are probably trying to compile program againt opencv 3.0 (your opencv version installed is 3.0).  
   In this case replace `CV_LOAD_IMAGE_COLOR` by `cv::IMREAD_COLOR` in `zcm2ros/src/zcm2roc/src/converter.cpp`

4) _Problems with Java Virtual Machine_  
   In case when you have problems like:  
   ```
   Unrecognized option: -Xincgc (or something else)
   Error: Could not create the Java Virtual Machine
   ```
   First run `java -version`. If your java version is defferent than 8, you need to switch its usage by:  
   `sudo update-alternatives --config java`  
   and than from the list choose v.8 (because it's used in the project)
   
   Further when you need different java version simply switch its usage
