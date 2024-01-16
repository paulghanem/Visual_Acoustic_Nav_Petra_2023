#!/bin/bash -i

echo "Upgrading pip to latest version for installations."
pip install --upgrade pip > /dev/null

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

echo "Installing ROS Noetic."
sudo apt install ros-noetic-desktop-full

echo "Setting up ROS environment."
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
. ~/.bashrc

echo "Making the catkin workspace."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build

# Install dependencies
echo "Installing Python dependencies."
pip install -r ~/installation_scripts/requirements.txt

cd ~/installation_scripts
echo "Downloading Intel OpenVINO Runtime Toolkit."
wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18617/l_openvino_toolkit_p_2022.1.0.643_offline.sh > /dev/null
chmod +x l_openvino_toolkit_p_2022.1.0.643_offline.sh
echo "Installing Intel OpenVINO Runtime Toolkit."
./l_openvino_toolkit_p_2022.1.0.643_offline.sh -a --cli -s --eula accept > /dev/null
cd ~/intel/openvino_2022/install_dependencies
echo "Installing external software dependencies for the Runtime Toolkit."
sudo -E ./install_openvino_dependencies.sh > /dev/null
echo "Installing additional intel development tools."
pip3 install -r ~/intel/openvino_2022/tools/requirements.txt > /dev/null
echo "Installing OpenVINO appropriate OpenCV version."
~/intel/openvino_2022/extras/scripts/download_opencv.sh > /dev/null
cd


echo "Adding environment variables to define locations of various directories."
echo -e "\nexport PATH_TO_MODULES='$HOME/vz_modules'" >> ~/.bashrc
echo -e "export VZ_MODEL_DIR='$HOME/vz_models'" >> ~/.bashrc
echo -e "export VZ_BAG_DIR='$HOME/vz_bags'" >> ~/.bashrc
echo -e "export VZ_REG_DIR='$HOME/vz_registration'" >> ~/.bashrc
echo -e "export CATKIN_DIR='$HOME/catkin_ws'" >> ~/.bashrc
echo "Adding line to set Intel's environment variables to bashrc."
echo -e "\n# Set Intel's environment variables " >> ~/.bashrc
echo -e "source ~/intel/openvino_2022/setupvars.sh" >> ~/.bashrc

echo "Sourcing the bashrc with new changes"
. ~/.bashrc


cd "$CATKIN_DIR"/src
git clone --quiet https://github.com/VZ-Project/stretch_ros.git
echo "Cloning leg_tracker repository into catkin_ws."
git clone --quiet https://github.com/VZ-Project/leg_tracker.git > /dev/null
echo "Cloning vision_opencv repository into catkin_ws."
git clone --quiet https://github.com/VZ-Project/vision_opencv.git > /dev/null
echo "Cloning vz_ros_packages repository into catkin_ws."
git clone --quiet https://github.com/VZ-Project/vz_ros_packages.git > /dev/null
echo "Cloning third_party_sensors repository into catkin_ws."
git clone --quiet https://github.com/VZ-Project/third_party_sensors.git > /dev/null


cd
echo "Creating a directory for the modules."
mkdir "$PATH_TO_MODULES"
cd "$PATH_TO_MODULES"

echo "Cloning the face_recognition module into the modules directory."
git clone --quiet https://github.com/VZ-Project/face_recognition.git > /dev/null
cd face_recognition
git checkout vz_deliverables > /dev/null
cd ..

echo "Cloning the pedestrian_tracker module into the modules directory."
git clone --quiet https://github.com/VZ-Project/pedestrian_tracker.git > /dev/null
cd pedestrian_tracker
git checkout vz_deliverables > /dev/null
cd ..

echo "Cloning the acoustic_scene_analysis into the modules directory."
git clone --quiet https://github.com/VZ-Project/acoustic_scene_analysis.git > /dev/null
cd acoustic_scene_analysis
git checkout vz_deliverables > /dev/null
cd ..

echo "Cloning the in_bed_pose_estimation into the modules directory."
git clone --quiet https://github.com/VZ-Project/in_bed_pose_estimation.git > /dev/null
cd in_bed_pose_estimation
git checkout vz_deliverables > /dev/null
cd ..

echo "Cloning the optical_remote_vital_sensing into the modules directory."
git clone --quiet https://github.com/VZ-Project/optical_remote_vital_sensing.git > /dev/null
cd optical_remote_vital_sensing
git checkout vz_deliverables > /dev/null
cd

echo "Cloning the visual_body_pose_recognition into the modules directory."
git clone --quiet https://github.com/VZ-Project/visual_body_pose_recognition.git > /dev/null
cd visual_body_pose_recognition
git checkout vz_deliverables > /dev/null
cd


echo "Creating a directory for the pedestrian_tracker and face_recognition models."
mkdir "$VZ_MODEL_DIR"
cd "$VZ_MODEL_DIR"
mkdir -p pedestrian_tracker
mkdir -p face_recognition
cp "$PATH_TO_MODULES"/pedestrian_tracker/pedestrian_tracker_demo/cpp/models.lst "$VZ_MODEL_DIR"/pedestrian_tracker
cp "$PATH_TO_MODULES"/face_recognition/face_recognition_demo/python/models.lst "$VZ_MODEL_DIR"/face_recognition
cd pedestrian_tracker
echo "Downloading the pedestrian_tracker models."
omz_downloader --list "$VZ_MODEL_DIR"/pedestrian_tracker/models.lst > /dev/null
cd ../face_recognition
echo "Downloading the face_recognition models."
omz_downloader --list "$VZ_MODEL_DIR"/face_recognition/models.lst > /dev/null
cd


echo "Creating a directory for registration data for the face_recognition and acoustic_scene_analysis modules."
cd
mkdir -p "$VZ_REG_DIR"/asa_reg
cd "$VZ_REG_DIR"
mkdir face_recognition_reg

echo "Creating a directory for ROS bagged data."
mkdir -p "$VZ_BAG_DIR"

# Structure Core SDK
echo "Setting up Structure Core SDK."
echo "Creating udev rules for non root users."
cd "$CATKIN_DIR"/src/third_party_sensors/structure_core/StructureSDK-CrossPlatform-0.9/DriverAndFirmware/Linux
chmod +x Install-CoreDriver-Udev-Linux.sh
sudo ./Install-CoreDriver-Udev-Linux.sh > /dev/null

echo "Installing firmware for the Structure Core Camera."
chmod +x CoreFirmwareUpdater-1.2.0-Linux-x86_64
sudo ./CoreFirmwareUpdater-1.2.0-Linux-x86_64 > /dev/null

echo "Building the Structure Core SDK."
cd ../../Scripts/
chmod +x build.sh
./build.sh > /dev/null

echo "Changing the Structure Core SDK permissions."
cd ../ROS/ros1/cfg/
chmod +x SCParams.cfg

echo "Building the Structure Core SDK ros package."
cd ../..
chmod +x to_ros1.sh
. to_ros1.shd +x build.sh
./build.sh > /dev/null

cd $CATKIN_DIR
echo "Install any necessary ROS package dependencies."
rosdep install --from-paths src --ignore-src -r -y 
catkin build

echo "Adding command to source catkin_ws to bashrc and sourcing the bashrc."
echo -e "\n# Source catkin environment" >> ~/.bashrc
echo -e "source $CATKIN_DIR/devel/setup.bash" >> ~/.bashrc

. ~/.bashrc
