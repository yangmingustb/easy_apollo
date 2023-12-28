locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update 
sudo apt install curl gnupg lsb-release

sudo curl -sSL http://packages.ros.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


sudo apt update
sudo apt upgrade

sudo apt install ros-iron-desktop-full


source /opt/ros/iron/setup.bash
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

#如果出现catkin错误，可以参考
#https://huaweicloud.csdn.net/637ef032df016f70ae4ca34c.html