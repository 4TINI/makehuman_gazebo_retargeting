#!/usr/bin/env bash

echo "Installing MakeHuman"
# Install Makehuman
sudo add-apt-repository ppa:makehuman-official/makehuman-community
sudo apt-get update
sudo apt-get install makehuman-community

echo "Downloading MakeHuman Plugins"
# MakeHuman Api Plugin
wget -O ~/Downloads/community-plugins-mhapi-master.zip https://github.com/makehumancommunity/community-plugins-mhapi/archive/refs/heads/master.zip 

# Socket Plugin
wget -O ~/Downloads/community-plugins-socket-master.zip https://github.com/makehumancommunity/community-plugins-socket/archive/refs/heads/master.zip 

# MassProduce Plugin
wget -O ~/Downloads/community-plugins-massproduce-master.zip https://github.com/makehumancommunity/community-plugins-massproduce/archive/refs/heads/master.zip

# Unzip Plugins and move in the Makehuman installation path
unzip ~/Downloads/community-plugins-mhapi-master.zip community-plugins-mhapi-master/1_mhapi/* -d ~/tmp 
sudo mv ~/tmp/community-plugins-mhapi-master/1_mhapi /usr/share/makehuman-community/plugins/1_mhapi

unzip ~/Downloads/community-plugins-socket-master.zip community-plugins-socket-master/8_server_socket/* -d ~/tmp   
sudo mv ~/tmp/community-plugins-socket-master/8_server_socket /usr/share/makehuman-community/plugins/8_server_socket

unzip ~/Downloads/community-plugins-massproduce-master.zip community-plugins-massproduce-master/9_massproduce/* -d ~/tmp  
sudo mv ~/tmp/community-plugins-massproduce-master/9_massproduce /usr/share/makehuman-community/plugins/9_massproduce

ws_path=$(catkin locate)

# add the open-vico skeleton
sudo cp $ws_path/src/makehuman_gazebo_retargeting/makehuman/open_vico* /usr/share/makehuman-community/data/rigs/

echo "Done"