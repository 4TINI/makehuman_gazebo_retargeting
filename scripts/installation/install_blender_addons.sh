#!/usr/bin/env bash

#mhx2 Makehuman Plugin
wget -O ~/Downloads/mhx-blender-latest.zip http://download.tuxfamily.org/makehuman/plugins/mhx-blender-latest.zip

# Rokoko plugin for blender
wget -O ~/Downloads/rokoko-studio-live-blender-master.zip https://github.com/Rokoko/rokoko-studio-live-blender/archive/refs/heads/master.zip

# Pip Install Blender Manager
wget -O ~/Downloads/blender_pip-master.zip https://github.com/amb/blender_pip/archive/refs/heads/master.zip

ws_path=$(catkin locate)

blender --python $ws_path/src/makehuman_gazebo_retargeting/scripts/installation/install_blender_addons.py

echo "Done"