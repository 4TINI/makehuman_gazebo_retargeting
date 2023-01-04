#!/usr/bin/env bash

# usage bash retarget_human.sh -m "makehuman_file" -b "bvh_file" -n "output_name"
# usage e.g. bash retarget_human.sh -m demo_actor.mhx2 -b Mixamo/GettingUpFromBack.fbx -n test
# usage e.g. bash retarget_human.sh -m demo_actor.mhx2 -b Optitrack/Running.bvh -n test

output_name="test"

ws_path=$(catkin locate)

while getopts m:b:n: flag; do
    case "${flag}" in
        m) mhx2_file=${OPTARG};;
        b) bvh_file=${OPTARG};;
        n) output_name=${OPTARG};;
    esac
done

bvh_dataset_path=$ws_path'/src/makehuman_gazebo_retargeting/datasets/'
bvh_file_path=$bvh_dataset_path$bvh_file

mhx2_dataset_path=$ws_path'/src/makehuman_gazebo_retargeting/makehuman/'
mhx2_file_path=$mhx2_dataset_path$mhx2_file

blender ../../blender/empty.blend --python retarget_human.py -- $mhx2_file_path $bvh_file_path $output_name 

echo "Done"