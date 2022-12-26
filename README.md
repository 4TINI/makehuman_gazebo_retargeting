# makehuman_gazebo_retarget


<div align="center">
    <img src=".github/camera.png" height = 200>
    <img src=".github/gazebo.jpg" height = 200>
</div>


To spawn the camera:
```bash
roslaunch generic_gazebo_camera_plugin spawn_camera.launch
```

| Argument | Default | Values | Description |
| ---| --- | --- | --- |
| `world_frame` | world |  |  |
| `camera_name` | camera |  |  |
| `camera_frame` | $(arg camera_name)_base_link |  |  |
| `x` | 0 | double (meters) |  |
| `y` | 0 | double (meters) |  |
| `z` | 0 | double (meters) |  |
| `roll` | 0 | double (rad) |  |
| `pitch` | 0 | double (rad) |  |
| `yaw` | 0 | double (rad) |  |
| `model_path` |  |  |  |
| `params_path` |  |  |  |





`sudo gedit /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py`
```
_TIMEOUT_SIGINT  = 0.5 #seconds
_TIMEOUT_SIGTERM = 0.5 #seconds
```

# volendo si può aggiungere
sudo cp libActorCollisionsPlugin.so /usr/lib/x86_64-linux-gnu/gazebo-9/plugins

collisioni funzionano solo con debug mode on. no nso perchè

