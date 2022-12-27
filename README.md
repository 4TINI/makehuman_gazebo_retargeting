# makehuman_gazebo_retarget


<div align="center">
    <img src=".github/camera.png" height = 200>
    <img src=".github/gazebo.jpg" height = 200>
</div>






`sudo gedit /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py`
```
_TIMEOUT_SIGINT  = 0.5 #seconds
_TIMEOUT_SIGTERM = 0.5 #seconds
```

# volendo si può aggiungere
sudo cp libActorCollisionsPlugin.so /usr/lib/x86_64-linux-gnu/gazebo-9/plugins

collisioni funzionano solo con debug mode on. no nso perchè

controlla in retarget gli fps con lìoptitrack
