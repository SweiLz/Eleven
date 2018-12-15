# Eleven


## Eleven robot dependencies
```
$ cd ~/<catkin_ws>/src
$ git clone https://github.com/AnmanTechnology/anman_sensors.git
$ cd ~/<catkin_ws> && catkin_make
```
### SLAM GMAPPING

this repo improve gmapping to multi-thread processing
```
$ git clone https://github.com/vrabaud/slam_gmapping.git
$ git clone https://github.com/eybee/openslam_gmapping.git
```

## Eleven gazebo simulation

![](https://github.com/SweiLz/Eleven/blob/master/docs/images/ele_gazebo.png?raw=true)
![](https://github.com/SweiLz/Eleven/blob/master/docs/images/ele_gazebo_wg.png?raw=true)




## Test hector slam mapping from bag file

Download bag file and put into eleven_slam/bags: 

* [test_mapping.bag](https://drive.google.com/file/d/1BhjfsTNKNU-KGuzlmm7wTWL_7RYnmVUL/view?usp=sharing) (~336mb)

```
$ roslaunch eleven_slam test_mapping_bagfile.launch
```

![](https://github.com/SweiLz/Eleven/blob/master/docs/images/wg_map_slam.jpeg?raw=true)




