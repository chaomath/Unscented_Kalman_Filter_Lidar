# **Unscented Kalman Filter Track for Lidar**

## Abstract
- Utilize sensor data only from  LIDAR  measurements for object (e.g. pedestrian, vehicles, or other moving objects) 
- Unscented Kalman Filter with CTRV model.

##  Run the code
- `mkdir build && cd build`
- `cmake .. && make` 
- `./UnscentedKF ../data/input.txt output.txt`
- `python ../plot/plotukf.py`

the last step will show the figure
![][image0] 

## Reference 
https://github.com/chaomath/tracking-with-Unscented-Kalman-Filter

[//]: # "Image References"
[image0]: ./images/ukf.png
