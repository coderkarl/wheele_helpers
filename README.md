# wheele_helpers

## ROS bag to csv data
Find the wheele_bag_inputs_to_csv.py script in the wheele/ros_vehicle_model/scripts folder
Convert bag to csv files. Existing csv will be overwritten
Run from the bag folder.
You can also run in a folder of bag folders and all nested bag files will be processed.
This is a custom csv creation that gets encoder, gyros, gps, and compass data to be used by the Kalman Filter octave script
```
roscore
cd <path to bags folder>
rosrun ros_vehicle_model wheele_bag_inputs_to_csv.py
```

## Offboard Kalman Filter using octave
Install octave in linux if necessary
This m script steps through the wheele csv data and is useful for tuning sensor fusion.
