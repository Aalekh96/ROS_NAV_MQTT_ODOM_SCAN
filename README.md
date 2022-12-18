# ROS_NAV_MQTT_ODOM_SCAN

The first file is named as Mobile description which is a ros package made to test the communication. 
  Step_1 -- To use the package, first run the launch file named, which will open the gazebo
              >> roslaunch mobile_robot_description main.launch <<
              
  Step_2 --  AMCL FILE
              ** roslaunch mobile_robot_description amcl_plus_map_server.launch <<
  
  Step_3 --  Rviz
              ** rviz  (type in terminal)<<
              
  Step_4 --  Move_base node
              ** roslaunch nav_mybot move_base.launch<<
              
This setps will open the all the thinks required to run the single robot with ros. 


To open the communication part, 
  -- There is a package named mqtt_client, which connects to mqtt broker on which i have installed on aws server
      >>> roslaunch mqtt_client standlone.launch
      
      ** Remember to change the ip address of the broker alon with port, username and password. 
  
  
