vrep_ros_control_example
========================

ros_control robot-specific back end for vrep (i.e. ~ gazebo_ros_control but for vrep and for a specific robot)

INSTRUCTIONS:

In order to create a ros_control back end for vrep:
   * obviously you need a .ttt vrep file corresponding to your simulated system
   * you need to modify you hardware interface file src/ControlLoop/MyRobot_vrepHW.cpp according to the kinematics of your own robot,
      - the other files like vrepControl_plugin .h or .cpp and vrepControl_server .h or .cpp are generic and should not be changed
      - files src/v_rep* are directly copied from vrep (and needed to create a plugin), they should not be changed
   * unfortunately you need a redundant description of your robot in urdf (so your robot is described both in the .tt file and in the urdf)
      - this is because the urdf contains information on the joints and transmissions not contained in the vrep .ttt file
      - to counterbalance this point, note that the urdf importer works pretty well. But once your urdf is imported and you modify something in vrep's .ttt file you need to redundantly modify it in the urdf (I told Coppelia about this situation but unfortunately did not receive an answer)
   * this plugin is based on the blank ros skeleton plugin vrep_plugin_skeleton provided in vrep's folder programming/ros_packages/vrep_plugin_skeleton

HOW TO RUN IT
   * once built, you need to copy this plugin file libv_repExtRosControl.so from your <workspace>/build/lib folder to vrep's main folder (along all the other plugins)
   * start a terminal with roscore
   * then start vrep and load vrep/ros_control.ttt
      - check that plugin libv_repExtRosControl.so is correctly loaded in vrep's trace (i.e. in vrep's console)
   * and launch file vrep.launch
      - roslaunch vrep_ros_control_example vrep.launch
   * start a follow_joint_trajectory action so as to give a goal to your controller
      - roscd vrep_ros_control_example
      - cd action
      - ./test_move.py follow_joint_trajectory:=mr_controller/follow_joint_trajectory

Please look at the code if you want to understand how the plugin works, it should be self explaining for people who know well about vrep and ros_control, and it is well commented for the others. The main "trick" is in function "bool ROS_server::initialize()" in file vrepControl_server.cpp.

In case of trouble feel free to contact me.

Thanks,

Antoine Rennuit

antoinerennuit@hotmail.com