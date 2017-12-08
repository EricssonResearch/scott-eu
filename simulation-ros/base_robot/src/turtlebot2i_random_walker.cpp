/**
 * @file turtlebot2i_random_walker.cpp
 *
 * @brief A controller implementing a simple random walker algorithm
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_random_walker/LICENSE
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <string>
#include <boost/shared_ptr.hpp>
//#include "kobuki_random_walker/random_walker_controller.hpp"
#include "random_walker_controller.hpp"

//namespace kobuki
//{
//
///**
// * @brief Node-wrapper of the RandomWalkerController class
// */
//class RandomWalkerControllerNode
//{
//public:
//  RandomWalkerControllerNode(): shutdown_requested_(false){};
//  ~RandomWalkerControllerNode()
//  {
//    shutdown_requested_ = true;
//  }
//
//  /**
//   * @brief Initialise the node
//   *
//   * This function is called, when the node manager loads the node.
//   */
//  void init()
//  {
//    ros::NodeHandle nh_priv("~");
//
//    // resolve node name
//    std::string name = nh_priv.getUnresolvedNamespace();
//    int pos = name.find_last_of('/');
//    name_ = name.substr(pos + 1);
//
//    ROS_INFO_STREAM("Initialising node ... [" << name_ << "]");
//    controller_.reset(new RandomWalkerController(nh_priv, name_));
//
//    nh_priv.param("update_rate", update_rate_, 10.0);
//    ROS_INFO_STREAM("Controller will spin at " << update_rate_ << " hz. [" << name_ << "]");
//
//    // Initialises the controller
//    if (controller_->init())
//    {
//      update();
//    }
//    else
//    {
//      ROS_ERROR_STREAM("Couldn't initialise node! Please restart. [" << name_ << "]");
//    }
//  }
//
//private:
//  /// Pointer to the random walker controller
//  boost::shared_ptr<RandomWalkerController> controller_;
//  /// Spin rate for the update thread
//  double update_rate_;
//  /// Node(let) name
//  std::string name_;
//  /// Flag for stopping the update thread
//  bool shutdown_requested_;
//
//  /// Method for update thread
//  void update()
//  {
//    ros::Rate spin_rate(update_rate_);
//    while (ros::ok() && !shutdown_requested_)
//    {
//      controller_->spin();
//      spin_rate.sleep();
//
//    }
//  }
//};
//
//} // namespace kobuki

int main(int arg, char** argv) {

  ros::init(arg, argv,"turtlebot2i_random_walker");

  //boost::shared_ptr<kobuki::RandomWalkerControllerNode> walker(new kobuki::RandomWalkerControllerNode());
  ros::NodeHandle nh_priv("~");

  // resolve node name
  std::string name = nh_priv.getUnresolvedNamespace();
  int pos = name.find_last_of('/');
  name = name.substr(pos + 1);

  ROS_INFO_STREAM("Initialising node ... [" << name << "]");
  boost::shared_ptr<kobuki::RandomWalkerController> walker(new kobuki::RandomWalkerController(nh_priv, name));
  walker->init();

  ros::Rate spin_rate(10);

  while (ros::ok()) {
    walker->spin();
    spin_rate.sleep();
    ros::spinOnce();
  }
  //ros::spin();

  return 0;
}
