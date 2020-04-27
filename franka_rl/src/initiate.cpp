#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <std_msgs/Float64.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <math.h>
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
double w = 0;

void jointCallback(const sensor_msgs::JointState::ConstPtr& state) {
  double pos1 = state->position[0];
  double pos2 = state->position[1];
  w = pos1 + pos2;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  ros::Publisher pub = node_handle.advertise<std_msgs::Float64>("/franka/panda_joint7_controller/command", 10);
  ros::Publisher grip = node_handle.advertise<std_msgs::Float64>("/franka/gripper_width", 100);
  ros::Publisher dn = node_handle.advertise<std_msgs::Bool>("/done_init", 10);
  ros::Subscriber sub = nh.subscribe("/franka/joint_states", 1000, jointCallback);
  spinner.start();
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  geometry_msgs::PoseStamped target_pose1;




  std_msgs::Float64 msg;
  std_msgs::Float64 width;
  std_msgs::Bool ifDone;
  msg.data = 0.785;
  width.data = 0.06;
  std::chrono::time_point<std::chrono::system_clock> start, end;
  ros::Rate loop_rate(100);
  int flag = 1;
  while (ros::ok()){
    boost::shared_ptr<std_msgs::Bool const> sharedPtr = NULL;
    while(sharedPtr == NULL){
      sharedPtr = ros::topic::waitForMessage<std_msgs::Bool>("/make_init", nh);
    }
    std_msgs::Bool t = *sharedPtr;
    bool a = t.data;
    if (a){
      ROS_INFO("Moving to initial position...");
      usleep(2000000);
      move_group.setNamedTarget("initial");
      move_group.move();
      pub.publish(msg);

      usleep(1000000);


      int i = 0;
      double time = 0;
      ifDone.data = true;
      usleep(1000000);
      dn.publish(ifDone);
      flag = 0;
    }
    loop_rate.sleep();
  }

  ROS_INFO("Done");
  ros::shutdown();
  return 0;
}
