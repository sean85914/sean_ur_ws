#include <ros/ros.h>
#include <ur_modern_driver/pose_control.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ur_kin/ur_kin.h>

// Gripper length
// 0.16 meter for robotiq
const double tcp_length = 0.22;
// Maxmimum joint speed
const double joint_speed = 3.0;

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm {
 private:
  TrajClient* traj_client_;
  control_msgs::FollowJointTrajectoryGoal goal_;  
  ros::NodeHandle nh_;
  ros::Subscriber sub_joint_state_;
  ros::ServiceClient client_;
  
  double joint_[6]; // joint state
  
  // Check if the joint is valid
  // Input: joint position
  // Output: valid joint position
  // if absolute value of angle greater than 2pi
  // then angle <- mod(angle, 2pi)
  // if angle < -pi
  // then angle <- angle + 2pi
  // else if angle > pi
  // then angle <- angle - 2pi
  
  double validAngle(double angle) {
    if (abs(angle) > 2*M_PI) angle = fmod(angle, 2*M_PI);
    if (angle < -M_PI) return 2*M_PI + angle;
    else if (angle > M_PI) return angle - 2*M_PI;
    else return angle;
  }

  // sub_joint_state_ callback function
  // change the value of joint_[6]
  void jointStateCallback(const sensor_msgs::JointState &msg) {
    for (int i = 0; i < 6; ++i)
      joint_[i] = msg.position[i];
  }

  // Calculate the DH matrix from given pose
  // Input: pose(position& orientation), T(matrix that save the result)
  // Output: None
  void pose_to_DH(geometry_msgs::Pose pose, double *T) {
    geometry_msgs::Point &p = pose.position;
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double sinr = sin(roll), cosr = cos(roll);
    double sinp = sin(pitch), cosp = cos(pitch);
    double siny = sin(yaw), cosy = cos(yaw);

    // DH matrix, ZYX convention
    // https://en.wikipedia.org/wiki/Euler_angles
    // Tait-Bryan angle, ZYX
    // Map tool0 back to ee_link
    T[0] = cosy*cosp;
    T[1] = cosy*sinp*sinr - cosr*siny;
    T[2] = siny*sinr + cosy*cosr*sinp;
    T[3] = p.x;
    T[4] = cosp*siny;
    T[5] = cosy*cosr + siny*sinp*sinr;
    T[6] = cosr*siny*sinp - cosy*sinr;
    T[7] = p.y;
    T[8] = -sinp;
    T[9] = cosp*sinr;
    T[10] = cosp*cosr;
    T[11] = p.z;
    T[15] = 1;
  }
  // Perform IK calculation
  // Input: target_pose(pose that ee_link going to be), sol(joint position container)
  // Output: None
  void perform_IK(geometry_msgs::Pose target_pose, double *sol) {
    double T[16] = {0};
    // TCP's DH Matrix
    pose_to_DH(target_pose, T);
    // ee_link's DH Matrix
    // tcp_link2base_link = ee_link2base_link * tcp_link2ee_link
    // ee_link2base_link = tcp_link2base_link * (tcp_link2ee_link)^-1
    // base_link2tcp_link = T
    // tcp_link2ee_link = [1 0 0 tcp_length; 0 1 0 0; 0 0 1 0; 0 0 0 1]
    T[3] -= tcp_length*T[0];
    T[7] -= tcp_length*T[4];
    T[11] -= tcp_length*T[8];

    double q_sols[8*6], min = 1e6, dist = 0;
    int num_sols = ur_kinematics::inverse(T, q_sols), index = 0;
    for (int i = 0; i < num_sols; ++i) {
 
      for (int j = 0; j < 6; ++j) {
        q_sols[i*6 + j] = validAngle(q_sols[i*6 + j]);
        dist += pow(q_sols[i*6 + j] - joint_[j], 2);
      }
      // Take the nearest one
      if (min > dist) {
        min = dist;
        index = i;
      } dist = 0;
    }

    for (int i = 0; i < 6; ++i)
      sol[i] = q_sols[index*6 + i];
  }
 
 public:
  RobotArm() : joint_() {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("/follow_joint_trajectory", true);

    // wait for action server to come up
    while (!traj_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the joint_trajectory_action server");
      break;
    }
    // Subscribe to /joint_state
    sub_joint_state_ = nh_.subscribe("/joint_states", 1, &RobotArm::jointStateCallback, this);

    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    t.joint_names.resize(6);
    t.joint_names[0] = "shoulder_pan_joint";
    t.joint_names[1] = "shoulder_lift_joint";
    t.joint_names[2] = "elbow_joint";
    t.joint_names[3] = "wrist_1_joint";
    t.joint_names[4] = "wrist_2_joint";
    t.joint_names[5] = "wrist_3_joint";
  }
  ~RobotArm() {
    delete traj_client_;
  }
  // Start to execute the action
  // Input: goal
  // Output: None
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal) {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now();
    traj_client_->sendGoal(goal);
  }
  
  // Let arm to desired pose
  // Input: pose
  // Output: goal
  control_msgs::FollowJointTrajectoryGoal armToDesiredPoseTrajectory(geometry_msgs::Pose pose) {
    ros::spinOnce();
    double sol[6] = {0};
    perform_IK(pose, sol);
    trajectory_msgs::JointTrajectory &t = goal_.trajectory;
    t.points.resize(2);
    for (int i = 0; i < 2; ++i) {
      t.points[i].positions.resize(6);
      t.points[i].velocities.resize(6);
    }

    t.points[0].time_from_start = ros::Duration(0);
    // Added by Sean.
    // Calculate the execution time
    double dist = 0;
    for(int i=0; i<6;++i){
      dist += pow(sol[i]-joint_[i], 2);
    }
    dist = sqrt(dist);
    int cost_time = ceil(dist/ joint_speed)* 3; // Multiply 3 to slow down
    t.points[1].time_from_start = ros::Duration(cost_time);

    for (int i = 0; i < 6; ++i) {
      t.points[0].positions[i] = joint_[i];
      t.points[0].velocities[i] = 0;
      t.points[1].positions[i] = sol[i];
      t.points[1].velocities[i] = 0;
    }
    return goal_;
  }
  // Get execution state
  // Input: None
  // Output: state
  actionlib::SimpleClientGoalState getState() {
    ros::spinOnce();
    return traj_client_->getState();
  }
  // Added by Sean
  // Input: pose
  // Output: 1 if the pose is in the working area and 0 otherwise
  bool getNumofSols(geometry_msgs::Pose pose) {
    double T[16] = {0};
    pose_to_DH(pose, T);
    T[3] -= tcp_length*T[0];
    T[7] -= tcp_length*T[4];
    T[11] -= tcp_length*T[8];

    double q_sols[8*6];
    int num_sols = ur_kinematics::inverse(T, q_sols);
    return (num_sols >= 1);
  }
}; // end class definition  

bool motion(ur_modern_driver::pose_control::Request &req,
	    ur_modern_driver::pose_control::Response &res)
{
  geometry_msgs::Pose pose; // target pose
  pose = req.target;
  
  RobotArm arm;
  ros::Time now = ros::Time::now();

  while(ros::Time::now().toSec() < (now.toSec()+3)) {
    ros::spinOnce();
  }
  
  if(arm.getNumofSols(pose)) {
    arm.startTrajectory(arm.armToDesiredPoseTrajectory(pose));
    res.result = 1;
    return true;
  }
  else {
    std::cout << "Pose out of working area, emergency stop!" << std::endl;
    res.result = -1;
    return false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "UR5_pose_control_server");
  ros::NodeHandle nh;
  

  ros::ServiceServer service = nh.advertiseService("ur5_pose_control", motion);
  ROS_INFO("Ready to move the arm.");
  ros::spin();

  return 0;
}
