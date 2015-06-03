#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// Create global variable to store the commanded linear and angular velocities
double g_uv = 0;
double g_uomega = 0;

// Subscribe to the velocity command and store v and omega in 
void cmd_velCB(const geometry_msgs::Twist& msg)
{
  g_uv = msg.linear.x;
  g_uomega = msg.angular.z;
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"robot_sim"); // name of this node will be "minimal_publisher1"
  ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS

  // Create a publisher object to publish the determined state of the robot. Odometry messages contain both Pose and Twist with covariance. In this simulator, we will not worry about the covariance.
  ros::Publisher robot_statePub = n.advertise<nav_msgs::Odometry>("robot_state",1);

  ros::Subscriber cmd_velSub= n.subscribe("cmd_vel",1,cmd_velCB);


  // Create an Odometry message to store the robot state
  nav_msgs::Odometry robot_state;
  // Create and initialize variables to store current and previous states (p stands for previous)
  // This could be stored in robot_state. But it is easier to use regular variable and then stuff it all back in robot_state at the end.
  double x = 0;
  double y = 0;
  double theta = 0;
  double v = 0;
  double omega = 0;

  double x_p = x;
  double y_p = y;
  double theta_p = theta;
  double v_p = v;
  double omega_p = omega;

  ros::Rate naptime(10); //create a ros object from the ros “Rate” class; set the sleep timer for 10Hz repetition rate
  while(ros::ok()) // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    {
      // Spin to check for messages published to our subscribed topic(s)
      ros::spinOnce();
      // Update the state estimate using x_p and u
      x = x_p + 0;
      y = y_p + 0;
      theta = theta_p + 0;

      v = g_uv;
      omega = g_uomega;

      // Stuff these state variables into our Odometry message, robot_state
      robot_state.pose.pose.position.x = x;
      robot_state.pose.pose.position.y = y;
      robot_state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      robot_state.twist.twist.linear.x = v;
      robot_state.twist.twist.angular.z = omega;

      // and publish the state. Note that no covariances or 3D fields are used.
      robot_statePub.publish(robot_state);


      // Set the previous state equal to the current state for the next loop
      x_p = x;
      y_p = y;
      theta_p = theta;
      v_p = v;
      omega_p = omega;
      //next line will cause the loop to sleep for the balance of the desired period to achieve the specified loop frequency
      naptime.sleep();
    }
}
