#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

// Create global variable to store the velocities of the left and right wheels
double g_vl = 0;
double g_vr = 0;

// Subscribe to the left and right wheel velocities and store them in a global variable
void left_wheelCB(const geometry_msgs::Twist& msg)
{
  g_vl = msg.linear.x;
}
void right_wheelCB(const geometry_msgs::Twist& msg)
{
  g_vr = msg.linear.x;
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"robot_sim"); // name of this node will be "minimal_publisher1"
  ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS

  // Create a publisher object to publish the determined state of the robot. Odometry messages contain both Pose and Twist with covariance. In this simulator, we will not worry about the covariance.
  ros::Publisher robot_statePub = n.advertise<nav_msgs::Odometry>("odom",1);

  // Create a subscriber object to subscribe to the topic cmd_vel, which will receive a Twist message that contains the commanded velocity of the robot.
  ros::Subscriber left_wheelSub = n.subscribe("left_wheel/vel",1,left_wheelCB);
  ros::Subscriber right_wheelSub = n.subscribe("right_wheel/vel",1,right_wheelCB);

  // Create an Odometry message to store the robot state
  // The state consists of x,y in the map frame, theta (or yaw) of the robot frame relative to the map frame, the velocity of the robot in the robot's frame (all in the x_robot direction), and omega, or the change in theta.
  // All of this fits nicely into an Odometry message. The Odometry message provides room for much more. However, we will only populate the fields that we are using.
  nav_msgs::Odometry robot_state;
  robot_state.child_frame_id = "robot0";
  robot_state.header.frame_id = "map";

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

  double cf; // Correction factor used for numerically integrating x and y

  // Set up the robot parameters and check the parameter server to replace the default value
  double b = 0.75; // Track width (distance between wheels) in meters
  n.getParam("b",b);

  double f = 10; // Frequency of loop in Hz
  double dt = 1/f; // Time step of loop in seconds.
  ros::Rate naptime(10); //create a ros object from the ros “Rate” class; set the sleep timer for 10Hz repetition rate
  while(ros::ok()) // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    {
      // Spin to check for messages published to our subscribed topic(s)
      ros::spinOnce();

      // Update the state estimate using x_p (the vector) and u

      ////////////////////////////////////////////////////////////
      // First determine the correction factor. This is described in Wang 1988. The limit of cf as omega approaches 0 is 1. The following if statement protects against dividing by zero.
      if (omega == 0){
	cf = 1;
      }
      else{
	cf = sin(omega*dt/2)/(omega*dt/2);
      }

      x = x_p + cf*v*dt*cos(theta+omega*dt/2);
      y = y_p + cf*v*dt*sin(theta+omega*dt/2);

      theta = theta_p + omega*dt;

      v     = (g_vr+g_vl)/2;
      omega = (g_vr-g_vl)/b; // b is the track width (the width between the wheels)
      ////////////////////////////////////////////////////////////

      // Stuff these state variables into our Odometry message, robot_state
      robot_state.pose.pose.position.x = x;
      robot_state.pose.pose.position.y = y;
      robot_state.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      robot_state.twist.twist.linear.x = v;
      robot_state.twist.twist.angular.z = omega;

      // and publish the state. Note that no covariances or 3D fields are used.
      robot_statePub.publish(robot_state);

      // In addition to publishing an odometry message, also broadcast a transform
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(x, y, 0.0) );
      tf::Quaternion q;
      q.setRPY(0, 0, theta);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "robot0"));

      // Set the previous state equal to the current state for the next loop
      x_p = x;
      y_p = y;
      theta_p = theta;

      //next line will cause the loop to sleep for the balance of the desired period to achieve the specified loop frequency
      naptime.sleep();
    }
}
