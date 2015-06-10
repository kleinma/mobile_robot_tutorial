#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

// Create global variable to store the torque gain. This ranges from -1 to 1 and will scale the torque determined by the torque/velocity curve.
double g_torqueK = 0;

// Subscribe to the command from the motor controller. The message type is a std_msgs/UInt8. This message can be decoded as 0 = stop. 1 is full reverse, 128 is stop, 255 is full forward.
void mc_cmdCB(const std_msgs::UInt8& mc_cmd)
{
  // Convert the command from the motor controller into the a torque gain.
  if (mc_cmd.data == 0){
    g_torqueK = 0;
  }
  else{
    g_torqueK = (mc_cmd.data-128)/(double)127;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"wheel_sim"); // name of this node will be "minimal_publisher1"
  ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS

  // Create a publisher object to publish the determined state of the robot. Odometry messages contain both Pose and Twist with covariance. In this simulator, we will not worry about the covariance.
  ros::Publisher wheel_velPub = n.advertise<geometry_msgs::Twist>("vel",1);

  // Create a publisher object to publish the encoder count

  // Create a subscriber object to subscribe to the topic mc_cmd, which will receive a UInt8 message that contains the motor controller command.
  ros::Subscriber mc_cmdSub = n.subscribe("mc_cmd",1,mc_cmdCB);

  // This Twist message contains the wheel's velocity in its own frame. The z axis goes through the axle of the wheel. The x-axis point foward, and the y axis points up.
  geometry_msgs::Twist wheel_vel;

  // Parameters of the motor and gearbox and wheel and encoder
  // Set default values here.
  double tau_s = 98;     // stall torque in N*m
  double omega_n = 18.5; // no load speed in rad/s
  double I_bar = 1;      // Moment of inertia of the wheel in kg*m^2
  double m = 10;         // Mass of the wheel in kgs
  double r = 0.5;        // Radius of the wheel in meters
  double c = 3;          // Coloumb friction constant.
  double tpr = 20000;    // Ticks per revolution of encoder

  // Check to see if these parameters have been set and reassign values.
  n.getParam("tau_s",tau_s);
  n.getParam("omega_n",omega_n);
  n.getParam("I_bar",I_bar);
  n.getParam("m",m);
  n.getParam("r",r);
  n.getParam("c",c);
  n.getParam("tpr",tpr);

  // Variable to store the torque applied to the wheel
  double T = 0;

  // Variables to store the kinematics of the wheel
  double alpha = 0;
  double omega = 0;
  double theta = 0;

  double v = 0;

  double f = 10; // Frequency of loop in Hz
  double dt = 1/f; // Time step of loop in seconds.
  ros::Rate naptime(f); //create a ros object from the ros “Rate” class; set the sleep timer for 10Hz repetition rate

  while(ros::ok()) // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    {
      // Spin to check for messages published to our subscribed topic(s)
      ros::spinOnce();

      // This is the max torque from the toque/velocity plot, given a the current velocity, scaled by the output of our motorcontroller. Note that the expression in parentheses is always positive and g_torqueK ranges from -1 to 1. It is all multiplied by -1 because a negative alpha results in a positive movement in the x direction and we want a mc_cmd of 255 to move the wheel in the positive x direction.
      if (g_torqueK > 0){
	T = g_torqueK*(-tau_s - omega*tau_s/omega_n) - c*omega*r*r;
      }
      else if (g_torqueK < 0){
	T = -g_torqueK*(tau_s - omega*tau_s/omega_n) - c*omega*r*r;
      }
      else{
	T = -c*omega*r*r;
      }

      // From the torque we can find the acceleration of the wheel
      alpha = (T)/(I_bar+m*r*r);
      // Now numerically integrate to find omega and theta.
      theta = theta + omega*dt;
      omega = omega + alpha*dt;
      ROS_INFO("Kt = %f, torque = %f, alpha = %f, omega = %f, theta = %f",g_torqueK,T,alpha,omega,theta);

      // A positive rotation about the z axis causes the wheel to move in the negative x direction when rolling without slip.
      wheel_vel.linear.x = -omega*r;
      wheel_vel.angular.z = omega;

      // and publish the state. Note that no covariances or 3D fields are used.
      wheel_velPub.publish(wheel_vel);

      //next line will cause the loop to sleep for the balance of the desired period to achieve the specified loop frequency
      naptime.sleep();
    }
}
