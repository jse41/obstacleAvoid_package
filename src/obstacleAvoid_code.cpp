#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>
#include <mutex>

// The speed of the robot in it's linear direction from the controller
float speed = 0;
// The angular rotation of the robot from the controller 
float rotate = 0; 

// Complete set of distances (and entire laser scan data) from the LIDAR
sensor_msgs::LaserScan dists; 

// Mutex lock just for safety 
std::mutex mtx;

// Response to Laser Data topic
void chatterCallback(const sensor_msgs::LaserScan& msg) 
{
  // Logging
  ROS_DEBUG("I heard certain dist: [%f]", msg.ranges[10]);

  // Move the data into Global Variable 
  dists = msg; 
}

// Response to Twist data from the user 
void cmdCallback(const geometry_msgs::Twist& msg) 
{
  // Logging 
  ROS_DEBUG("Intended Linear: [%f]", msg.linear.x);
  ROS_DEBUG("Intended Angular: [%f]", msg.angular.z);

  // Move the data into the global variables 
  mtx.lock();
  speed = msg.linear.x; 
  rotate = msg.angular.z;
  mtx.unlock(); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacleAvoid");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  // Ideally will be populated from the commarnd args for the controller topic name 
  std_msgs::String topic;

  // The Publisher for the arguments sent to the robot 
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1000);

  // The Subsciber to the laser for the robot 
  ros::Subscriber sub = n.subscribe("/robot0/laser_1", 1000, chatterCallback); 

  // The subscriber to the command velocity 
  ros::Subscriber input = n.subscribe("/cmd_vel", 1000, cmdCallback);

  // Rate in hertz that the node will write to the robot 
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     
    geometry_msgs::Twist msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
*/
  	// Generate twist command for the robot 
	geometry_msgs::Twist msg;
	mtx.lock(); 
  	msg.linear.x = speed;
  	msg.angular.z = rotate;
  	mtx.unlock();  

    //ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);


    ros::spinOnce();


    loop_rate.sleep();
  }


  return 0;
}
