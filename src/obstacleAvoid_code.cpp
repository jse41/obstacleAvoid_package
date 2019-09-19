#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include <math.h>
#include <sstream>
#include <mutex>
#include <stdlib.h>
#include <string>


// The speed of the robot in it's linear direction from the controller
float speed = 0;
// The angular rotation of the robot from the controller 
float rotate = 0; 

// Complete set of distances (and entire laser scan data) from the LIDAR
sensor_msgs::LaserScan dists; 

// Mutex lock just for safety 
std::mutex mtxTwist;

// Mutex lock for distances 
std::mutex mtxDist; 

// Response to Laser Data topic
void chatterCallback(const sensor_msgs::LaserScan& msg) 
{
  // Logging
  ROS_DEBUG("I heard certain dist: [%f]", msg.ranges[10]);

  // Move the data into Global Variable 
  mtxDist.lock();
  dists = msg; 
  mtxDist.unlock(); 
}

// Response to Twist data from the user 
void cmdCallback(const geometry_msgs::Twist& msg) 
{
  // Logging 
  ROS_DEBUG("Intended Linear: [%f]", msg.linear.x);
  ROS_DEBUG("Intended Angular: [%f]", msg.angular.z);

  // Move the data into the global variables 
  mtxTwist.lock();
  speed = msg.linear.x; 
  rotate = msg.angular.z;
  mtxTwist.unlock(); 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacleAvoid");

  // int opt; 
  // char* topic_name;
  // while ((opt = getopt(argc, (argv), "n:")) != -1)
  // {
  // 	switch (opt) {
  // 		case 'n':
  // 			topic_name = optarg;
  // 			break;
  // 		default:
  // 			printf("The -%c is not a recognized paraneter\n", opt);
  // 			break; 
  //  	}
  // }
  char* nameAr;
  if (argc > 1)
  {
  	for (int i = 1; i < argc; i++)
	  {	 
	  	ROS_DEBUG("Arg: [%s]", argv[i]);
	  }
	  nameAr = argv[1];
  }
  else
  {
  	char nameTemp[] = "des_vel"; 
  	nameAr = nameTemp;
  	ROS_DEBUG("No Arg, Default Controller [%s]", nameAr);
  }  
  ROS_INFO("Current Controller: [%s]", nameAr);
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
  //ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1000);
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // The Subsciber to the laser for the robot 
  //ros::Subscriber sub = n.subscribe("/robot0/laser_1", 1000, chatterCallback); 
  ros::Subscriber sub = n.subscribe("laser_1", 1000, chatterCallback); 

  // The subscriber to the command velocity 
  ros::Subscriber input = n.subscribe(nameAr, 1000, cmdCallback);

  // Rate in hertz that the node will write to the robot 
  ros::Rate loop_rate(10);

  // Needs to actively avoid a wall 
  bool avoid; 

  bool right; 

  // Nearest distance a wall could be going forward 
  float nearestDist = 0.3; 

  //Don't worry about this one 
  float avoidDist = nearestDist * sqrt(2); 

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

	avoid = false;

	right = true;

	mtxDist.lock();
	int max = dists.ranges.size();
	for(int beam = max / 3; beam < 2 * max / 3; beam++)
	{
		if(dists.ranges[beam] < avoidDist)
		{
			// There is a wall too close 
			avoid = true; 

			// Should I turn right or left
			if(beam < max / 2)
				right = false; 

			// Logging
			ROS_INFO("Avoiding because of Beam: [%d]", beam);
		}
	}
	mtxDist.unlock(); 

	if(!avoid)
	{
		mtxTwist.lock(); 
  		msg.linear.x = speed;
  		msg.angular.z = rotate;
  		mtxTwist.unlock(); 
	}
	else 
	{
		mtxTwist.lock(); 
		if(speed < 0)
			msg.linear.x = speed;
		else
			msg.linear.x = 0;
  		mtxTwist.unlock(); 
		
		if(right)
			msg.angular.z = -0.4; 
		else
			msg.angular.z = 0.4;
	}

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


  // Shutdown Stop Sequence 
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.angular.z = 0;
  chatter_pub.publish(msg);

  return 0;
}
