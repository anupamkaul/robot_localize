#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>

// class contain all the processes to track the robot location and add or delete markers as necessary
class AddMarker {

  private:
    ros::NodeHandle n;
    ros::Publisher marker_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber pickzone_sub;
    ros::Subscriber dropzone_sub;
    visualization_msgs::Marker marker;
    double pickupZone_x = 3.4000;
    double pickupZone_y = -1.5400;
    double dropoffZone_x = 2.4000;
    double dropoffZone_y = -3.2400;
    double threshold;
	const int threshold_multiplier = 3;

public:
    AddMarker() {
      
        //create the publisher node for displaying the markers
        marker_pub = n.advertise<visualization_msgs::Marker>("/add_markers/visualization_marker", 1);
      
        // create the subscriber node to track the odomentry
        odom_sub = n.subscribe("/odom", 1, &AddMarker::odomCallback, this);

        // create listeners for pickzone and dropzone topics
        pickzone_sub = n.subscribe("/pick_objects/pickzone", 1000, &AddMarker::pickzone_callback, this);
        dropzone_sub = n.subscribe("/pick_objects/dropzone", 1000, &AddMarker::dropzone_callback, this);

        // initialize marker
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "add_markers";
        marker.id = 0;

        // Set the marker type.  This is CUBE
        marker.type = visualization_msgs::Marker::CUBE;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side - set to 1/4 of a meter
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;

        // Set the color -- red --  be sure to set alpha to something non-zero!
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // Set marker's initial coordinates 
        marker.pose.position.x = pickupZone_x;
        marker.pose.position.y = pickupZone_y;
      
        // set the duration of the marker.  Until deleted      
        marker.lifetime = ros::Duration();

        // allow for noise from odomentry.  allow a difference of postion of 0.25 meter
        threshold = marker.scale.x;
      
        // publish the marker at the pickup zone
		publishMarker();

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        // This method tracks the odometry reading to determine if the robot is at the pickup zone or the dropoff zone
      
        float pos_x = msg->pose.pose.position.x;
        float pos_y = msg->pose.pose.position.y;

        //ROS_INFO_STREAM("pos_x :" << pos_x << " pos_y :" << pos_y << "\n");

        // odom could be coming in as x-inverted..
      
        /*
        if (pos_x > 0) {
            pos_x = -abs(pos_x);
        } 
        else 
        {
            pos_x = abs(pos_x);
        }


        if (pos_y > 0) {
            pos_y = -abs(pos_y);
        } 
        else 
        {
            pos_y = abs(pos_y);
        }
        */


        // give position tolerance of + or - the marker's radius
        bool atPickupZone = (pickupZone_x - threshold < pos_x && pos_x < pickupZone_x + threshold)
                && (pickupZone_y - threshold < pos_y && pos_y < pickupZone_y + threshold);

        bool atDropoffZone = (dropoffZone_x - threshold <= pos_x && pos_x <= dropoffZone_x + threshold)
                && (dropoffZone_y - threshold < pos_y) && (pos_y < dropoffZone_y + threshold);

        if (atPickupZone) {

            // delete marker and set it to new coordinates
            marker.action = visualization_msgs::Marker::DELETE; 
            // publish marker
            publishMarker();
 
            // setup marker for drop off zone
			ROS_INFO("In Pickup Zone   ");
  
           // pause 5s to simulate pickup
            ros::Duration(5).sleep();
        } 
       else
       {
          if (atDropoffZone) {
            // setup marker for drop off zone
            marker.pose.position.x = dropoffZone_x;
            marker.pose.position.y = dropoffZone_y;
            marker.action = visualization_msgs::Marker::ADD;
			ROS_INFO("In Dropoff Zone   ");
           // publish marker
           publishMarker();
           // pause 5s to simulate dropoff
            ros::Duration(5).sleep();
          }
       }


    }

    void pickzone_callback(const std_msgs::String::ConstPtr& msg) {

        // delete marker and set it to new coordinates
        marker.action = visualization_msgs::Marker::DELETE; 
        // publish marker
        publishMarker();
 
        // setup marker for drop off zone
        ROS_INFO("In Pickup Zone (callback)  ");
  
        // pause 5s to simulate pickup
        ros::Duration(5).sleep();

    }

    void dropzone_callback(const std_msgs::String::ConstPtr& msg) {

        // setup marker for drop off zone
        marker.pose.position.x = dropoffZone_x;
        marker.pose.position.y = dropoffZone_y;
        marker.action = visualization_msgs::Marker::ADD;
        ROS_INFO("In Dropoff Zone (callback)  ");

        // publish marker
        publishMarker();
        // pause 5s to simulate dropoff
        ros::Duration(5).sleep();

    }

    void publishMarker() {
        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1) {
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }

        marker_pub.publish(marker);
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_markers");

    // create a call called AddMarker.  This class has all the call for creating and deleting markers based on robot location
    AddMarker addMarker;
  
    // keep the process alive
    ros::spin();

    // after process is complete - exit the program  
    return 0;
}
