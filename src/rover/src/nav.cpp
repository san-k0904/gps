#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;
double cur_latitude, cur_longitude, dest_latitude, dest_longitude,initial_yaw;

double bearing(double lat1, double lon1, double lat2, double lon2)
{
    double dlong,y,x;
    dlong=(lon2-lon1);
    y=sin(dlong)*cos(lat2);
    x=cos(lat1) *sin(lat2)-sin(lat1)*cos(lat2)*cos(dlong);
    double angle = atan2(y,x);
     // Convert the angle from radians to degrees
    angle = angle * 180.0 / M_PI;

    // Normalize the angle to the range [0, 360)
    angle = fmod((angle + 360.0), 360.0)+90;

    return angle;
}

double toRadians(const long double & degree)
{
    // cmath library in C++ 
    // defines the constant
    // M_PI as the value of
    // pi accurate to 1e-30
    double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}
 
double distance(double lat1, double long1,double lat2,double long2)
{
    // Convert the latitudes 
    // and longitudes
    // from degree to radians.
     
    // Haversine Formula
    double dlong = long2 - long1;
    double dlat = lat2 - lat1;
 
    double ans = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlong / 2), 2);
 
    ans = 2 * asin(sqrt(ans));
    double R = 6371000;

    ans = ans * R;
 
    return ans;
}
 

void imucallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double  yaw ,roll, pitch;
    //ROS_INFO("%lf",msg->orientation.z);
    tf2::Quaternion quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);

    tf2::Matrix3x3 rot_matrix(quaternion);
    
    rot_matrix.getRPY(roll, pitch, yaw);
    yaw = yaw * 180.0 / M_PI;
    initial_yaw = fmod(yaw + 360.0, 360.0);

    //ROS_INFO("yaw: %lf",yaw);
}

void gpscallback(const sensor_msgs::NavSatFix::ConstPtr &gpsmsg)
{
    //cout<<"latitude:"<< gpsmsg->latitude<< "longitude: "<<gpsmsg->longitude<<endl;
    cur_latitude = toRadians(gpsmsg->latitude);
    cur_longitude = toRadians(gpsmsg->longitude);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "line_follow_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/imu", 1, imucallback);
    ros::Subscriber sub_gps = nh.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 1, gpscallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate loop_rate(20);

    ROS_INFO("ENTER CO-ORDINATES:");
    cin >> dest_latitude >> dest_longitude;
    dest_latitude=toRadians(dest_latitude);
    dest_longitude=toRadians(dest_longitude);
    ros::spinOnce();
    bool target_angle = false;
    bool reached_target_dest = false;
    bool perfect= false;
    double target_dist = distance(cur_latitude, cur_longitude, dest_latitude, dest_longitude);
    //ROS_INFO("LATITUDE: %lf, LONGITUDE: %lf , target distance: %lf", cur_latitude, cur_longitude, target_dist);
    double distance_accuracy = 0.2;
    double rotation_accuracy = 0.5;
    geometry_msgs::Twist traverse;
    while (ros::ok() && !target_angle &&!perfect)
    {
        
        double dest_angle = bearing(cur_latitude, cur_longitude, dest_latitude, dest_longitude);
        double angle_diff = (dest_angle - initial_yaw);

        cout<<"Destination angle: "<<dest_angle<<endl;
        
        cout<<"Remaining angle: "<<angle_diff<<endl;
        // Update traverse values based on conditions
        
        if (abs(angle_diff) > dest_angle+0.7 )
        {
            double clockwise_diff = dest_angle - (initial_yaw + 360.0);
            double counterclockwise_diff = (initial_yaw + 360.0) - dest_angle;

            if (abs(clockwise_diff) < abs(counterclockwise_diff))
            {
                // Clockwise rotation is faster
                traverse.angular.z = -0.5;
                traverse.linear.x=0;
            }
            else
            {
                // Counterclockwise rotation is faster
                traverse.linear.x=0;
                traverse.angular.z = +0.5;
            }

        //     traverse.linear.x = 0.2 * remaining_distance;
        //     if (angle_diff < 0)
        //     {
        //         traverse.angular.z = -0.005 * angle_diff;
        //     }
        //     else
        //     {
        //         traverse.angular.z = 0.005 * angle_diff;
        //     }
        // }
        // else {
        //     if (abs(angle_diff) < 1)
        //     {
        //         if (abs(angle_diff) <= abs(dest_angle)+rotation_accuracy)
        //         {
        //             target_angle = true;
        //             traverse.angular.z = 0;
        //         }
        //         else
        //         {
        //             traverse.angular.z = (angle_diff < 0) ? -0.001 : 0.001;
        //         }
        //     }
        //     if (remaining_distance < 2)
        //     {
        //         if (remaining_distance <= distance_accuracy)
        //         {
        //             reached_target_dest = true;
        //             traverse.linear.x = 0;
        //         }
        //         else
        //         {
        //             traverse.linear.x = 0.05;
        //         }
        //     }

        // Publish traverse message
        //ROS_INFO("%lf  , %lf", traverse.linear.x, traverse.angular.z);
        
                //ROS_INFO("%lf  , %lf",traverse.linear.x,traverse.angular.z);
        }
        else{
            target_angle=true;
            traverse.linear.x=0;
            traverse.angular.z=0;
        }
        pub.publish(traverse);
        

            // if (abs(angle_diff) > rotation_accuracy)
            // {

            //     rotate.angular.z = 0.05*angle_diff;
            //     pub.publish(rotate);
            // }

            // else
            // {
            //     rotate.angular.z= 0;
            //     pub.publish(rotate);
            //     target_angle = true;
            // }

            //ROS_INFO("ANGLE: %lf , ANGLE DIFF: %lf , YAW: %lf", dest_angle, angle_diff, yaw);
            //ROS_INFO("CURRENT LATITUDE LONGITUDE %lf  ,  %lf", cur_latitude, cur_longitude);
        ros::spinOnce();
        loop_rate.sleep();
    }
    while (ros::ok()&&!reached_target_dest&&!perfect)
    {
        double remaining_distance = distance(cur_latitude, cur_longitude, dest_latitude, dest_longitude);
        cout<<"Remaining distance: "<<remaining_distance<<endl;
        if(remaining_distance>0.5){
            traverse.angular.z=0;
            traverse.linear.x=0.05*remaining_distance;
        }
        else{
            reached_target_dest=true;
            traverse.linear.x=0;
            traverse.angular.z=0;
        }
        if(target_angle && reached_target_dest){
            perfect=true;
            traverse.linear.x = 0;
            traverse.angular.z = 0;
        }
        pub.publish(traverse);
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    
    // while (ros::ok() && !reached_target_dest)
    // {
        
    //     double remaining_distance = distance(cur_latitude, cur_longitude, dest_latitude, dest_longitude);
    //     double distance_accuracy = 1.0;
    //     ROS_INFO("dist diff = %lf", remaining_distance);

    //     geometry_msgs::Twist move;
    //     if (remaining_distance > distance_accuracy)
    //     {
    //         move.linear.x = 0.5; //can be y for me
    //         pub.publish(move);
    //     }
    //     else
    //     {
    //         move.linear.x = 0; //can be y for me
    //         pub.publish(move);
    //         reached_target_dest = true;
    //         ROS_INFO("reached destination" );
    //     }
         

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    return 0;
}