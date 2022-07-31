// UNIVERSAL //
#include <cmath>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <sstream>
#include <numeric>
#include <math.h>       /* atan, sin */

// ROS //
#include <bits/stdc++.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <burgerpilot_semiremote/detection.h>

// CV //
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#define _USE_MATH_DEFINES

using namespace std;
using namespace cv;

// GLOBAL VARIABLES
int freqs = 10;
int idx = 0;
float xc = 0.;
float yc = 0.;
float vel = 0.;
float yaw = 0.;
float v_prev_error = 0.;
vector<vector<float>> waypoints;

// CAR VARIABLES
float LOOKAHEAD = 0.1;
float WB = 0.04; // wheelbase

// PORGRAM VARIABLES
bool pure_pursuit_flag = true;

// Initialize Twist message
geometry_msgs::Twist msg;

// Initialize Detected Image
burgerpilot_semiremote::detection obj;
Mat3b current_frame;
float frame_area = 640 * 480;

// Stopsign state
int stopsign_state = 0;

// Stopsign state
int trafficlight_state = 0; // 0 = green, 1 = red

// AEB parameters
int path_blocked = 0;
float obj_dist;
int ANGLE_RANGE = 360;
int STEERING_ANGLE = 0;
float DISTANCE_THRESHOLD = 0.2;
float VELOCITY = 0.05;
float TIME_THRESHOLD = 0.5;



// ------ READ CSV ------ //
vector<vector<float>> read_points(string fname) {
    vector<vector<float>> array;
    vector<float> row;
    string line, word;
    float val;
 
    fstream file (fname, ios::in);
    while(getline(file, line))
    {
        row.clear();
        stringstream str(line);
 
        while(getline(str, word, ',')) {
            // Conver string to float
            val = stof(word);
            row.push_back(val);
        }
        array.push_back(row);
    }
    return array;
}

void read_file() 
{
    string fname = "//home/jefffer705/sae_ws/ros_ws/src/burgerpilot_semiremote/src/waypoints/wp_file.csv";
 
    fstream file (fname, ios::in);

    if(file.is_open())
    {
        waypoints = read_points(fname);
    }
    else
        ROS_INFO_STREAM("Could not open the file");
}

// ----- ODOMETRY ------ //
float norm(vector<float> vect)
{
    float sum = 0.;
    for (int i = 0; i < vect.size(); ++i) {
        sum += pow(vect[i], 2.);
    }
    return sqrt(sum);
}

/**
 * Callback function executes when new topic data comes.
 * Collects latest data for postion, orientation and velocity
 */
void pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Position
    xc = msg->pose.pose.position.x;
    yc = msg->pose.pose.position.y;
    float zc = msg->pose.pose.position.z;

    // Orientation
    float qx = msg->pose.pose.orientation.x;
    float qy = msg->pose.pose.orientation.y;
    float qz = msg->pose.pose.orientation.z;
    float qw = msg->pose.pose.orientation.w;

    // Linear velocity
    float linear_vx = msg->twist.twist.linear.x;
    float linear_vy = msg->twist.twist.linear.y;
    float linear_vz = msg->twist.twist.linear.z;

    vector<float> vect_vel = {linear_vx, linear_vy, linear_vz};
    float linear_vel = norm(vect_vel);

    // Angular velocity
    float angular_vel = msg->twist.twist.angular.z;

    // Euler from Quaternion
    tf::Quaternion quat(qx, qy, qz, qw);
    tf::Matrix3x3 mat(quat);

    double curr_roll, curr_pitch, curr_yaw;
    mat.getEulerYPR(curr_yaw, curr_pitch, curr_roll);

    // Assign to global variables
    vel = linear_vel;
    yaw = curr_yaw;

    // TESTING: Print values 
    // ROS_INFO("GLOBAL Position [%f, %f, %f], vel [%f], yaw [%f]", xc, yc, zc, vel, yaw);
    // ROS_INFO("Roll, Pitch, Yaw = [%f, %f, %f]", roll, pitch, yaw);
    // ROS_INFO("Seq -> [%d], Vel (linear x, norm, angular) -> [%f, %f, %f]", msg->header.seq, linear_vx, linear_vel, angular_vel);
    // ROS_INFO("Vel (linear x, norm) -> [%f, %f]", linear_vx, linear_vel);
    // ROS_INFO("Position (xc, yc, zc) -> [%f, %f, %f]", xc, yc, zc);
    // ROS_INFO("Orientation (qx, qy, qz, qw) -> [%f, %f, %f, %f]", qx, qy, qz, qw);
    // ROS_INFO("\n");
}

// ----- ARRAY MANIOULATION ------ //
float find_distance(float x1, float y1)
{
    float P = 2.0; // power of 2
    float distance = sqrt(pow(x1 - xc, P) + pow(y1 - yc, P));
    return distance;
}

float find_distance_index_based(int idx)
{
    float x1 = waypoints[idx][0];
    float y1 = waypoints[idx][1];
    return find_distance(x1, y1);
}

int find_nearest_waypoint()
{
    int nearest_idx = 0;
    float smallest_dist = 0.;
    float P = 2.;
    for (int i = 0; i < waypoints.size(); i++)
    {
        float wpx = waypoints[i][0];
        float wpy = waypoints[i][1];
        float idx_dist = pow(xc - wpx, P) + pow(yc - wpy, P);

        if (i == 0) {
            smallest_dist = idx_dist;
            nearest_idx = i;
        }
        if (idx_dist < smallest_dist ) {
            smallest_dist = idx_dist;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

int idx_close_to_lookahead(int idx)
{
    while (find_distance_index_based(idx) < LOOKAHEAD) {
        idx += 1;
        if (idx == waypoints.size()) { break; }
    }
    return idx - 1;
}


// ------ PURE PURSUIT ------ //
void PurePursuit()
{
    // Get the closest waypoint
    int nearest_idx = find_nearest_waypoint();
    idx = idx_close_to_lookahead(nearest_idx);
    float target_x = waypoints[idx][0];
    float target_y = waypoints[idx][1];

    // Velocity PID controller
    float kp = 1.;
    float kd = 0.;
    float ki = 0.;

    float dt = 1. / freqs;
    float v_desired = waypoints[nearest_idx][3];
    float v_error = v_desired - vel;

    float P_vel = kp * v_error;
    float I_vel = v_error * dt;
    float D_vel = kd * (v_error - v_prev_error) / dt;

    float velocity = P_vel + I_vel + D_vel;
    v_prev_error = v_error;

    // Pure Pursuit controller
    float x_delta = target_x - xc;
    float y_delta = target_y - yc;
    float alpha = atan(y_delta / x_delta) - yaw;

    if (alpha > M_PI_2) { alpha -= M_PI; }
    if (alpha < -M_PI_2) { alpha += M_PI; }

    // Set lookahead distance depending on the speed
    float lookahead = find_distance(target_x, target_y);
    float steering_angle = atan((2. * WB * sin(alpha)) / lookahead);

    // Set max wheel turning angle
    if (steering_angle > 0.5) {
        steering_angle = 0.5;
    } else if (steering_angle < -0.5) {
        steering_angle = -0.5;
    }

    // Publish the message
    msg.linear.x = velocity;
    msg.angular.z = steering_angle;
}


// ------ VISION ------ //
void camera_callback(const burgerpilot_semiremote::detection msg)
{
 
  // Pointer used for the conversion from a ROS message to 
  // an OpenCV-compatible image
  cv_bridge::CvImagePtr cv_ptr;
   
  try
  { 
   
    // Convert the ROS message
    obj = msg;
    cv_ptr = cv_bridge::toCvCopy(obj.image, "bgr8");
     
    // Store the values of the OpenCV-compatible image
    // into the current_frame variable
    current_frame = cv_ptr->image;
     
    // Display the current frame
    imshow("view", current_frame);
     
    // Display frame for 30 milliseconds
    waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert image to 'bgr8'.");
  }
}

void trafficlightState(Mat3b trafficlight) {
    // Convert to HSV
    Mat3b hsv;
    cvtColor(trafficlight, hsv, COLOR_BGR2HSV);

    // Isolate red and green light
    int width = hsv.cols;
    int height = hsv.rows;
    int h_athird = height / 3;
    int tolerance = 30;

    // Red light
    Mat1b redMask1, redMask2;
    Rect red_light(tolerance, 0, width - tolerance, h_athird + 10);
    Mat3b crop_red = hsv(red_light);
    inRange(crop_red, Scalar(0, 70, 50), Scalar(10, 255, 255), redMask1);
    inRange(crop_red, Scalar(170, 70, 50), Scalar(180, 255, 255), redMask2);
    Mat1b redMask = redMask1 | redMask2;
    float redCount = countNonZero(redMask);
    float redFrameArea = (width - tolerance) * (h_athird + 10);
    float redRatio = redCount / redFrameArea;

    // Green light
    Mat1b grnMask0;
    Rect grn_light(tolerance, height - h_athird - 15, width - tolerance, h_athird);
    Mat3b crop_grn = hsv(grn_light);
    inRange(crop_grn, Scalar(45, 100, 50), Scalar(75, 255, 255), grnMask0);
    Mat1b grnMask = grnMask0;
    float grnCount = countNonZero(grnMask);
    float grnFrameArea = (width - tolerance) * h_athird;
    float grnRatio = grnCount / grnFrameArea;

    if (redRatio > 0.25 && grnRatio < 0.25) { trafficlight_state = 1; }
    if (grnRatio > 0.25 && redRatio < 0.25) { trafficlight_state = 0; }
}



// ------ AUTOMATIC EMERGENCY BREAKING ------ // 
float get_average(std::vector<float> v) {
    if (v.empty()) {
        return 0;
    }

    float sum;
    float v_len = v.size();
    for (int i = 0; i < v_len; i++) {
        sum += v[i];
    }
    return  sum / v_len;
}

vector<float> get_data(int angle, vector<float> data) {
    int ilen = data.size();
    float ipd = (float)ilen / (float)ANGLE_RANGE; // index per division
    int half_angle = angle / 2; 
    int lwr_bound = (ipd * half_angle);
    int upr_bound = (ilen - lwr_bound);
    vector<float> idx_ranges;

    for (int i = 0; i < data.size(); i++) {
        if (i < lwr_bound || i > upr_bound) {
            idx_ranges.push_back(data[i]);
        }
    }

    return idx_ranges;
}

float get_distance(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // find average distance of a range of points directly in fornt of the vehicle
    int angle_front = 30; // Range of angle in the front of the vehicle we want to observe
    float avg_dist = 0.;

    // Get the corresponding list of indices for given range of angles
    vector<float> index_front = get_data(angle_front, msg->ranges);

    // Find avg range distance
    avg_dist = get_average(index_front);

    return avg_dist;
}

void dist_control(float distance) {
    float velocity;
    float kp_dist = 0.75;
    float dist_error = 0.0;
    float time_error = 0.0;

    if (distance > DISTANCE_THRESHOLD) {
        if (distance <= 0.4) {
            dist_error = distance - DISTANCE_THRESHOLD;
            velocity = kp_dist * dist_error;
            msg.linear.x = velocity;
            msg.angular.z = STEERING_ANGLE;
            path_blocked = 1;
        } else {
            path_blocked = 0;
        }
    } else {
        velocity = 0.;
        msg.linear.x = velocity;
        msg.angular.z = STEERING_ANGLE;
        path_blocked = 1;
    }
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    obj_dist = get_distance(msg); 
}


// ------ MAIN FUNCTION ------ //
int main(int argc, char **argv)
{
    // Load waypoints
    read_file();

    // Initialize node
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;

    // Initialize Subscriber
    ros::Subscriber controller_sub = nh.subscribe("odom", 1000, pose_callback);
    ros::Subscriber cam_sub = nh.subscribe("burger/message", 1000, camera_callback);
    ros::Subscriber sub = nh.subscribe("/scan", 1000, laser_callback);

    // Initizlize Publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    // Initialize rate
    ros::Rate rate(freqs);
    ros::spinOnce();

    // MOVE ROBOT
    while(ros::ok()) 
    {
        // ROS_INFO_STREAM("index = " << waypoints.size() << " idx = " << idx << " current pose [" << xc << "," << yc << "] [vel, yaw] = [" << vel << "," << yaw << "]");
        if (idx < waypoints.size() - 1) {
            // Check to see if the road is blocked
            dist_control(obj_dist);

            if (path_blocked == 0) {
                // Find area of detected object
                float object_area = (obj.x2 - obj.x1) * (obj.y2 - obj.y1);
                float ratio = object_area / frame_area;
                
                // Check if there's a stoplight
                if (obj.label == "stopsign" && ratio > 0.13 && stopsign_state == 0) {
                    stopsign_state = 1;
                    msg.linear.x = 0.;
                    msg.angular.z = 0.;
                    cout << "stopping for 3s" << endl;
                    pub.publish(msg);
                    ros::Duration(3).sleep();
                }

                // Check if there's a trafficlight
                if (obj.label == "trafficlight" && ratio > 0.15) {
                    Rect rec((int)obj.x1, (int)obj.y1, (int)(obj.x2 - obj.x1), (int)(obj.y2 - obj.y1));
                    Mat3b trafficlight = current_frame(rec);
                    trafficlightState(trafficlight);
                    cout << "trafficlight state = " << trafficlight_state << endl;
                    waitKey(30);
                }

                if (trafficlight_state == 1) {
                    msg.linear.x = 0.;
                    msg.angular.z = 0.;
                } else {
                    PurePursuit();

                    if (stopsign_state == 1 && ratio < 0.13) {
                        stopsign_state = 0;
                    }
                }
            }

            // Publish velocity and steering angle
            pub.publish(msg);

            // Wait until it's time for another iteration.
            ros::spinOnce();
            rate.sleep();
        }  else {
            // Publish the message
            msg.linear.x = 0.;
            msg.angular.z = 0.;
            pub.publish(msg);
            break;
        }
    }

    ROS_INFO_STREAM("DESTENATION REACHED!!!");
    return 0;
}
