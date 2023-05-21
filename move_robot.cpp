#include <iostream>
#include <k4a/k4a.h>
#include <k4abt.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/Marker.h>
#include <chrono>
#include <thread>
#include <csignal>
#include <random>
#include <fstream>

class KalmanFilter {
public:
    KalmanFilter(double process_noise, double measurement_noise, double init_value)
    {
        Q = process_noise;
        R = measurement_noise;
        x = init_value;
        P = 1.0;
    }

    double update(double measurement)
    {
        // Prediction
        P = P + Q;

        // Update
        double K = P / (P + R);
        x = x + K * (measurement - x);
        P = (1 - K) * P;

        return x;
    }

private:
    double Q; // Process noise
    double R; // Measurement noise
    double x; // Estimated value
    double P; // Uncertainty
};

const char *k4a_library_path = "/usr/lib/x86_64-linux-gnu/libk4a.so";
volatile bool keep_running = true;
ros::Time marker_creation_time;
geometry_msgs::Point marker_position;
std::ofstream times_file("times.txt");
std::ofstream dist_file("distances.txt");

float random_float(float min, float max)
{
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = max - min;
    float r = random * diff;
    return ((r > min + 0.1 || r < min - 0.1) ? (min + r) : (min + 0.1));
}

void signal_handler(int signal)
{
    if (signal == SIGINT)
    {
        keep_running = false;
    }
}

k4a_float3_t transform_coordinates(const k4a_float3_t &point, const k4a_float3_t &spine_base)
{
    k4a_float3_t new_coords;
    new_coords.xyz.x = -(point.xyz.z - spine_base.xyz.z) / 1000.0 * 1.5; // Convert to meters, negate Z axis, and scale
    new_coords.xyz.y = (point.xyz.x - spine_base.xyz.x) / 1000.0 * 1.5; // Convert to meters and scale
    new_coords.xyz.z = -(point.xyz.y - spine_base.xyz.y) / 1000.0 * 1.5; // Convert to meters, negate, and scale
    return new_coords;
}

uint32_t update_kinect(k4a_device_t device, k4abt_tracker_t bodyTracker, k4abt_frame_t &body_frame)
{
    k4a_capture_t capture = NULL;
    if (k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE) == K4A_WAIT_RESULT_SUCCEEDED)
    {
        if (k4abt_tracker_enqueue_capture(bodyTracker, capture, K4A_WAIT_INFINITE) == K4A_WAIT_RESULT_SUCCEEDED)
        {
            if (k4abt_tracker_pop_result(bodyTracker, &body_frame, K4A_WAIT_INFINITE) == K4A_WAIT_RESULT_SUCCEEDED)
            {
                uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                k4a_capture_release(capture);
                return num_bodies;
            }
        }
    }

    k4a_capture_release(capture);
    return 0;
}

visualization_msgs::Marker create_marker(const geometry_msgs::Point &position, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time::now();
    marker.ns = "kinect_target";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = position;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    return marker;
}

visualization_msgs::Marker create_random_marker()
{
    marker_position.x = random_float(0.0, 0.8);  // only positive values for x-coordinate
    marker_position.y = random_float(-0.8, 0.8); 
    marker_position.z = random_float(0.1, 0.8);  // minimum 10cm from the ground

    marker_creation_time = ros::Time::now();

    return create_marker(marker_position, 1);  // Set the ID of the random marker to 1
}



int main(int argc, char **argv)
{
std::signal(SIGINT, signal_handler);
srand(time(NULL));

// Initialize Kinect and MoveIt libraries
k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
device_config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
device_config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;

k4a_device_t device;
k4abt_skeleton_t skeleton;
k4a_result_t result = k4a_device_open(K4A_DEVICE_DEFAULT, &device);
if (result != K4A_RESULT_SUCCEEDED)
{
    std::cerr << "Failed to open device!" << std::endl;
    return -1;
}

k4a_device_start_cameras(device, &device_config);

// Create Kalman filters for wrist position
double process_noise = 0.001;
double measurement_noise = 0.1;
KalmanFilter kf_x(process_noise, measurement_noise, 0.0);
KalmanFilter kf_y(process_noise, measurement_noise, 0.0);
KalmanFilter kf_z(process_noise, measurement_noise, 0.0);

k4abt_tracker_t bodyTracker;
k4a_calibration_t calibration;
k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &calibration);
k4abt_tracker_create(&calibration, tracker_config, &bodyTracker);

ros::init(argc, argv, "move_robot");
ros::NodeHandle nh;
ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
ros::AsyncSpinner spinner(1);
spinner.start();

moveit::planning_interface::MoveGroupInterface arm("panda_arm");
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

visualization_msgs::Marker marker = create_random_marker();
marker_pub.publish(marker);

std::vector<geometry_msgs::Pose> waypoints;

k4abt_frame_t body_frame;

while (ros::ok() && keep_running)
{
    uint32_t num_bodies = update_kinect(device, bodyTracker, body_frame);

    for (uint32_t i = 0; i < num_bodies; i++)
    {
        k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
        k4a_float3_t left_wrist_joint = skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position;
        k4a_float3_t spine_base = skeleton.joints[K4ABT_JOINT_SPINE_NAVEL].position;

        k4a_float3_t left_wrist_ros = transform_coordinates(left_wrist_joint, spine_base);

        geometry_msgs::Point left_wrist_position;
        left_wrist_position.x = kf_x.update(left_wrist_ros.xyz.x);
        left_wrist_position.y = kf_y.update(left_wrist_ros.xyz.y);
        left_wrist_position.z = kf_z.update(left_wrist_ros.xyz.z);

        // Print the current position
        //std::cout << "Left Wrist ROS Coordinates: x=" << left_wrist_position.x
        //          << " y=" << left_wrist_position.y
        //          << " z=" << left_wrist_position.z << std::endl;

        // Create and publish the 3D ball marker at the wrist position
        visualization_msgs::Marker wrist_marker = create_marker(left_wrist_position, 0);
        marker_pub.publish(wrist_marker);



        // Compute the distance to the marker
            double dist_to_marker = sqrt(pow(left_wrist_position.x - marker_position.x, 2)
                                         + pow(left_wrist_position.y - marker_position.y, 2)
                                         + pow(left_wrist_position.z - marker_position.z, 2));

            std::cout << "Current Distance to Marker: " << dist_to_marker << std::endl;
            dist_file << dist_to_marker << "\n";
            // Check if the hand is within 0.10m of the marker
            if (dist_to_marker <= 0.10)
            {
                // Record the time between the creation of the marker and the current time
                double elapsed_time = (ros::Time::now() - marker_creation_time).toSec();

                // Write the elapsed time to the file
                times_file << elapsed_time << std::endl;

                // Create a new random marker
                marker = create_random_marker();
                marker_pub.publish(marker);
            }

         // Update the target pose of the robot arm
        geometry_msgs::Pose target_pose;
        target_pose.position.x = left_wrist_position.x;
        target_pose.position.y = left_wrist_position.y;
        target_pose.position.z = left_wrist_position.z;

        // Add the target pose to the waypoints vector
        waypoints.push_back(target_pose);

        // Compute the Cartesian path
        double fraction = arm.computeCartesianPath(waypoints, 0.01, 0.0, my_plan.trajectory_, false);

        // Execute the trajectory in a non-blocking manner
        arm.asyncExecute(my_plan);

        // Clear the waypoints for the next iteration
        waypoints.clear();
    }

    k4abt_frame_release(body_frame);

}
// Cleanup
times_file.close();
dist_file.close();
k4abt_tracker_shutdown(bodyTracker);
k4abt_tracker_destroy(bodyTracker);
k4a_device_stop_cameras(device);
k4a_device_close(device);

    return 0;
}