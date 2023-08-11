#include <ros/ros.h>

// interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>

#include "racecar_simulator/ackermann_kinematics.hpp"
#include "racecar_simulator/pose_2d.hpp"
#include "racecar_simulator/scan_simulator_2d.hpp"

#include "racecar_simulator/car_params.hpp"
#include "racecar_simulator/car_state.hpp"
#include "racecar_simulator/ks_kinematics.hpp"
#include "racecar_simulator/precompute.hpp"
#include "racecar_simulator/st_kinematics.hpp"

#include <iostream>
#include <math.h>

#include <functional>

using namespace racecar_simulator;

class RacecarSimulator {
  private:
    // A ROS node
    ros::NodeHandle n;
    int obj_num_;
    // The transformation frames used
    std::string map_frame, base_frame, scan_frame;

    // obstacle states (1D index) and parameters
    std::vector<int> added_obs;
    // listen for clicked point for adding obstacles
    ros::Subscriber obs_sub;
    int obstacle_size;

    // interactive markers' server
    interactive_markers::InteractiveMarkerServer im_server;

    // The car state and parameters
    std::vector<CarState> state_;
    double previous_seconds;
    double scan_distance_to_base_link_;
    double max_speed_, max_steering_angle_;
    double max_accel_, max_steering_vel_, max_decel_;
    std::vector<double> desired_speed_, desired_steer_ang_, desired_accel_;
    std::vector<double> accel_, steer_angle_vel_;
    CarParams params_;
    double width_;

    // A simulator of the laser
    ScanSimulator2D scan_simulator;
    double map_free_threshold;

    // For publishing transformations
    tf2_ros::TransformBroadcaster br_;

    // A timer to update the pose
    ros::Timer update_pose_timer;

    // Listen for drive commands
    std::vector<ros::Subscriber> drive_sub_;

    // Listen for a map
    ros::Subscriber map_sub;
    bool map_exists = false;

    // Listen for updates to the pose
    std::vector<ros::Subscriber> pose_sub_;
    ros::Subscriber pose_rviz_sub_;
    ros::Subscriber opp_pose_rviz_sub_;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    std::vector<ros::Publisher> scan_pub_;
    std::vector<ros::Publisher> odom_pub_;
    std::vector<ros::Publisher> imu_pub_;

    // publisher for map with obstacles
    ros::Publisher map_pub;

    // keep an original map for obstacles
    nav_msgs::OccupancyGrid original_map_;
    nav_msgs::OccupancyGrid current_map_;

    // for obstacle collision
    int map_width, map_height;
    double map_resolution, origin_x, origin_y;

    // safety margin for collisions
    double thresh;
    double speed_clip_diff;

    // precompute cosines of scan angles
    std::vector<double> cosines;

    // scan parameters
    double scan_fov;
    double scan_ang_incr;

    // pi
    const double PI = 3.1415;

    // precompute distance from lidar to edge of car for each beam
    std::vector<double> car_distances;

    // for collision check
    bool TTC = false;
    double ttc_threshold;

  public:
    RacecarSimulator() : im_server("racecar_sim") {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        clearvector();

        previous_seconds = ros::Time::now().toSec();

        // Get the topic names
        std::string drive_topic, map_topic, scan_topic, pose_topic,
            pose_rviz_topic, opp_pose_rviz_topic, odom_topic, imu_topic;
        n.getParam("drive_topic", drive_topic);
        n.getParam("map_topic", map_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("pose_topic", pose_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("pose_rviz_topic", pose_rviz_topic);
        n.getParam("opp_pose_rviz_topic", opp_pose_rviz_topic);
        n.getParam("imu_topic", imu_topic);

        // Get the transformation frame names
        n.getParam("map_frame", map_frame);
        n.getParam("base_frame", base_frame);
        n.getParam("scan_frame", scan_frame);

        // Fetch the car parameters
        int scan_beams;
        double update_pose_rate, scan_std_dev;
        n.getParam("obj_num", obj_num_);
        n.getParam("wheelbase", params_.wheelbase);
        n.getParam("update_pose_rate", update_pose_rate);
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_field_of_view", scan_fov);
        n.getParam("scan_std_dev", scan_std_dev);
        n.getParam("map_free_threshold", map_free_threshold);
        n.getParam("scan_distance_to_base_link", scan_distance_to_base_link_);
        n.getParam("max_speed", max_speed_);
        n.getParam("max_steering_angle", max_steering_angle_);
        n.getParam("max_accel", max_accel_);
        n.getParam("max_decel", max_decel_);
        n.getParam("max_steering_vel", max_steering_vel_);
        n.getParam("friction_coeff", params_.friction_coeff);
        n.getParam("height_cg", params_.h_cg);
        n.getParam("l_cg2rear", params_.l_r);
        n.getParam("l_cg2front", params_.l_f);
        n.getParam("C_S_front", params_.cs_f);
        n.getParam("C_S_rear", params_.cs_r);
        n.getParam("moment_inertia", params_.I_z);
        n.getParam("mass", params_.mass);
        n.getParam("width", width_);

        // clip velocity
        n.getParam("speed_clip_diff", speed_clip_diff);

        // Determine if we should broadcast
        n.getParam("broadcast_transform", broadcast_transform);

        // Get obstacle size parameter
        n.getParam("obstacle_size", obstacle_size);

        // Initialize car state and driving commands
        for (int i = 0; i < obj_num_; i++) {
            CarState state = {.x = i,
                              .y = i,
                              .theta = 0,
                              .velocity = 0,
                              .steer_angle = 0.0,
                              .angular_velocity = 0.0,
                              .slip_angle = 0.0,
                              .st_dyn = false};

            state_.push_back(state);

            accel_.push_back(0.0);
            steer_angle_vel_.push_back(0.0);
            desired_speed_.push_back(0.0);
            desired_steer_ang_.push_back(0.0);
            desired_accel_.push_back(0.0);
        }

        // Initialize a simulator of the laser scanner
        scan_simulator = ScanSimulator2D(scan_beams, scan_fov, scan_std_dev);

        // Make a publisher for publishing map with obstacles
        map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

        // Start a timer to output the pose
        update_pose_timer = n.createTimer(ros::Duration(update_pose_rate),
                                          &RacecarSimulator::update_pose, this);

        // pose_rviz_sub_ = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(pose_rviz_topic, 1, &RacecarSimulator::pose_rviz_callback, this);
        pose_rviz_sub_ = n.subscribe(pose_rviz_topic, 1, &RacecarSimulator::pose_rviz_callback, this);
        opp_pose_rviz_sub_ = n.subscribe(opp_pose_rviz_topic, 1, &RacecarSimulator::opp_pose_rviz_callback, this);

        // Start a subscriber to listen to drive commands
        for (int i = 0; i < obj_num_; i++) {
            ros::Subscriber drive_sub, pose_sub;
            ros::Publisher scan_pub, odom_pub, imu_pub;

            drive_sub = n.subscribe<ackermann_msgs::AckermannDriveStamped>(
                drive_topic + std::to_string(i), 1,
                boost::bind(&RacecarSimulator::drive_callback, this, _1, i));
            pose_sub = n.subscribe<geometry_msgs::PoseStamped>(
                pose_topic + std::to_string(i), 1,
                boost::bind(&RacecarSimulator::pose_callback, this, _1, i));
            

            // Make a publisher for laser scan messages
            scan_pub = n.advertise<sensor_msgs::LaserScan>(
                scan_topic + std::to_string(i), 1);

            // Make a publisher for odometry messages
            odom_pub = n.advertise<nav_msgs::Odometry>(
                odom_topic + std::to_string(i), 1);

            // Make a publisher for IMU messages
            imu_pub =
                n.advertise<sensor_msgs::Imu>(imu_topic + std::to_string(i), 1);

            drive_sub_.push_back(drive_sub);
            // pose_sub_.push_back(pose_sub);
            // pose_rviz_sub_.push_back(pose_rviz_sub);
            scan_pub_.push_back(scan_pub);
            odom_pub_.push_back(odom_pub);
            imu_pub_.push_back(imu_pub);
        }

        // Start a subscriber to listen to new maps
        map_sub =
            n.subscribe(map_topic, 1, &RacecarSimulator::map_callback, this);

        // obstacle subscriber
        obs_sub = n.subscribe("/clicked_point", 1,
                              &RacecarSimulator::obs_callback, this);

        // get collision safety margin
        n.getParam("coll_threshold", thresh);
        n.getParam("ttc_threshold", ttc_threshold);

        scan_ang_incr = scan_simulator.get_angle_increment();

        cosines =
            Precompute::get_cosines(scan_beams, -scan_fov / 2.0, scan_ang_incr);
        car_distances = Precompute::get_car_distances(
            scan_beams, params_.wheelbase, width_, scan_distance_to_base_link_,
            -scan_fov / 2.0, scan_ang_incr);

        // OBSTACLE BUTTON:
        // wait for one map message to get the map data array
        boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
        nav_msgs::OccupancyGrid map_msg;
        map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
        if (map_ptr != NULL) {
            map_msg = *map_ptr;
        }
        original_map_ = map_msg;
        current_map_ = map_msg;
        std::vector<int8_t> map_data_raw = map_msg.data;
        std::vector<int> map_data(map_data_raw.begin(), map_data_raw.end());

        map_width = map_msg.info.width;
        map_height = map_msg.info.height;
        origin_x = map_msg.info.origin.position.x;
        origin_y = map_msg.info.origin.position.y;
        map_resolution = map_msg.info.resolution;

        // create button for clearing obstacles
        visualization_msgs::InteractiveMarker clear_obs_button;
        clear_obs_button.header.frame_id = "map";
        // clear_obs_button.pose.position.x =
        // origin_x+(1/3)*map_width*map_resolution;
        // clear_obs_button.pose.position.y =
        // origin_y+(1/3)*map_height*map_resolution;
        // TODO: find better positioning of buttons
        clear_obs_button.pose.position.x = 0;
        clear_obs_button.pose.position.y = -5;
        clear_obs_button.scale = 1;
        clear_obs_button.name = "clear_obstacles";
        clear_obs_button.description = "Clear Obstacles\n(Left Click)";
        visualization_msgs::InteractiveMarkerControl clear_obs_control;
        clear_obs_control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::BUTTON;
        clear_obs_control.name = "clear_obstacles_control";
        // make a box for the button
        visualization_msgs::Marker clear_obs_marker;
        clear_obs_marker.type = visualization_msgs::Marker::CUBE;
        clear_obs_marker.scale.x = clear_obs_button.scale * 0.45;
        clear_obs_marker.scale.y = clear_obs_button.scale * 0.65;
        clear_obs_marker.scale.z = clear_obs_button.scale * 0.45;
        clear_obs_marker.color.r = 0.0;
        clear_obs_marker.color.g = 1.0;
        clear_obs_marker.color.b = 0.0;
        clear_obs_marker.color.a = 1.0;

        clear_obs_control.markers.push_back(clear_obs_marker);
        clear_obs_control.always_visible = true;
        clear_obs_button.controls.push_back(clear_obs_control);

        im_server.insert(clear_obs_button);
        im_server.setCallback(
            clear_obs_button.name,
            boost::bind(&RacecarSimulator::clear_obstacles, this, _1));

        im_server.applyChanges();

        ROS_INFO("Simulator constructed.");
    }

    void clearvector() {
        state_.clear();
        accel_.clear();
        steer_angle_vel_.clear();
        desired_speed_.clear();
        desired_steer_ang_.clear();
        desired_accel_.clear();
        drive_sub_.clear();
        scan_pub_.clear();
        odom_pub_.clear();
        imu_pub_.clear();
    }

    void update_pose(const ros::TimerEvent &) {

        // simulate P controller
        for (int i = 0; i < obj_num_; i++) {
            // compute_accel(desired_speed);
            set_accel(desired_accel_[i], i);
            set_steer_angle_vel(compute_steer_vel(desired_steer_ang_[i],i),i);

            // Update the pose
            ros::Time timestamp = ros::Time::now();
            double current_seconds = timestamp.toSec();
            state_[i] = STKinematics::update(
                state_[i], accel_[i], steer_angle_vel_[i], params_,
                current_seconds - previous_seconds);
            state_[i].velocity =
                std::min(std::max(state_[i].velocity, -max_speed_), max_speed_);
            state_[i].steer_angle =
                std::min(std::max(state_[i].steer_angle, -max_steering_angle_),
                         max_steering_angle_);

            previous_seconds = current_seconds;

            /// Publish the pose as a transformation
            pub_pose_transform(timestamp, i);

            /// Publish the steering angle as a transformation so the wheels
            pub_steer_ang_transform(timestamp, i);

            // Make an odom message as well and publish it
            pub_odom(timestamp, i);

            // TODO: make and publish IMU message
            pub_imu(timestamp, i);

            /// KEEP in sim
            // If we have a map, perform a scan
            if (map_exists) {
                // Get the pose of the lidar, given the pose of base link
                // (base link is the center of the rear axle)
                Pose2D scan_pose;
                scan_pose.x = state_[i].x + scan_distance_to_base_link_ *
                                                std::cos(state_[i].theta);
                scan_pose.y = state_[i].y + scan_distance_to_base_link_ *
                                                std::sin(state_[i].theta);
                scan_pose.theta = state_[i].theta;

                // Compute the scan from the lidar
                std::vector<double> scan = scan_simulator.scan(scan_pose);

                // Convert to float
                std::vector<float> scan_float(scan.size());
                for (size_t i = 0; i < scan.size(); i++)
                    scan_float[i] = scan[i];

                // TTC Calculations are done here so the car can be halted in
                // the simulator: to reset TTC
                bool no_collision = true;
                if (state_[i].velocity != 0) {
                    for (size_t i = 0; i < scan_float.size(); i++) {
                        // TTC calculations

                        // calculate projected velocity
                        double proj_velocity = state_[i].velocity * cosines[i];
                        double ttc = (scan_float[i] - car_distances[i]) /
                                     proj_velocity; // ???
                        // if it's small enough to count as a collision
                        if ((ttc < ttc_threshold) && (ttc >= 0.0)) {
                            // if (!TTC) {
                            //     first_ttc_actions();
                            // }

                            no_collision = false;
                            TTC = true;

                            ROS_INFO("Collision detected");
                        }
                    }
                }

                // reset TTC
                if (no_collision)
                    TTC = false;

                // Publish the laser message
                for (int i = 0; i < obj_num_; i++) {
                    sensor_msgs::LaserScan scan_msg;
                    scan_msg.header.stamp = timestamp;
                    scan_msg.header.frame_id = scan_frame + std::to_string(i);
                    scan_msg.angle_min =
                        -scan_simulator.get_field_of_view() / 2.;
                    scan_msg.angle_max =
                        scan_simulator.get_field_of_view() / 2.;
                    scan_msg.angle_increment =
                        scan_simulator.get_angle_increment();
                    scan_msg.range_max = 100;
                    scan_msg.ranges = scan_float;
                    scan_msg.intensities = scan_float;
                    scan_pub_[i].publish(scan_msg);
                }

                // Publish a transformation between base link and laser
                pub_laser_link_transform(timestamp, i);
            }
        }

    } // end of update_pose

    /// ---------------------- GENERAL HELPER FUNCTIONS ----------------------

    std::vector<int> ind_2_rc(int ind) {
        std::vector<int> rc;
        int row = floor(ind / map_width);
        int col = ind % map_width - 1;
        rc.push_back(row);
        rc.push_back(col);
        return rc;
    }

    int rc_2_ind(int r, int c) { return r * map_width + c; }

    std::vector<int> coord_2_cell_rc(double x, double y) {
        std::vector<int> rc;
        rc.push_back(static_cast<int>((y - origin_y) / map_resolution));
        rc.push_back(static_cast<int>((x - origin_x) / map_resolution));
        return rc;
    }

    void first_ttc_actions() {
        for (int i = 0; i < obj_num_; i++) {
            state_[i].velocity = 0.0;
            state_[i].angular_velocity = 0.0;
            state_[i].slip_angle = 0.0;
            state_[i].steer_angle = 0.0;
            steer_angle_vel_[i] = 0.0;
            accel_[i] = 0.0;
            desired_speed_[i] = 0.0;
            desired_steer_ang_[i] = 0.0;
        }
        // completely stop vehicle
    }

    void set_accel(double accel, int i) {
        accel_[i] = std::min(std::max(accel, -max_accel_), max_accel_);
    }

    void set_steer_angle_vel(double steer_angle_vel, int i) {
        steer_angle_vel_[i] = std::min(
            std::max(steer_angle_vel, -max_steering_vel_), max_steering_vel_);
    }

    void add_obs(int ind) {
        std::vector<int> rc = ind_2_rc(ind);
        for (int i = -obstacle_size; i < obstacle_size; i++) {
            for (int j = -obstacle_size; j < obstacle_size; j++) {
                int current_r = rc[0] + i;
                int current_c = rc[1] + j;
                int current_ind = rc_2_ind(current_r, current_c);
                current_map_.data[current_ind] = 100;
            }
        }
        map_pub.publish(current_map_);
    }

    void clear_obs(int ind) {
        std::vector<int> rc = ind_2_rc(ind);
        for (int i = -obstacle_size; i < obstacle_size; i++) {
            for (int j = -obstacle_size; j < obstacle_size; j++) {
                int current_r = rc[0] + i;
                int current_c = rc[1] + j;
                int current_ind = rc_2_ind(current_r, current_c);
                current_map_.data[current_ind] = 0;
            }
        }
        map_pub.publish(current_map_);
    }

    double compute_steer_vel(double desired_angle, size_t i) {
        // get difference between current and desired
        double dif = (desired_angle - state_[i].steer_angle);

        // calculate velocity
        double steer_vel;
        if (std::abs(dif) > .0001) // if the difference is not trivial
            steer_vel = dif / std::abs(dif) * max_steering_vel_;
        else {
            steer_vel = 0;
        }

        return steer_vel;
    }

    // void compute_accel(double desired_velocity, size_t i) {
    //     // get difference between current and desired
    //     double dif = (desired_velocity - state_[i].velocity);

    //     if (state_[i].velocity > 0) {
    //         if (dif > 0) {
    //             // accelerate
    //             double kp = 2.0 * max_accel_ / max_speed_;
    //             set_accel(kp * dif);
    //         } else {
    //             // brake
    //             accel_[i] = -max_decel_;
    //         }
    //     } else {
    //         if (dif > 0) {
    //             // brake
    //             accel_[i] = max_decel_;

    //         } else {
    //             // accelerate
    //             double kp = 2.0 * max_accel_ / max_speed_;
    //             set_accel(kp * dif);
    //         }
    //     }
    // }

    /// ---------------------- CALLBACK FUNCTIONS ----------------------

    void obs_callback(const geometry_msgs::PointStamped &msg) {
        double x = msg.point.x;
        double y = msg.point.y;
        std::vector<int> rc = coord_2_cell_rc(x, y);
        int ind = rc_2_ind(rc[0], rc[1]);
        added_obs.push_back(ind);
        add_obs(ind);
    }

    // void pose_callback(const geometry_msgs::PoseStamped &msg) {
    //     state_[i].x = msg.pose.position.x;
    //     state_[i].y = msg.pose.position.y;
    //     geometry_msgs::Quaternion q = msg.pose.orientation;
    //     tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    //     state_[i].theta = tf2::impl::getYaw(quat);
    // }

    void pose_callback(const geometry_msgs::PoseStampedConstPtr &msg, int i) {
        state_[i].x = msg->pose.position.x;
        state_[i].y = msg->pose.position.y;
        geometry_msgs::Quaternion q = msg->pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        state_[i].theta = tf2::impl::getYaw(quat);
    }

    void pose_rviz_callback(
        const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose.pose;
        boost::shared_ptr<geometry_msgs::PoseStamped> shared_pose(
            &temp_pose, [](geometry_msgs::PoseStamped *) {});
        pose_callback(shared_pose, 0);
        // pose_callback(&temp_pose, i);
    }

    void opp_pose_rviz_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose;
        boost::shared_ptr<geometry_msgs::PoseStamped> shared_pose(
            &temp_pose, [](geometry_msgs::PoseStamped *) {});
        pose_callback(shared_pose, 1);
        // pose_callback(&temp_pose, i);
    }


    void drive_callback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg,
                        size_t i) {
        std::cout<<"received drive command \n";
        desired_speed_[i] = msg->drive.speed;
        desired_accel_[i] = msg->drive.acceleration;
        desired_steer_ang_[i] = msg->drive.steering_angle;
    }

    // button callbacks
    void clear_obstacles(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        bool clear_obs_clicked = false;
        if (feedback->event_type == 3) {
            clear_obs_clicked = true;
        }
        if (clear_obs_clicked) {
            ROS_INFO("Clearing obstacles.");
            current_map_ = original_map_;
            map_pub.publish(current_map_);

            clear_obs_clicked = false;
        }
    }

    void map_callback(const nav_msgs::OccupancyGrid &msg) {
        // Fetch the map parameters
        size_t height = msg.info.height;
        size_t width = msg.info.width;
        double resolution = msg.info.resolution;
        // Convert the ROS origin to a pose
        Pose2D origin;
        origin.x = msg.info.origin.position.x;
        origin.y = msg.info.origin.position.y;
        geometry_msgs::Quaternion q = msg.info.origin.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        origin.theta = tf2::impl::getYaw(quat);

        // Convert the map to probability values
        std::vector<double> map(msg.data.size());
        for (size_t i = 0; i < height * width; i++) {
            if (msg.data[i] > 100 or msg.data[i] < 0) {
                map[i] = 0.5; // Unknown
            } else {
                map[i] = msg.data[i] / 100.;
            }
        }

        // Send the map to the scanner
        scan_simulator.set_map(map, height, width, resolution, origin,
                               map_free_threshold);
        map_exists = true;
    }

    /// ---------------------- PUBLISHING HELPER FUNCTIONS
    /// ----------------------

    void pub_pose_transform(ros::Time timestamp, size_t i) {
        // Convert the pose into a transformation
        geometry_msgs::Transform t;
        t.translation.x = state_[i].x;
        t.translation.y = state_[i].y;
        tf2::Quaternion quat;
        quat.setEuler(0., 0., state_[i].theta);
        t.rotation.x = quat.x();
        t.rotation.y = quat.y();
        t.rotation.z = quat.z();
        t.rotation.w = quat.w();

        // Add a header to the transformation
        geometry_msgs::TransformStamped ts;
        ts.transform = t;
        ts.header.stamp = timestamp;
        ts.header.frame_id = map_frame;
        ts.child_frame_id = base_frame + std::to_string(i);

        // Publish them
        if (broadcast_transform) {
            br_.sendTransform(ts);
        }
    }

    void pub_steer_ang_transform(ros::Time timestamp, size_t i) {
        // Set the steering angle to make the wheels move
        // Publish the steering angle
        tf2::Quaternion quat_wheel;
        quat_wheel.setEuler(0., 0., state_[i].steer_angle);
        geometry_msgs::TransformStamped ts_wheel;
        ts_wheel.transform.rotation.x = quat_wheel.x();
        ts_wheel.transform.rotation.y = quat_wheel.y();
        ts_wheel.transform.rotation.z = quat_wheel.z();
        ts_wheel.transform.rotation.w = quat_wheel.w();
        ts_wheel.header.stamp = timestamp;
        ts_wheel.header.frame_id = "front_left_hinge" + std::to_string(i);
        ts_wheel.child_frame_id = "front_left_wheel" + std::to_string(i);
        br_.sendTransform(ts_wheel);
        ts_wheel.header.frame_id = "front_right_hinge" + std::to_string(i);
        ts_wheel.child_frame_id = "front_right_wheel" + std::to_string(i);
        br_.sendTransform(ts_wheel);
    }

    void pub_laser_link_transform(ros::Time timestamp, size_t i) {
        // Publish a transformation between base link and laser
        geometry_msgs::TransformStamped scan_ts;
        scan_ts.transform.translation.x = scan_distance_to_base_link_;
        scan_ts.transform.rotation.w = 1;
        scan_ts.header.stamp = timestamp;
        scan_ts.header.frame_id = base_frame + std::to_string(i);
        scan_ts.child_frame_id = scan_frame + std::to_string(i);
        br_.sendTransform(scan_ts);
    }

    void pub_odom(ros::Time timestamp, size_t i) {

        // Make an odom message and publish it
        nav_msgs::Odometry odom;
        odom.header.stamp = timestamp;
        odom.header.frame_id = map_frame;
        odom.child_frame_id = base_frame + std::to_string(i);
        odom.pose.pose.position.x = state_[i].x;
        odom.pose.pose.position.y = state_[i].y;
        tf2::Quaternion quat;
        quat.setEuler(0., 0., state_[i].theta);
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        odom.twist.twist.linear.x = state_[i].velocity;
        odom.twist.twist.angular.z = state_[i].angular_velocity;

        odom_pub_[i].publish(odom);
    }

    void pub_imu(ros::Time timestamp, size_t i) {
        // Make an IMU message and publish it
        // TODO: make imu message
        sensor_msgs::Imu imu;
        imu.header.stamp = timestamp;
        imu.header.frame_id = map_frame;

        imu_pub_[i].publish(imu);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "racecar_simulator");
    RacecarSimulator rs;
    ros::spin();
    return 0;
}
