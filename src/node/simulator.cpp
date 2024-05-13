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

#include <fstream>
#include <functional>
// #include <racecar_simulator/observation.h>
#include "control_msgs/CarState.h"
#include "control_msgs/reset.h"
#include "control_msgs/sync_control.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

using namespace racecar_simulator;
// using namespace mpcc;

enum ModelType { ORIGINALMODEL, PAJEKAMODEL };

std::vector<geometry_msgs::PointStamped> sampleWithoutReplacement(const std::vector<geometry_msgs::PointStamped> &data, int N) {
    if (N > data.size()) {
        throw std::runtime_error("N is greater than the size of the vector.");
    }

    std::vector<geometry_msgs::PointStamped> shuffledData = data; // 원본 데이터 복사
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(shuffledData.begin(), shuffledData.end(), g); // 데이터 섞기

    return std::vector<geometry_msgs::PointStamped>(shuffledData.begin(),
                                                    shuffledData.begin() + N); // 앞에서부터 N개의 원소 선택
}

void computeYaw(std::vector<geometry_msgs::PointStamped> &global_path) {

    for (size_t i = 0; i < global_path.size(); ++i) {
        // 다음 점을 가져옵니다. 마지막 점의 경우 첫 번째 점을 다음 점으로
        // 사용합니다.
        const geometry_msgs::Point &current = global_path[i].point;
        const geometry_msgs::Point &next = (i == global_path.size() - 1) ? global_path[0].point : global_path[i + 1].point;

        // 두 점 사이의 방향을 계산합니다.
        double dx = next.x - current.x;
        double dy = next.y - current.y;

        // atan2를 사용하여 yaw 값을 계산합니다.
        double yaw = atan2(dy, dx);

        global_path[i].point.z = yaw;
    }
}

std::vector<geometry_msgs::PointStamped> createRandomizedPath(const std::vector<geometry_msgs::PointStamped> &global_path) {
    std::vector<geometry_msgs::PointStamped> randomized_path;
    randomized_path.reserve(global_path.size());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.4,
                                         0.4); // -0.25 ~ 0.25 사이의 랜덤 값

    for (size_t i = 0; i < global_path.size(); ++i) {
        // 탄젠트(접선) 벡터 계산: 여기서는 간단한 예시로 다음 포인트 방향 사용
        // 실제로는 스플라인 등을 이용하여 정확한 탄젠트 벡터를 계산해야 함
        geometry_msgs::Vector3 tangent;
        if (i < global_path.size() - 1) {
            tangent.x = global_path[i + 1].point.x - global_path[i].point.x;
            tangent.y = global_path[i + 1].point.y - global_path[i].point.y;
        } else {
            tangent.x = global_path[i].point.x - global_path[i - 1].point.x;
            tangent.y = global_path[i].point.y - global_path[i - 1].point.y;
        }

        // 노멀(수직) 벡터 계산
        geometry_msgs::Vector3 normal;
        normal.x = -tangent.y;
        normal.y = tangent.x;
        normal.z = 0;

        // 랜덤 오프셋 생성 및 적용
        double randomOffset = dis(gen);
        geometry_msgs::PointStamped new_point = global_path[i];
        new_point.point.x += normal.x * randomOffset;
        new_point.point.y += normal.y * randomOffset;

        randomized_path.push_back(new_point);
    }

    return randomized_path;
}

class RacecarSimulator {
  private:
    // A ROS node
    ros::NodeHandle n;
    int obj_num_;
    // The transformation frames used
    std::string map_frame, base_frame, scan_frame;

    // obstacle states (1D index) and parameters
    std::vector<int> static_obs_idx;
    std::vector<int> dyn_obs_idx;
    // listen for clicked point for adding obstacles
    ros::Subscriber obs_sub;
    // ros::Subscriber reset_sub;
    int obstacle_size;

    // interactive markers' server
    interactive_markers::InteractiveMarkerServer im_server;

    // The car state and parameters
    std::vector<CarState> state_;
    //  std::vector<std::shared_ptr<IState>> states_;
    double previous_seconds;
    double init_time_;
    double scan_distance_to_base_link_;
    double max_speed_, max_steering_angle_;
    double max_accel_, max_steering_vel_, max_decel_;
    std::vector<double> desired_speed_, desired_steer_ang_, desired_accel_;
    std::vector<double> accel_, steer_angle_vel_;
    CarParams params_;
    double width_;

    // A simulator of the laser
    ScanSimulator2D scan_simulator_;
    double map_free_threshold;

    // For publishing transformations
    tf2_ros::TransformBroadcaster br_;

    // A timer to update the pose
    ros::Timer update_pose_timer;
    double iter_;

    // std::vector<ros::ServiceClient> client_;
    // ros::ServiceClient client_;

    // Listen for drive commands
    std::vector<ros::Subscriber> drive_sub_;

    // Listen for a map
    ros::Subscriber map_sub;
    bool map_exists = false;

    // Listen for updates to the pose
    std::vector<ros::Subscriber> pose_sub_;
    ros::Subscriber pose_rviz_sub_;
    ros::Subscriber opp_pose_rviz_sub_;
    ros::Subscriber observation_sub_;

    // synchronized mode
    double sync_time_step_; // in seconds, TODO : synchronize with MPC time
                            // step(now 0.025)
    bool synchronized_mode_;
    // double sync_time_;
    ros::ServiceServer reset_service_;
    ros::ServiceServer observation_server_;
    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    std::vector<ros::Publisher> scan_pub_;
    std::vector<ros::Publisher> state_pub_;
    std::vector<ros::Publisher> odom_pub_;
    std::vector<ros::Publisher> imu_pub_;
    std::vector<ros::Publisher> noise_pose_pub_;

    // publisher for map with obstacles
    ros::Publisher map_pub_;

    // collision publisher
    ros::Publisher collision_pub_;

    // ros time publisher
    ros::Publisher time_pub_;

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
    bool is_collision_;
    bool restart_mode_;
    bool noise_mode_;
    double pose_noise_;
    double yaw_noise_;
    double vel_noise_;
    int model_type_;
    std::vector<bool> obj_collision_;
    bool random_pose_;
    std::vector<geometry_msgs::PointStamped> global_path_;
    std::string control_mode_;

    std::vector<std::vector<geometry_msgs::Point>> obs_corner_pts_;
    std::vector<double> min_scan_distances_; // minumum distance between wall and obj for each
                                             // obstacle same size with obj_num

  public:
    RacecarSimulator() : im_server("racecar_sim") {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        clearvector();

        previous_seconds = ros::Time::now().toSec();

        // Get the topic names
        std::string drive_topic, map_topic, scan_topic, pose_topic, state_topic, pose_rviz_topic, opp_pose_rviz_topic, odom_topic, imu_topic;
        n.getParam("drive_topic", drive_topic);
        n.getParam("map_topic", map_topic);
        n.getParam("scan_topic", scan_topic);
        n.getParam("pose_topic", pose_topic);
        n.getParam("state_topic", state_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("pose_rviz_topic", pose_rviz_topic);
        n.getParam("opp_pose_rviz_topic", opp_pose_rviz_topic);
        n.getParam("imu_topic", imu_topic);
        n.getParam("sync_time_step", sync_time_step_);

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
        n.getParam("B", params_.B);
        n.getParam("C", params_.C);
        n.getParam("D", params_.D);
        n.getParam("width", width_);

        // clip velocity
        n.getParam("speed_clip_diff", speed_clip_diff);

        // Determine if we should broadcast
        n.getParam("broadcast_transform", broadcast_transform);

        // Get obstacle size parameter
        n.getParam("obstacle_size", obstacle_size);
        n.getParam("random_pose", random_pose_);
        n.getParam("control_mode", control_mode_);

        // synchoronized mode
        n.getParam("synchronized_mode", synchronized_mode_);
        n.getParam("restart_mode", restart_mode_);
        n.getParam("noise_mode", noise_mode_);
        n.getParam("pose_noise", pose_noise_);
        n.getParam("yaw_noise", yaw_noise_);
        n.getParam("vel_noise", vel_noise_);
        n.getParam("model_type", model_type_);
        std::string param_path;
        std::string cost_path;
        std::string bounds_path;
        std::string track_path;
        std::string normalization_path;
        n.getParam("param_path", param_path);
        n.getParam("cost_path", cost_path);
        n.getParam("bounds_path", bounds_path);
        n.getParam("track_path", track_path);
        n.getParam("normalization_path", normalization_path);
        // n.getParam("json_paths", json_paths);

        // integrator(update_pose_rate, json_paths);
        is_collision_ = false;
        std::vector<geometry_msgs::PointStamped> random_pose_array;
        if (random_pose_) {
            std::ifstream read_file;
            std::string waypoint_file;
            n.getParam("/waypoint_file", waypoint_file);
            std::cout << "waypoint file : " << waypoint_file << std::endl;
            read_file.open(waypoint_file.c_str());

            if (read_file.is_open()) {
                while (!read_file.eof()) {
                    std::string line;
                    geometry_msgs::PointStamped pos;

                    std::string buffer;
                    std::vector<std::string> res;

                    getline(read_file, line);
                    if (line.empty()) {
                        continue;
                    }

                    std::istringstream iss(line);

                    while (getline(iss, buffer, ',')) {
                        res.push_back(buffer);
                    }

                    pos.point.x = std::stof(res[0]);
                    pos.point.y = std::stof(res[1]);

                    global_path_.push_back(pos);
                }
            }
            computeYaw(global_path_);
            std::cout << "waypoint size for random pose :" << global_path_.size() << std::endl;
            random_pose_array = sampleWithoutReplacement(global_path_, obj_num_);
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        // Initialize car state and driving commands
        for (int i = 0; i < obj_num_; i++) {
            if (random_pose_) {
                CarState state = {
                    .x = random_pose_array[i].point.x,
                    .y = random_pose_array[i].point.y,
                    .theta = random_pose_array[i].point.z,
                    .velocity = 0,
                    .steer_angle = 0.0,
                    .angular_velocity = 0.0,
                    .slip_angle = 0.0,
                };
                state_.push_back(state);

            } else {
                CarState state = {
                    .x = i,
                    .y = i,
                    .theta = 0,
                    .velocity = 0,
                    .steer_angle = 0.0,
                    .angular_velocity = 0.0,
                    .slip_angle = 0.0,
                };
                state_.push_back(state);
            }

            accel_.push_back(0.0);
            steer_angle_vel_.push_back(0.0);
            desired_speed_.push_back(0.0);
            desired_steer_ang_.push_back(0.0);
            desired_accel_.push_back(0.0);

            obj_collision_.push_back(false);
        }

        // Initialize a simulator of the laser scanner
        scan_simulator_ = ScanSimulator2D(scan_beams, scan_fov, scan_std_dev);

        // Make a publisher for publishing map with obstacles
        map_pub_ = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

        // publish the original
        // synchronized_mode_ = true;
        if (synchronized_mode_) {
            // sync_time_ = 0.;

            observation_server_ = n.advertiseService("/sync_control_service", &RacecarSimulator::SyncControlServer, this);
            // observation_sub_ =
            //     n.subscribe(drive_topic + std::to_string(0), 1,
            //                 &RacecarSimulator::ObservationCallback, this);
            // observation_server_ = n.advertiseService(
            //     "/observation_service",
            //     &RacecarSimulator::ObservationCallback, this);
        } else {
            // Start a timer to output the pose
            update_pose_timer = n.createTimer(ros::Duration(update_pose_rate), &RacecarSimulator::update_pose, this);
        }

        // pose_rviz_sub_ =
        // n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(pose_rviz_topic,
        // 1, &RacecarSimulator::pose_rviz_callback, this);
        pose_rviz_sub_ = n.subscribe(pose_rviz_topic, 1, &RacecarSimulator::pose_rviz_callback,
                                     this); // ego vehicle pose
        opp_pose_rviz_sub_ = n.subscribe(opp_pose_rviz_topic, 1, &RacecarSimulator::opp_pose_rviz_callback,
                                         this); // opponent vehicle pose

        reset_service_ = n.advertiseService("/reset_service", &RacecarSimulator::reset_server, this);

        // Start a subscriber to listen to drive commands
        for (int i = 0; i < obj_num_; i++) {
            ros::Subscriber drive_sub, pose_sub;
            ros::Publisher scan_pub, odom_pub, imu_pub, state_pub, noise_pose_pub;
            // ros::ServiceClient client;
            if (synchronized_mode_) {
                if (i != 0) {
                    drive_sub = n.subscribe<ackermann_msgs::AckermannDriveStamped>(
                        drive_topic + std::to_string(i), 1,
                        boost::bind(&RacecarSimulator::drive_callback, this, _1, i)); // TODO : now we can
                                                                                      // syncrhonize only to 0 agent
                }

            } else {
                init_time_ = ros::Time::now().toSec();
                drive_sub = n.subscribe<ackermann_msgs::AckermannDriveStamped>(drive_topic + std::to_string(i), 1,
                                                                               boost::bind(&RacecarSimulator::drive_callback, this, _1, i));
            }
            state_pub = n.advertise<control_msgs::CarState>(state_topic + std::to_string(i), 1);

            pose_sub = n.subscribe<geometry_msgs::PoseStamped>(pose_topic + std::to_string(i), 1,
                                                               boost::bind(&RacecarSimulator::pose_callback, this, _1, i));

            // Make a publisher for laser scan messages
            scan_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic + std::to_string(i), 1);

            // Make a publisher for odometry messages
            odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic + std::to_string(i), 1);

            // Make a publisher for IMU messages
            imu_pub = n.advertise<sensor_msgs::Imu>(imu_topic + std::to_string(i), 1);

            if (noise_mode_) {
                noise_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/noised_pose" + std::to_string(i), 1);
            }
            // colision publisher
            collision_pub_ = n.advertise<std_msgs::Bool>("/collision", 1);
            time_pub_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

            // std::string service_name = "/collision_service" +
            // std::to_string(i); fprintf(stderr, "%s\n", service_name.c_str());
            // client = n.serviceClient<collision_msgs::collision>(
            //     service_name.c_str());

            // client_.push_back(client);
            drive_sub_.push_back(drive_sub);
            state_pub_.push_back(state_pub);
            // pose_sub_.push_back(pose_sub);
            // pose_rviz_sub_.push_back(pose_rviz_sub);
            noise_pose_pub_.push_back(noise_pose_pub);
            scan_pub_.push_back(scan_pub);
            odom_pub_.push_back(odom_pub);
            imu_pub_.push_back(imu_pub);
            obs_corner_pts_.push_back(std::vector<geometry_msgs::Point>());
        }

        // Start a subscriber to listen to new maps
        map_sub = n.subscribe(map_topic, 1, &RacecarSimulator::map_callback, this);

        // obstacle subscriber
        obs_sub = n.subscribe("/clicked_point", 1, &RacecarSimulator::obs_callback, this);

        // reset_sub =
        //     n.subscribe("/RL_reset", 1, &RacecarSimulator::reset_callback,
        //     this);

        // get collision safety margin
        // n.getParam("coll_threshold", thresh);
        // n.getParam("ttc_threshold", ttc_threshold);

        scan_ang_incr = scan_simulator_.get_angle_increment();
        // TODO : if the vehicle model of multiple vehicles is different,
        //        the following code should be modified.
        cosines = Precompute::get_cosines(scan_beams, -scan_fov / 2.0, scan_ang_incr);
        car_distances =
            Precompute::get_car_distances(scan_beams, params_.wheelbase, width_, scan_distance_to_base_link_, -scan_fov / 2.0, scan_ang_incr);
        //----------------------------

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
        clear_obs_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
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
        im_server.setCallback(clear_obs_button.name, boost::bind(&RacecarSimulator::clear_obstacles, this, _1));

        im_server.applyChanges();

        if (synchronized_mode_) {
            sleep(3.);
            RestartSimulation();
            ros::Time timestamp = ros::Time::now();

            for (size_t i = 0; i < obj_num_; i++) {
                pub_pose_transform(timestamp, i);

                /// Publish the steering angle as a transformation so the wheels
                pub_steer_ang_transform(timestamp, i);

                // Make an odom message as well and publish it
                pub_odom(timestamp, i);

                // TODO: make and publish IMU message
                pub_imu(timestamp, i);
            }
        }

        ROS_INFO("Simulator constructed.");
    }

    void visualizeTimeInRviz(double time) {
        visualization_msgs::Marker marker;

        // 마커 설정
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "time_visualization";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;

        // 시간 위치 및 스케일 설정
        marker.pose.position.x = -4.808;
        marker.pose.position.y = -4.331;
        marker.pose.position.z = 1.0;
        marker.scale.z = 1.0; // 텍스트 크기

        // 시간을 문자열로 변환 (최대 소수점 3자리)
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3) << time;
        marker.text = "Sim Time : " + ss.str();

        // 색상 및 기간 설정
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        // 마커 발행
        time_pub_.publish(marker);
    }

    void clearvector() {
        state_.clear();
        accel_.clear();
        steer_angle_vel_.clear();
        desired_speed_.clear();
        desired_steer_ang_.clear();
        desired_accel_.clear();
        // drive_sub_.clear();
        // scan_pub_.clear();
        // odom_pub_.clear();
        // imu_pub_.clear();
    }

    double compute_accel(double desired_velocity, int i) {
        // get difference between current and desired
        double dif = (desired_velocity - state_[i].velocity);

        double kp = 2.0 * max_accel_ / max_speed_;

        // calculate acceleration
        double acceleration = kp * dif;

        return acceleration;
    }

    void RestartSimulation() {
        // sync_time_ = 0.0;
        clearvector();
        iter_ = 0;
        for (size_t i = 0; i < obj_collision_.size(); i++)
            obj_collision_[i] = false;
        std::vector<geometry_msgs::PointStamped> random_pose_array;
        std::vector<geometry_msgs::PointStamped> fixed_pose_array;
        std::vector<geometry_msgs::PointStamped> randomized_path;
        if (random_pose_) {
            randomized_path = createRandomizedPath(global_path_);
            random_pose_array = sampleWithoutReplacement(randomized_path, obj_num_);
        }

        else {
            geometry_msgs::PointStamped msg;

            msg.point.x = 0.484;
            msg.point.y = -0.557;
            msg.point.z = -1.203;

            // msg.point.x = 0.260;
            // msg.point.y = -0.212;
            // msg.point.z = -0.671;
            fixed_pose_array.push_back(msg);

            // msg.point.x = 5.656;
            // msg.point.y = -3.4;
            // msg.point.z = 1.942;
            // fixed_pose_array.push_back(msg);

            // msg.point.x = -3.261;
            // msg.point.y = -12.916;
            // msg.point.z = 0.288;
            // fixed_pose_array.push_back(msg);

            // the number of fixed pose should be same with obj_num_
            assert(fixed_pose_array.size() == obj_num_);
        }

        // Initialize car state and driving commands
        for (int i = 0; i < obj_num_; i++) {
            if (random_pose_) {
                CarState state = {
                    .x = random_pose_array[i].point.x,
                    .y = random_pose_array[i].point.y,
                    .theta = random_pose_array[i].point.z,
                    .velocity = 0,
                    .steer_angle = 0.0,
                    .angular_velocity = 0.0,
                    .slip_angle = 0.0,
                };
                state_.push_back(state);

            } else {
                CarState state = {
                    .x = fixed_pose_array[i].point.x,
                    .y = fixed_pose_array[i].point.y,
                    .theta = fixed_pose_array[i].point.z,
                    .velocity = 0,
                    .steer_angle = 0.0,
                    .angular_velocity = 0.0,
                    .slip_angle = 0.0,
                };
                state_.push_back(state);
            }

            accel_.push_back(0.0);
            steer_angle_vel_.push_back(0.0);
            desired_speed_.push_back(0.0);
            desired_steer_ang_.push_back(0.0);
            desired_accel_.push_back(0.0);
        }
    }

    void UpdateObstaclePosition(int i) {
        for (size_t idx = 0; idx < obj_num_; idx++) {
            if (idx == i)
                continue;
            double x = state_[idx].x;
            double y = state_[idx].y;
            std::vector<int> rc = coord_2_cell_rc(x, y);
            // int ind = rc_2_ind(rc[0], rc[1]);
            // std::vector<int> rc = ind_2_rc(ind);
            int current_r = rc[0];
            int current_c = rc[1];
            int current_ind = rc_2_ind(current_r, current_c);
            // current_map_.data[current_ind] = 100;
            scan_simulator_.addOccupancyGrid(current_ind,
                                             0); // 0 : occupied , 99999 : Free
        }
        scan_simulator_.updateDistance();

        // double x = msg.point.x;
        // double y = msg.point.y;
        // std::vector<int> rc = coord_2_cell_rc(x, y);
        // added_obs.push_back(ind);
        // add_obs(ind);
    }
    bool SyncControlServer(control_msgs::sync_control::Request &req, control_msgs::sync_control::Response &res) {
        desired_accel_[0] = req.control_input.data[0];
        desired_steer_ang_[0] = req.control_input.data[1];

        // obs_corner_pts_.clear();
        min_scan_distances_.clear();

        // Update the pose
        ros::Time timestamp = ros::Time::now();
        // fprintf(stderr, "timestamp : %f\n", timestamp.toSec());
        // simulate P controller
        double current_seconds = timestamp.toSec();

        for (int i = 0; i < obj_num_; i++) {

            if (control_mode_ == "v") {
                if (std::isnan(desired_speed_[i]))
                    desired_speed_[i] = 0.0;
                double accel = compute_accel(desired_speed_[i], i);
                set_accel(desired_accel_[i], i);
            } else if (control_mode_ == "a") {
                if (std::isnan(desired_accel_[i]))
                    desired_accel_[i] = 0.0;
                set_accel(desired_accel_[i], i);
            } else
                ROS_INFO("control mode error");
            set_steer_angle_vel(compute_steer_vel(desired_steer_ang_[i], i), i);

            if (!obj_collision_[i]) {
                state_[i] = STKinematics::update(state_[i], accel_[i], steer_angle_vel_[i], params_, sync_time_step_);
                state_[i].velocity = std::min(std::max(state_[i].velocity, -max_speed_), max_speed_);
                state_[i].steer_angle = std::min(std::max(state_[i].steer_angle, -max_steering_angle_), max_steering_angle_);
            }

            // previous_seconds = current_seconds;

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
                scan_pose.x = state_[i].x + scan_distance_to_base_link_ * std::cos(state_[i].theta);
                scan_pose.y = state_[i].y + scan_distance_to_base_link_ * std::sin(state_[i].theta);
                scan_pose.theta = state_[i].theta;

                getConerPoint(i);

                // Update the oppenent vehicle pose to include in scan
                // UpdateObstaclePosition(i);

                // Compute the scan from the lidar
                std::vector<double> scan = scan_simulator_.scan(scan_pose); // scan : distance from lidar to obstacle

                // Convert to float
                std::vector<float> scan_float(scan.size());
                for (size_t idx = 0; idx < scan.size(); idx++)
                    scan_float[idx] = scan[idx];

                // TTC Calculations are done here so the car can be halted in
                // the simulator: to reset TTC
                bool no_collision = true;
                double min_scan = *std::min_element(scan_float.begin(), scan_float.end());
                min_scan_distances_.push_back(min_scan);

                // Publish the laser message
                sensor_msgs::LaserScan scan_msg;
                scan_msg.header.stamp = timestamp;
                scan_msg.header.frame_id = scan_frame + std::to_string(i);
                scan_msg.angle_min = -scan_simulator_.get_field_of_view() / 2.;
                scan_msg.angle_max = scan_simulator_.get_field_of_view() / 2.;
                scan_msg.angle_increment = scan_simulator_.get_angle_increment();
                scan_msg.range_max = 100;
                scan_msg.ranges = scan_float;
                scan_msg.intensities = scan_float;
                scan_pub_[i].publish(scan_msg);

                // Publish a transformation between base link and laser
                pub_laser_link_transform(timestamp, i);
            }

            control_msgs::CarState state;
            state.x = state_[i].x;
            state.y = state_[i].y;
            state.theta = state_[i].theta;
            state.velocity = state_[i].velocity;
            state.steer_angle = state_[i].steer_angle;
            state.angular_velocity = state_[i].angular_velocity;
            state.slip_angle = state_[i].slip_angle;

            res.state.push_back(state);
        }
        iter_++;
        visualizeTimeInRviz(iter_ * sync_time_step_);

        bool curr_collision = checkAllCollisions(obs_corner_pts_);
        // double g = 9.81;
        // double rear_val = g * params_.l_r - desired_accel_[0] * params_.h_cg;
        // double front_val = g * params_.l_f + desired_steer_ang_[0] *
        // params_.h_cg;

        // double ay =
        //     state_[0].velocity *
        //     (params_.friction_coeff / (state_[0].velocity * (params_.l_r +
        //     params_.l_f))) * (params_.cs_f * desired_steer_ang_[0] * rear_val
        //     -
        //      state_[0].slip_angle * (params_.cs_r * front_val + params_.cs_f
        //      * rear_val) + (state_[0].angular_velocity / state_[0].velocity)
        //      * (params_.cs_r * params_.l_r * front_val -
        //                                      params_.cs_f * params_.l_f *
        //                                      rear_val));
        // if(isnan(ay))
        //     ay = 0.0;
        // if (fabs(ay) > g * params_.friction_coeff) {
        //     curr_collision = true;
        //     std::cout << " ay exceed limit \n"
        //                 << " ay : " << ay << std::endl;
        // }

        res.collision = curr_collision;

        if (curr_collision != is_collision_) {
            is_collision_ = curr_collision;
            std_msgs::Bool is_collision;
            is_collision.data = is_collision_;
            if (is_collision_)
                collision_pub_.publish(is_collision);
        }
        return true;
    }

    void ObservationCallback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg) {
        // obs_corner_pts_.clear();
        min_scan_distances_.clear();
        // sync_time_ += 1.0;

        std::cout << "ego received drive command \n";
        desired_speed_[0] = msg->drive.speed;
        desired_accel_[0] = msg->drive.acceleration;
        desired_steer_ang_[0] = msg->drive.steering_angle;

        // Update the pose
        ros::Time timestamp = ros::Time::now();
        // fprintf(stderr, "timestamp : %f\n", timestamp.toSec());
        // simulate P controller
        for (int i = 0; i < obj_num_; i++) {
            if (control_mode_ == "v") {
                if (std::isnan(desired_speed_[i]))
                    desired_speed_[i] = 0.0;
                double accel = compute_accel(desired_speed_[i], i);
                set_accel(desired_accel_[i], i);
            } else if (control_mode_ == "a") {
                if (std::isnan(desired_accel_[i]))
                    desired_accel_[i] = 0.0;
                set_accel(desired_accel_[i], i);
            } else
                ROS_INFO("control mode error");
            set_steer_angle_vel(compute_steer_vel(desired_steer_ang_[i], i), i);

            // double current_seconds = timestamp.toSec();
            state_[i] = STKinematics::update(state_[i], accel_[i], steer_angle_vel_[i], params_, sync_time_step_);
            state_[i].velocity = std::min(std::max(state_[i].velocity, -max_speed_), max_speed_);
            state_[i].steer_angle = std::min(std::max(state_[i].steer_angle, -max_steering_angle_), max_steering_angle_);

            // previous_seconds = current_seconds;

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
                scan_pose.x = state_[i].x + scan_distance_to_base_link_ * std::cos(state_[i].theta);
                scan_pose.y = state_[i].y + scan_distance_to_base_link_ * std::sin(state_[i].theta);
                scan_pose.theta = state_[i].theta;

                getConerPoint(i);

                // Update the oppenent vehicle pose to include in scan
                // UpdateObstaclePosition(i);

                // Compute the scan from the lidar
                std::vector<double> scan = scan_simulator_.scan(scan_pose); // scan : distance from lidar to obstacle

                // Convert to float
                std::vector<float> scan_float(scan.size());
                for (size_t idx = 0; idx < scan.size(); idx++)
                    scan_float[idx] = scan[idx];

                // TTC Calculations are done here so the car can be halted in
                // the simulator: to reset TTC
                bool no_collision = true;
                double min_scan = *std::min_element(scan_float.begin(), scan_float.end());
                min_scan_distances_.push_back(min_scan);

                // Publish the laser message
                sensor_msgs::LaserScan scan_msg;
                scan_msg.header.stamp = timestamp;
                scan_msg.header.frame_id = scan_frame + std::to_string(i);
                scan_msg.angle_min = -scan_simulator_.get_field_of_view() / 2.;
                scan_msg.angle_max = scan_simulator_.get_field_of_view() / 2.;
                scan_msg.angle_increment = scan_simulator_.get_angle_increment();
                scan_msg.range_max = 100;
                scan_msg.ranges = scan_float;
                scan_msg.intensities = scan_float;
                scan_pub_[i].publish(scan_msg);

                // Publish a transformation between base link and laser
                pub_laser_link_transform(timestamp, i);
            }
        }

        bool curr_collision = checkAllCollisions(obs_corner_pts_);
        if (curr_collision != is_collision_) {
            is_collision_ = curr_collision;
            std_msgs::Bool is_collision;
            is_collision.data = is_collision_;
            if (is_collision_)
                collision_pub_.publish(is_collision);
            // if (is_collision_) {
            //     sleep(2.);
            //     RestartSimulation();
            //     ros::Time timestamp0 = ros::Time::now();

            //     for (size_t i = 0; i < obj_num_; i++) {
            //         pub_pose_transform(timestamp0, i);

            //         /// Publish the steering angle as a transformation so the
            //         /// wheels
            //         pub_steer_ang_transform(timestamp0, i);

            //         // Make an odom message as well and publish it
            //         pub_odom(timestamp0, i);

            //         // TODO: make and publish IMU message
            //         pub_imu(timestamp0, i);
            //     }
            // }
        }
    }

    void update_pose(const ros::TimerEvent &) {

        obs_corner_pts_.clear();
        min_scan_distances_.clear();
        ros::Time timestamp = ros::Time::now();
        double current_seconds = timestamp.toSec();
        // simulate P controller
        for (int i = 0; i < obj_num_; i++) {
            if (control_mode_ == "v") {
                if (std::isnan(desired_speed_[i]))
                    desired_speed_[i] = 0.0;
                double accel = compute_accel(desired_speed_[i], i);
                set_accel(accel, i);
            } else if (control_mode_ == "a") {
                if (std::isnan(desired_accel_[i]))
                    desired_accel_[i] = 0.0;
                set_accel(desired_accel_[i], i);
            } else
                ROS_INFO("control mode error");
            set_steer_angle_vel(compute_steer_vel(desired_steer_ang_[i], i), i);

            // Update the pose

            // Update the state
            if (model_type_ == ORIGINALMODEL)
                state_[i] = STKinematics::update(state_[i], accel_[i], steer_angle_vel_[i], params_, 0.001);
            else if (model_type_ == PAJEKAMODEL)
                state_[i] = STKinematics::update_with_pacejka(state_[i], accel_[i], steer_angle_vel_[i], params_, 0.001);
            else {
                state_[i] = STKinematics::update(state_[i], accel_[i], steer_angle_vel_[i], params_, 0.001);
            }

            state_[i].velocity = std::min(std::max(state_[i].velocity, -max_speed_), max_speed_);
            state_[i].steer_angle = std::min(std::max(state_[i].steer_angle, -max_steering_angle_), max_steering_angle_);

            /// Publish the pose as a transformation
            pub_pose_transform(timestamp, i);

            /// Publish the steering angle as a transformation so the wheels
            pub_steer_ang_transform(timestamp, i);

            pub_state(timestamp, i);

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
                scan_pose.x = state_[i].x + scan_distance_to_base_link_ * std::cos(state_[i].theta);
                scan_pose.y = state_[i].y + scan_distance_to_base_link_ * std::sin(state_[i].theta);
                scan_pose.theta = state_[i].theta;

                getConerPoint(i);

                // Update the oppenent vehicle pose to include in scan
                // UpdateObstaclePosition(i);

                // Compute the scan from the lidar
                std::vector<double> scan = scan_simulator_.scan(scan_pose);

                // Convert to float
                std::vector<float> scan_float(scan.size());
                for (size_t idx = 0; idx < scan.size(); idx++)
                    scan_float[idx] = scan[idx];

                // TTC Calculations are done here so the car can be halted in
                // the simulator: to reset TTC
                bool no_collision = true;
                // if (state_[i].velocity != 0) {
                //     // scan_float minest value

                //     for (size_t idx = 0; idx < scan_float.size(); idx++) {
                //         // TTC calculations

                //         // calculate projected velocity
                //         double proj_velocity =
                //             state_[i].velocity * cosines[idx];
                //         double ttc = (scan_float[idx] - car_distances[idx]) /
                //                      proj_velocity; // ???
                //         // if it's small enough to count as a collision
                //         if ((ttc < ttc_threshold) && (ttc >= 0.0)) {
                //             // if (!TTC) {
                //             //     first_ttc_actions();
                //             // }

                //             no_collision = false;
                //             TTC = true;

                //             ROS_INFO("Collision detected");
                //         }
                //     }
                // }
                double min_scan = *std::min_element(scan_float.begin(), scan_float.end());
                min_scan_distances_.push_back(min_scan);

                // reset TTC
                // if (no_collision)
                //     TTC = false;

                // Publish the laser message
                sensor_msgs::LaserScan scan_msg;
                scan_msg.header.stamp = timestamp;
                scan_msg.header.frame_id = scan_frame + std::to_string(i);
                scan_msg.angle_min = -scan_simulator_.get_field_of_view() / 2.;
                scan_msg.angle_max = scan_simulator_.get_field_of_view() / 2.;
                scan_msg.angle_increment = scan_simulator_.get_angle_increment();
                scan_msg.range_max = 100;
                scan_msg.ranges = scan_float;
                scan_msg.intensities = scan_float;
                scan_pub_[i].publish(scan_msg);

                // Publish a transformation between base link and laser
                pub_laser_link_transform(timestamp, i);
            }
        }
        previous_seconds = current_seconds;

        visualizeTimeInRviz(current_seconds - init_time_);

        bool curr_collision = checkAllCollisions(obs_corner_pts_);
        if (curr_collision != is_collision_) {
            is_collision_ = curr_collision;
            std_msgs::Bool is_collision;
            is_collision.data = is_collision_;
            if (is_collision_)
                collision_pub_.publish(is_collision);
            if (is_collision_ && restart_mode_) {
                RestartSimulation();
            }
        }

        // if (is_collision_) {
        //     collision_msgs::collision srv;
        //     srv.request.is_collision = is_collision_;
        //     for (size_t k = 0; k < obj_num_; k++) {
        //         if (!client_[k].call(srv)) {
        //             fprintf(stderr,
        //                     "Failed to call service collision_service\n");
        //         }
        //     }

        //     RestartSimulation();
        //     is_collision_ = false;
        //     for (size_t k = 0; k < obj_num_; k++) {
        //         if (!client_[k].call(srv)) {
        //             fprintf(stderr, "Failed to call restart service\n");
        //         }
        //     }
        // }

    } // end of update_pose

    void pub_state(ros::Time timestamp, size_t i) {
        control_msgs::CarState state;
        // state.header.stamp = timestamp;
        // state.header.frame_id = "state"

        state.x = state_[i].x;
        state.y = state_[i].y;
        state.theta = state_[i].theta;
        state.velocity = state_[i].velocity;
        state.steer_angle = state_[i].steer_angle;
        state.angular_velocity = state_[i].angular_velocity;
        state.slip_angle = state_[i].slip_angle;

        state_pub_[i].publish(state);
    }

    /**
     * @brief Get the Coner Point object
     *
     * @param i : index of vehicle
     */
    void getConerPoint(int i) {
        std::vector<geometry_msgs::Point> corner_pts;
        geometry_msgs::Point p1, p2, p3, p4;

        p1.x = state_[i].x + params_.wheelbase / 2.0 * cos(state_[i].theta) - width_ / 2.0 * sin(state_[i].theta);
        p1.y = state_[i].y + params_.wheelbase / 2.0 * sin(state_[i].theta) + width_ / 2.0 * cos(state_[i].theta);

        p2.x = state_[i].x + params_.wheelbase / 2.0 * cos(state_[i].theta) + width_ / 2.0 * sin(state_[i].theta);
        p2.y = state_[i].y + params_.wheelbase / 2.0 * sin(state_[i].theta) - width_ / 2.0 * cos(state_[i].theta);

        p3.x = state_[i].x - params_.wheelbase / 2.0 * cos(state_[i].theta) + width_ / 2.0 * sin(state_[i].theta);
        p3.y = state_[i].y - params_.wheelbase / 2.0 * sin(state_[i].theta) - width_ / 2.0 * cos(state_[i].theta);

        p4.x = state_[i].x - params_.wheelbase / 2.0 * cos(state_[i].theta) - width_ / 2.0 * sin(state_[i].theta);
        p4.y = state_[i].y - params_.wheelbase / 2.0 * sin(state_[i].theta) + width_ / 2.0 * cos(state_[i].theta);

        corner_pts.push_back(p1);
        corner_pts.push_back(p2);
        corner_pts.push_back(p3);
        corner_pts.push_back(p4);

        obs_corner_pts_[i] = corner_pts;
    }

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

    void set_accel(double accel, int i) { accel_[i] = std::min(std::max(accel, -max_accel_), max_accel_); }

    void set_steer_angle_vel(double steer_angle_vel, int i) {
        steer_angle_vel_[i] = std::min(std::max(steer_angle_vel, -max_steering_vel_), max_steering_vel_);
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
        map_pub_.publish(current_map_);
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
        map_pub_.publish(current_map_);
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
    /**
     * rviz clicked point callback
     */
    void obs_callback(const geometry_msgs::PointStamped &msg) {
        double x = msg.point.x;
        double y = msg.point.y;
        std::vector<int> rc = coord_2_cell_rc(x, y);
        int ind = rc_2_ind(rc[0], rc[1]);
        static_obs_idx.push_back(ind);
        add_obs(ind);
    }

    bool reset_server(control_msgs::reset::Request &req, control_msgs::reset::Response &res) {
        fprintf(stderr, "reset callback \n");
        // sleep(2.);
        RestartSimulation();
        ros::Time timestamp0 = ros::Time::now();

        for (size_t i = 0; i < obj_num_; i++) {
            pub_pose_transform(timestamp0, i);

            /// Publish the steering angle as a transformation so the
            /// wheels
            pub_steer_ang_transform(timestamp0, i);

            // Make an odom message as well and publish it
            pub_odom(timestamp0, i);

            // TODO: make and publish IMU message
            pub_imu(timestamp0, i);
        }

        for (size_t i = 0; i < obj_num_; i++) {
            control_msgs::CarState state;
            state.x = state_[i].x;
            state.y = state_[i].y;
            state.theta = state_[i].theta;
            state.velocity = state_[i].velocity;
            state.steer_angle = state_[i].steer_angle;
            state.angular_velocity = state_[i].angular_velocity;
            state.slip_angle = state_[i].slip_angle;

            res.state.push_back(state);
        }

        return true;
    }

    // void reset_callback(const std_msgs::Bool &msg) {
    //     std::cout<<(msg.data);
    //     fprintf(stderr, "reset callback \n");
    //     sleep(2.);
    //     RestartSimulation();
    //     ros::Time timestamp0 = ros::Time::now();

    //     for (size_t i = 0; i < obj_num_; i++) {
    //         pub_pose_transform(timestamp0, i);

    //         /// Publish the steering angle as a transformation so the
    //         /// wheels
    //         pub_steer_ang_transform(timestamp0, i);

    //         // Make an odom message as well and publish it
    //         pub_odom(timestamp0, i);

    //         // TODO: make and publish IMU message
    //         pub_imu(timestamp0, i);
    //     }
    // }

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

    void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose.pose;
        boost::shared_ptr<geometry_msgs::PoseStamped> shared_pose(&temp_pose, [](geometry_msgs::PoseStamped *) {});
        pose_callback(shared_pose, 0);
        state_[0].velocity = 0.0;
        state_[0].angular_velocity = 0.0;
        state_[0].slip_angle = 0.0;

        desired_accel_[0] = 0.0;
        desired_steer_ang_[0] = 0.0;
        desired_speed_[0] = 0.0;

        // pose_callback(&temp_pose, i);
    }

    void opp_pose_rviz_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose;
        boost::shared_ptr<geometry_msgs::PoseStamped> shared_pose(&temp_pose, [](geometry_msgs::PoseStamped *) {});
        pose_callback(shared_pose, 1);
        state_[1].velocity = 0.0;

        // pose_callback(&temp_pose, i);
    }

    void drive_callback(const ackermann_msgs::AckermannDriveStampedConstPtr &msg, size_t i) {
        std::cout << "received drive command \n";
        desired_speed_[i] = msg->drive.speed;
        desired_accel_[i] = msg->drive.acceleration;
        desired_steer_ang_[i] = msg->drive.steering_angle;
    }

    // button callbacks
    void clear_obstacles(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        bool clear_obs_clicked = false;
        if (feedback->event_type == 3) {
            clear_obs_clicked = true;
        }
        if (clear_obs_clicked) {
            ROS_INFO("Clearing obstacles.");
            current_map_ = original_map_;
            map_pub_.publish(current_map_);

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
        scan_simulator_.set_map(map, height, width, resolution, origin, map_free_threshold);
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
        double yaw_noise, x_noise, y_noise, vel_noise;
        if (noise_mode_) {
            x_noise = pose_noise_ * (rand() % 2 - 1);
            y_noise = pose_noise_ * (rand() % 2 - 1);
            yaw_noise = yaw_noise_ * (rand() % 2 - 1);
            vel_noise = vel_noise_ * (rand() % 2 - 1);
        } else {
            x_noise = 0.0;
            y_noise = 0.0;
            yaw_noise = 0.0;
            vel_noise = 0.0;
        }

        // Make an odom message and publish it
        nav_msgs::Odometry odom;
        odom.header.stamp = timestamp;
        odom.header.frame_id = map_frame;
        odom.child_frame_id = base_frame + std::to_string(i);
        odom.pose.pose.position.x = state_[i].x + x_noise;
        odom.pose.pose.position.y = state_[i].y + y_noise;
        tf2::Quaternion quat;
        quat.setEuler(0., 0., state_[i].theta + yaw_noise);
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        odom.twist.twist.linear.x = state_[i].velocity + vel_noise;
        odom.twist.twist.angular.z = state_[i].angular_velocity;

        if (std::isnan(state_[i].x) || std::isnan(state_[i].y) || std::isnan(state_[i].theta)) {
            std::cout << "nan detected" << std::endl;
            return;
        }

        odom_pub_[i].publish(odom);

        if (noise_mode_) {
            // Publish the noise pose with noise_pose_pub_
            geometry_msgs::PoseStamped noise_pose;
            noise_pose.header.stamp = timestamp;
            noise_pose.header.frame_id = map_frame;
            noise_pose.pose.position.x = state_[i].x + x_noise;
            noise_pose.pose.position.y = state_[i].y + y_noise;
            noise_pose.pose.position.z = 0.0;
            noise_pose.pose.orientation.x = quat.x();
            noise_pose.pose.orientation.y = quat.y();
            noise_pose.pose.orientation.z = quat.z();
            noise_pose.pose.orientation.w = quat.w();
            noise_pose_pub_[i].publish(noise_pose);
        }
    }

    void pub_imu(ros::Time timestamp, size_t i) {
        // Make an IMU message and publish it
        // TODO: make imu message
        sensor_msgs::Imu imu;
        imu.header.stamp = timestamp;
        imu.header.frame_id = map_frame;

        imu_pub_[i].publish(imu);
    }

    bool checkCollision(const std::vector<geometry_msgs::Point> &polyA, const std::vector<geometry_msgs::Point> &polyB) {
        for (int shape = 0; shape < 2; shape++) {
            const std::vector<geometry_msgs::Point> &polygon = (shape == 0) ? polyA : polyB;

            for (size_t i = 0; i < polygon.size(); i++) {
                int j = (i + 1) % polygon.size();
                double normalX = polygon[j].y - polygon[i].y;
                double normalY = polygon[i].x - polygon[j].x;

                double minA = std::numeric_limits<double>::max();
                double maxA = std::numeric_limits<double>::min();
                for (const auto &point : polyA) {
                    double projected = normalX * point.x + normalY * point.y;
                    minA = std::min(minA, projected);
                    maxA = std::max(maxA, projected);
                }

                double minB = std::numeric_limits<double>::max();
                double maxB = std::numeric_limits<double>::min();
                for (const auto &point : polyB) {
                    double projected = normalX * point.x + normalY * point.y;
                    minB = std::min(minB, projected);
                    maxB = std::max(maxB, projected);
                }

                if (maxA < minB || maxB < minA) {
                    return false; // Separating axis found
                }
            }
        }

        return true; // No separating axis found, the polygons are colliding
    }

    bool checkAllCollisions(const std::vector<std::vector<geometry_msgs::Point>> &obs_corner_pts) {
        double thresh = width_ / 2.0;
        for (size_t i = 0; i < min_scan_distances_.size(); i++) { // wall collision check
            if (min_scan_distances_[i] < thresh) {
                // fprintf(stderr, "Collision detected\n");
                if (i == 0)      // collision reset occurs only when ego vehicle collides
                    return true; // Collision detected between two vehicles
                else {
                    obj_collision_[i] = true;
                }
            }
        }

        for (size_t i = 0; i < obs_corner_pts.size(); i++) {
            for (size_t j = i + 1; j < obs_corner_pts.size(); j++) {
                if (checkCollision(obs_corner_pts[i],
                                   obs_corner_pts[j])) { // vehicle to vehicle
                                                         // collision check
                    // fprintf(stderr, "Collision detected\n");
                    if (i == 0)      // collision reset occurs only when ego vehicle
                                     // collides
                        return true; // Collision detected between two vehicles
                    else {
                        obj_collision_[i] = true;
                        obj_collision_[j] = true;
                    }
                }
            }
        }
        return false; // No collisions detected
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "racecar_simulator");
    RacecarSimulator rs;
    ros::spin();
    return 0;
}
