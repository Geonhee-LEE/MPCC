// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////


#include "Tests/spline_test.h"
#include "Tests/model_integrator_test.h"
#include "Tests/constratins_test.h"
#include "Tests/cost_test.h"

#include "MPC/mpc.h"
#include "Model/integrator.h"
#include "Params/track.h"
#include "Plotting/plotting.h"

#include <nlohmann/json.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <syscon_msgs/RobotState.h>

using json = nlohmann::json;

nav_msgs::Path path_;
nav_msgs::Odometry odometry_;
syscon_msgs::RobotState robot_state;

void pathCallback(const nav_msgs::Path &path){

    std::cout << " pathCallback " << std::endl;
    path_ = path;
}
void rsStateCallback(const syscon_msgs::RobotState &data){

    //std::cout << " rsStateCallback " << std::endl;
    robot_state = data;
}
void odomStateCallback(const nav_msgs::Odometry &data){

    //std::cout << " rsStateCallback " << std::endl;
    odometry_ = data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_predictive_contouring_controller");
    ros::NodeHandle priv_n("~");
    ros::NodeHandle nh;
    ros::Subscriber path_sub_ = nh.subscribe("/R_001/move_base/WaypointsGlobalPlanner/global_plan", 1, pathCallback);
    ros::Subscriber rs_sub_ = nh.subscribe("/R_001/robot_state", 1, rsStateCallback);
    ros::Subscriber odom_sub_ = nh.subscribe("/R_001/odom", 1, odomStateCallback);

    using namespace mpcc;

    std::string const package_path = ros::package::getPath("model_predictive_contouring_control");
    std::ifstream iConfig(package_path + "/Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    PathToJson json_paths {jsonConfig["model_path"],
                           jsonConfig["cost_path"],
                           jsonConfig["bounds_path"],
                           jsonConfig["track_path"],
                           jsonConfig["normalization_path"]};

    int return_flag;

    return_flag = testSpline();
    std::cout << " Result of testSpline(): " << return_flag << std::endl;

    return_flag = testArcLengthSpline(json_paths);
    std::cout << " Result of testArcLengthSpline(): " << return_flag << std::endl;

    return_flag = testIntegrator(json_paths);
    std::cout << " Result of testIntegrator(): " <<  return_flag << std::endl;

    return_flag = testLinModel(json_paths);
    std::cout << " Result of testLinModel(): " << return_flag << std::endl;
    //std::cout << testAlphaConstraint(json_paths) << std::endl;
    //std::cout << testTireForceConstraint(json_paths) << std::endl;

    return_flag = testTrackConstraint(json_paths);
    std::cout << " Result of testTrackConstraint(): " << return_flag << std::endl;

    return_flag = testCost(json_paths);
    std::cout << " Result of cost(): " << return_flag << std::endl;

    Integrator integrator = Integrator(jsonConfig["Ts"], json_paths);
    Plotting plotter = Plotting(jsonConfig["Ts"], json_paths);

    Track track = Track(json_paths.track_path);

    ros::Rate wait(1.0);
    while(path_.poses.size() == 0 && ros::ok()){
        std::cout << " waiting for path cb" << std::endl;
        ros::spinOnce();
        wait.sleep();
    }
    std::vector<double> x, y;

    for(int i = 0; i < path_.poses.size(); i++){
        x.push_back(path_.poses[i].pose.position.x);
        y.push_back(path_.poses[i].pose.position.y);
    }
    track.setTrack(x, y); // Set ROS Global Path

    TrackPos track_xy = track.getTrack();

    std::list<MPCReturn> log;
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    mpc.setTrack(track_xy.X, track_xy.Y);
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    /*
    State x0 = {track_xy.X(0), track_xy.Y(0), phi_0, jsonConfig["v0"], 0,0};
    
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {
        MPCReturn mpc_sol = mpc.runMPC(x0);
        x0 = integrator.simTimeStep(x0, mpc_sol.u0, jsonConfig["Ts"]);
        log.push_back(mpc_sol);
    }
    */

    // ROS
    ros::Publisher path_pub_ = priv_n.advertise<nav_msgs::Path>("predicted_path", 1);
    ros::Publisher cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("R_001/cmd_vel", 1);
    nav_msgs::Path raw_path;
    geometry_msgs::Twist cmd_vel;

    double ts = jsonConfig["Ts"];
    int i = 0;
    ros::Rate r(1.0f/ts);
    double lin_vel = 0;
    double ang_vel = 0;
    double vs = 0;

    State x0 = {robot_state.pose.x, robot_state.pose.y, robot_state.pose.theta, 0, 0, 0};
    
    while(ros::ok())
    {
        raw_path.poses.clear();


        std::cout << "x0.X, Y, phi: " << x0.X << ", " << x0.Y << ", " << x0.phi << std::endl;
        std::cout << "x0.s, vx, vs: " << x0.s << ", " << x0.vx << ", " << x0.vs << std::endl;

        x0.X = robot_state.pose.x;
        x0.Y = robot_state.pose.y;
        x0.phi = robot_state.pose.theta;
        x0.vx = odometry_.twist.twist.linear.x;
        MPCReturn mpc_sol = mpc.runMPC(x0);

        lin_vel += mpc_sol.u0.dVx;
        vs += mpc_sol.u0.dVs;
        std::cout << "linear.x, angular.z: " << lin_vel << ", " << mpc_sol.u0.dPhi << std::endl;
        cmd_vel.linear.x = lin_vel;
        cmd_vel.angular.z = mpc_sol.u0.dPhi;
        cmd_vel_pub_.publish(cmd_vel);

        x0 = integrator.simTimeStep(x0, mpc_sol.u0, ts);
        
        raw_path.header.frame_id = "map";
        raw_path.header.stamp = ros::Time::now();
        for(int j=0;j<mpc_sol.mpc_horizon.size();j++)
        {
            geometry_msgs::PoseStamped track_pose;
            track_pose.pose.position.x = mpc_sol.mpc_horizon[0].xk.X;
            track_pose.pose.position.y = mpc_sol.mpc_horizon[0].xk.Y;
            track_pose.pose.orientation.z = 1;
            raw_path.poses.insert(raw_path.poses.end(), track_pose);
        }
        path_pub_.publish(raw_path); // For visualization

        i++;
        if(1000 < i)
            break;

        log.push_back(mpc_sol);
        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        
        ros::spinOnce();
        r.sleep();
    }
    geometry_msgs::Twist stop_vel;
    cmd_vel_pub_.publish(stop_vel);
    
    plotter.plotRun(log, track_xy);
    //plotter.plotSim(log, track_xy);

    double mean_time = 0.0;
    double max_time = 0.0;
    for(MPCReturn log_i : log)
    {
        mean_time += log_i.time_total;
        if(log_i.time_total > max_time)
            max_time = log_i.time_total;
    }
    std::cout << "mean nmpc time " << mean_time/double(jsonConfig["n_sim"]) << std::endl;
    std::cout << "max nmpc time " << max_time << std::endl;

    return 0;
}


