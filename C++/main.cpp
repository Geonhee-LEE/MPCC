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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

using json = nlohmann::json;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_predictive_contouring_controller");
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
    TrackPos track_xy = track.getTrack();

    std::list<MPCReturn> log;
    MPC mpc(jsonConfig["n_sqp"],jsonConfig["n_reset"],jsonConfig["sqp_mixing"],jsonConfig["Ts"],json_paths);
    mpc.setTrack(track_xy.X,track_xy.Y);
    const double phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),track_xy.X(1) - track_xy.X(0));
    State x0 = {track_xy.X(0), track_xy.Y(0), phi_0, 0, jsonConfig["v0"], 0};
    
    for(int i=0;i<jsonConfig["n_sim"];i++)
    {
        MPCReturn mpc_sol = mpc.runMPC(x0);
        x0 = integrator.simTimeStep(x0, mpc_sol.u0, jsonConfig["Ts"]);
        log.push_back(mpc_sol);
    }
    //plotter.plotRun(log, track_xy);
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
 
    // ROS
    ros::NodeHandle priv_n("~");
    ros::NodeHandle nh;
    ros::Publisher route_planner_path_pub_ = priv_n.advertise<nav_msgs::Path>("path", 1);
    ros::Publisher cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    nav_msgs::Path raw_path;
    geometry_msgs::Twist cmd_vel;
    std::vector<geometry_msgs::Twist> cmd_vel_array;

    raw_path.header.frame_id = "map";
    raw_path.header.stamp = ros::Time::now();
    for(MPCReturn log_i : log)
    {
        geometry_msgs::PoseStamped track_pose;
        track_pose.pose.position.x = log_i.mpc_horizon[0].xk.X;
        track_pose.pose.position.y = log_i.mpc_horizon[0].xk.Y;
        track_pose.pose.orientation.z = 1;
        raw_path.poses.insert(raw_path.poses.end(), track_pose);
        
        cmd_vel.linear.x = log_i.mpc_horizon[0].xk.vx;
        cmd_vel.angular.z = log_i.mpc_horizon[0].uk.dPhi;
        cmd_vel_array.push_back(cmd_vel);
    }

    double ts = jsonConfig["Ts"];
    int i = 0;
    ros::Rate r(1.0f/ts);
    while(ros::ok())
    {
        route_planner_path_pub_.publish(raw_path); // For visualization
        cmd_vel_pub_.publish(cmd_vel_array[i]);

        i++;
        if(cmd_vel_array.size() < i)
            break;
        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep();
    }

    return 0;
}


