#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "game_theory_sim/game_theory_planner.h"
#include <vector>
#include <cmath>
#include <algorithm>

int main(int argc, char** argv) {
    ros::init(argc, argv, "game_theory_planner_node");
    ros::NodeHandle nh;

    // --- 1. CONFIGURAZIONE DEI CANALI MOTORI ---
    std::vector<ros::Publisher> publishers;
    publishers.push_back(nh.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 10));
    publishers.push_back(nh.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 10));

    double Dt = 0.10; 
    int Mn = 6;
    int MAX_STEP = 500; // Leggermente aumentato per dargli il tempo di girarsi con calma

    Environment env;

    // --- 2. CONFIGURAZIONE DEGLI AGENTI ---
    std::vector<Agent> agents;
    agents.push_back(Agent(1, {0.0, 0.0}, {8.0, 8.0})); 
    agents.push_back(Agent(2, {8.0, 8.0}, {0.0, 0.0})); 

    std::vector<double> yaws(agents.size(), 0.0);

    ros::Rate rate(1.0 / Dt); 
    ROS_INFO("=== PLANNER C++ MODULARE (Agenti attivi: %lu) ===", agents.size());

    for (int step = 1; step <= MAX_STEP && ros::ok(); ++step) {
        bool all_reached = true;
        for (auto& ag : agents) {
            if (!ag.hasReachedGoal()) all_reached = false;
        }
        if (all_reached) {
            ROS_INFO("Tutti gli agenti hanno raggiunto il goal!");
            break;
        }

        ROS_INFO("--- Step %d ---", step);

        for (auto& ag : agents) {
            if (!ag.hasReachedGoal()) ag.plan(env, Mn);
        }

        if (agents.size() == 2) {
            std::vector<std::vector<double>> M1, M2;
            computePayoffMatrix(agents[0], agents[1], M1, M2);
            std::pair<int, int> nash_idx = findNashEquilibrium(M1, M2);

            agents[0].CurrentPath = agents[0].StrategySet[nash_idx.first];
            agents[1].CurrentPath = agents[1].StrategySet[nash_idx.second];
            agents[0].PrevNashPath = agents[0].CurrentPath;
            agents[1].PrevNashPath = agents[1].CurrentPath;
        }

        double step_dist = 0.22 * Dt; 

        // --- 3. MOVIMENTO REALE CON LIMITI FISICI ---
        for (size_t i = 0; i < agents.size(); ++i) {
            if (agents[i].hasReachedGoal()) {
                if (i < publishers.size()) {
                    geometry_msgs::Twist stop_cmd;
                    publishers[i].publish(stop_cmd);
                }
                continue;
            }

            double dx = agents[i].Goal.x - agents[i].Position.x;
            double dy = agents[i].Goal.y - agents[i].Position.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (i < publishers.size() && dist > 0.1) {
                double desired_theta = atan2(dy, dx);
                
                double err_theta = desired_theta - yaws[i];
                while (err_theta > M_PI) err_theta -= 2.0 * M_PI;
                while (err_theta < -M_PI) err_theta += 2.0 * M_PI;

                geometry_msgs::Twist cmd;
                
                // Calcolo rotazione
                cmd.angular.z = 1.0 * err_theta; 
                
                // CLAMP: Non superare mai la velocità fisica di rotazione di Gazebo (0.5 rad/s è sicuro)
                if (cmd.angular.z > 0.5) cmd.angular.z = 0.5;
                if (cmd.angular.z < -0.5) cmd.angular.z = -0.5;
                
                // Margine di tolleranza più stretto: avanza solo se sta guardando quasi dritto all'obiettivo
                if (std::abs(err_theta) > 0.1) {
                    cmd.linear.x = 0.0; // Fermo e ruota
                } else {
                    cmd.linear.x = 0.22; // Allineato, avanza
                }
                publishers[i].publish(cmd);

                // Adesso l'aggiornamento matematico riflette esattamente quello che riesce a fare Gazebo
                yaws[i] += cmd.angular.z * Dt;
                
                if (cmd.linear.x > 0.0) {
                    if (dist > step_dist) {
                        agents[i].Position.x += (dx / dist) * step_dist;
                        agents[i].Position.y += (dy / dist) * step_dist;
                    } else {
                        agents[i].Position = agents[i].Goal;
                    }
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    for (auto& pub : publishers) {
        geometry_msgs::Twist stop_cmd;
        pub.publish(stop_cmd);
    }
    ROS_INFO("Simulazione terminata. Robot fermati.");

    return 0;
}
