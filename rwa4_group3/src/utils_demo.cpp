/**
 * @file utils_demo.cpp
 * @author Shivam Sehgal(ssehga7@umd.edu), Darshit Desai(Darshit@umd.edu),
 *   Patrik Pordi(ppordi@umd.edu), Rohith(rohithvs@umd.edu)
 * @brief Utility Interface class handles the information contained in order
 * message and kitting task details, This file is used to define the functions
 * @version 0.1
 * @date 2024-03-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "rwa4_group3/utils_interface.hpp"

//=========================================
int Kitting::getAgvId() { return agv_id_; }
//=========================================
int Kitting::getTrayId() { return tray_id_; }
//=========================================
std::vector<ariac_msgs::msg::KittingPart> Kitting::getParts() { return parts_; }
//=========================================
unsigned int Kitting::getDestination() { return destination_; }
//=========================================

//=========================================
std::string Order::getOrderId() { return order_id_; }
//=========================================
int Order::getOrderType() { return order_type_; }
//=========================================
bool Order::getOrderPriority() { return order_priority_; }
//=========================================
Kitting Order::getOrderTask() { return order_task_; }
//=========================================
double Order::getTimePassed() { return time_passed_; }
//=========================================
bool Order::isIncomplete() { return incomplete_; }
//=========================================
void Order::completeOrder() { incomplete_ = false; }
//=========================================
void Order::setTimePassed(double time_passed) { time_passed_ = time_passed; }

//=========================================
unsigned int Parts::getPartType() { return part_type_; }
//=========================================
unsigned int Parts::getPartColor() { return part_color_; }
//=========================================
geometry_msgs::msg::Pose Parts::getPartPose() { return part_pose_; }

//=========================================
unsigned int Trays::getTrayId() { return tray_id_; }
//=========================================
geometry_msgs::msg::Pose Trays::getTrayPose() { return tray_pose_; }