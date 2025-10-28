#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h> // Contains POSIX terminal control definitions
#include <errno.h> // Error integer and strerror() function
#include <math.h>
#include <time.h>
#include <Eigen/Geometry>
#include "mavlink/ardupilotmega/mavlink.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

// ROS coordinate system, x axis = vehicle front

#define MY_COMP_ID 191

#define MAX_WP_DIST_M 8

#define SEARCH_STRUCT_CROSS 1
#define PASS_STRUCT_CROSS 2
#define MOVE_UP 1
#define MOVE_RIGHT 2
#define MOVE_DOWN 3
#define MOVE_LEFT 4
#define LAND 5
#define HOVER 6

#define CLOSE_DIST_M 0.6f
#define FAR_DIST_M 0.9f

using namespace std::chrono_literals;

template <typename T, std::size_t N>
class MyCircularBuffer {
public:
    void push(const T& value) {
        data_[(head_ + size_) % N] = value;
        if (size_ < N) {
            ++size_;
        } else {
            head_ = (head_ + 1) % N; // overwrite oldest
        }
    }

    std::optional<T> pop() {
        if (empty()) return std::nullopt;
        T value = data_[head_];
        head_ = (head_ + 1) % N;
        --size_;
        return value;
    }

    bool empty() const { return size_ == 0; }
    bool full() const { return size_ == N; }
    std::size_t size() const { return size_; }
    constexpr std::size_t capacity() const { return N; }
    std::array<T, N> data_copy() const { return data_; }

private:
    std::array<T, N> data_{};
    std::size_t head_ = 0;
    std::size_t size_ = 0;
};

class MavRosNode : public rclcpp::Node {
    public:
        MavRosNode(int uart_fd) : Node("mavlink_ros"), uart_fd_(uart_fd) {
            roll_pub = this->create_publisher<std_msgs::msg::Float32>("roll", rclcpp::QoS(1).best_effort().durability_volatile());
            //voltage_pub = this->create_publisher<std_msgs::msg::Float32>("voltage", rclcpp::QoS(1).best_effort().durability_volatile());
            sonar_pub = this->create_publisher<sensor_msgs::msg::Range>("sonar", rclcpp::QoS(1).best_effort().durability_volatile());
            vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("tgt_vel", rclcpp::QoS(1).best_effort().durability_volatile());
            //is_armable_pub = this->create_publisher<std_msgs::msg::Int32>("is_armable", 1);
            //odom_cor_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry_corrected", rclcpp::QoS(1).best_effort().durability_volatile());
            wp_pub = this->create_publisher<visualization_msgs::msg::Marker>("waypoints", rclcpp::QoS(1).best_effort().durability_volatile());
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&MavRosNode::odom_callback, this, std::placeholders::_1));
            intersec_sub = this->create_subscription<geometry_msgs::msg::Point>("templateCOG", 1, std::bind(&MavRosNode::intersect_callback, this, std::placeholders::_1));
            vert_hori_line_sub = this->create_subscription<geometry_msgs::msg::PolygonStamped>("vert_hori_line", 1, std::bind(&MavRosNode::vert_hori_line_callback, this, std::placeholders::_1));
            cmd_sub = this->create_subscription<std_msgs::msg::String>("cmd", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&MavRosNode::cmd_callback, this, std::placeholders::_1));
            uart_timer = this->create_wall_timer(2ms, std::bind(&MavRosNode::timer_callback, this));
        }

        void read_mission(const char* csv_path) {
            FILE *file = fopen(csv_path, "r");
            if (file) {
                int act, type, dist_cm;
                while (fscanf(file, "%d ,%d ,%d", &act, &type, &dist_cm) == 3) {
                    missions.push_back({act, type, dist_cm});
                }
                printf("mission loaded, total %ld\n", missions.size());
                fclose(file);
            } else printf("can not open mission file: %s\n", csv_path);
        }

    private:
        void cmd_callback(std_msgs::msg::String::SharedPtr cmd_msg) {
            int len;
            unsigned char buf[256];
            mavlink_message_t msg;
            std::cout << "rcv " << cmd_msg->data << "\n";
            if (cmd_msg->data == "{\"cmd\": \"arm\"}") {
                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
            } else if (cmd_msg->data == "{\"cmd\": \"disarm\"}") {
                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
            }
        }
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
            mavlink_message_t msg;
            float covar[21] = {0};
            float q[4];
            int len;
            unsigned char buf[256];
            auto pos = odom_msg->pose.pose.position;
            auto ori = odom_msg->pose.pose.orientation;
            auto vel = odom_msg->twist.twist.linear;
            cur_pos = pos;

#if 0
            float heading = atan2f(2 * (ori.w * ori.z + ori.x * ori.y), 1 - 2 * (ori.y * ori.y + ori.z * ori.z));
            //std::cout << heading * 180.0 / M_PI << " deg\n";
            if (hori_line_angle != 0) {
                //RCLCPP_WARN(this->get_logger(), "vio heading %f deg, shelf hori angle %f deg", heading * 180.0 / M_PI, hori_line_angle * 180.0 / M_PI);
                Eigen::Quaternionf q_z(Eigen::AngleAxisf(hori_line_angle - heading, Eigen::Vector3f::UnitZ()));
                Eigen::Quaternionf q_heading(1, ori.x, ori.y, ori.z);
                Eigen::Quaternionf q_cor = q_z * q_heading;
                q_cor.normalize();
                //Eigen::Vector3f t_vio(pos.x, pos.y, pos.z);
                //Eigen::Vector3f t_cor = q_cor * t_vio;
                nav_msgs::msg::Odometry odom_cor;
                odom_cor.header = odom_msg->header;
                odom_cor.child_frame_id = "map";
                //odom_cor.pose.pose.position.x = t_cor.x();
                //odom_cor.pose.pose.position.y = t_cor.y();
                //odom_cor.pose.pose.position.z = t_cor.z();
                odom_cor.pose.pose.position = pos;
                odom_cor.pose.pose.orientation.w = q_cor.w();
                odom_cor.pose.pose.orientation.x = q_cor.x();
                odom_cor.pose.pose.orientation.y = q_cor.y();
                odom_cor.pose.pose.orientation.z = q_cor.z();
                odom_cor.twist.twist.linear = vel;
                odom_cor_pub->publish(odom_cor);
                ori = odom_cor.pose.pose.orientation;
            }
#endif

            if (mav_sysid != 0 && time_offset_ns != 0) {
                q[0] = ori.w;
                q[1] = ori.x;
                q[2] = -ori.y;
                q[3] = -ori.z;
                int64_t odom_fc_us = ((odom_msg->header.stamp.sec * 1000000000LL + odom_msg->header.stamp.nanosec) - time_offset_ns) / 1000;
                mavlink_msg_att_pos_mocap_pack(mav_sysid, MY_COMP_ID, &msg, odom_fc_us, q, pos.x, -pos.y, -pos.z, covar);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
                mavlink_msg_vision_speed_estimate_pack(mav_sysid, MY_COMP_ID, &msg, odom_fc_us, vel.x, -vel.y, -vel.z, covar, 0);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
            }
        }

        void intersect_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
            //printf("intersection %f %f\n", msg->x, msg->y);
            clock_gettime(CLOCK_MONOTONIC, &tp_intersect);
        }

        void vert_hori_line_callback(const geometry_msgs::msg::PolygonStamped::SharedPtr poly_msg) {
            unsigned char buf[256];
            mavlink_message_t msg;
            struct timespec tp;
            clock_gettime(CLOCK_MONOTONIC, &tp);
            int len;
            auto vert_p = poly_msg->polygon.points[0];
            auto vert_v = poly_msg->polygon.points[1];
            auto hori_p = poly_msg->polygon.points[2];
            auto hori_v = poly_msg->polygon.points[3];

            //float hori_angle = atan2f(hori_v.y, hori_v.x);
            //std::cout << "hori " << hori_angle * 180.0 / M_PI << " degrees\n";
            if (hori_p.x != 0) {
                float vy;
                if (hori_v.x < 0) {
                    vy = -hori_v.y;
                } else {
                    vy = hori_v.y;
                }
                float angle_y_hori = acosf(vy);
                //std::cout << "raw hori " << angle_y_hori * 180 / M_PI << " deg\n";
                hori_line_angles.push(angle_y_hori);
                if (hori_line_angles.full()) {
                    auto angles = hori_line_angles.data_copy();
                    std::nth_element(angles.begin(), angles.begin() + 2, angles.end());
                    hori_line_angle = angles[2];
                    if (hori_line_angle > M_PI / 2) hori_line_angle = hori_line_angle - M_PI;
                    //std::cout << "filtered hori " << hori_line_angle * 180.0 / M_PI << " deg\n";
                }
            }
            //if (hori_p.x != 0) std::cout << "heading " << acosf(hori_v.y) * 180 / M_PI << " degrees\n";

#if 0
            // just for test
            if (hori_p.x != 0) {
                // find the intersection point of the hori struct line and the plane y = 0
                float t = -hori_p.y / hori_v.y;
                float z = hori_p.z + hori_v.z * t;
                float x = hori_p.x + hori_v.x * t;
                hori_line_z.push(z);
                if (hori_line_z.full()) {
                    auto zz = hori_line_z.data_copy();
                    std::nth_element(zz.begin(), zz.begin() + 2, zz.end());
                    float mid_z = zz[2];
                    if (mid_z > 0.1f) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too low, move up");
                    } else if (mid_z < -0.2f) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too high, move down");
                    }
                }
            }
            if (vert_p.x != 0) {
                float t = -vert_p.z / vert_v.z;
                float x = vert_p.x + vert_v.x * t;
                float y = vert_p.y + vert_v.y * t;
                vert_line_y.push(y);
                if (vert_line_y.full()) {
                    auto yy = vert_line_y.data_copy();
                    std::nth_element(yy.begin(), yy.begin() + 2, yy.end());
                    float mid_y = yy[2];
                    if (mid_y > 0.2f) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too right, move left");
                    } else if (mid_y < -0.2f) {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too left, move right");
                    }
                }
            }
#endif

            if (yaw_adj_cd > 0) yaw_adj_cd--;
            if (mission_idx >= 0 && mission_idx < (int)missions.size()) {
                if (navi_status == SEARCH_STRUCT_CROSS) {
                    bool intersect_detected = ((tp.tv_sec - tp_intersect.tv_sec) * 1'000'000'000 + tp.tv_nsec - tp_intersect.tv_nsec) < 500'000'000;
                    if (vert_p.x != 0 && hori_p.x != 0) intersect_confirm_cnt++;
                    if (intersect_confirm_cnt > 2 || intersect_detected) {
                        intersect_confirm_cnt = 0;
                        RCLCPP_INFO(this->get_logger(), "arrival at waypoint %d", mission_idx);
                        navi_status = PASS_STRUCT_CROSS;
                        move_status = missions[mission_idx][0];
                        last_wp_pos = cur_pos;

                        visualization_msgs::msg::Marker marker;
                        marker.header.frame_id = "body";
                        marker.header.stamp.sec = tp.tv_sec;
                        marker.header.stamp.nanosec = tp.tv_nsec;
                        marker.ns = "wp";
                        marker.id = mission_idx;
                        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                        marker.action = visualization_msgs::msg::Marker::ADD;
                        marker.pose.position.x = vert_p.x;
                        marker.pose.position.y = vert_p.y;
                        marker.pose.position.z = vert_p.z;
                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;
                        marker.scale.z = 0.3;
                        marker.color.r = 1.0f;
                        marker.color.g = 1.0f;
                        marker.color.b = 1.0f;
                        marker.color.a = 1.0f;
                        marker.text = std::to_string(mission_idx+1);
                        wp_pub->publish(marker);
                        if (vert_p.x != 0 && hori_p.x != 0) {
                            marker.ns = "intersect";
                            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
                            marker.scale.x = 0.02;
                            marker.pose.position.x = 0;
                            marker.pose.position.y = 0;
                            marker.pose.position.z = 0;
                            geometry_msgs::msg::Point h1, h2, v1, v2;
                            h1.x = hori_p.x+hori_v.x;
                            h1.y = hori_p.y+hori_v.y;
                            h1.z = hori_p.z+hori_v.z;
                            h2.x = hori_p.x-hori_v.x;
                            h2.y = hori_p.y-hori_v.y;
                            h2.z = hori_p.z-hori_v.z;
                            v1.x = vert_p.x+vert_v.x;
                            v1.y = vert_p.y+vert_v.y;
                            v1.z = vert_p.z+vert_v.z;
                            v2.x = vert_p.x-vert_v.x;
                            v2.y = vert_p.y-vert_v.y;
                            v2.z = vert_p.z-vert_v.z;
                            marker.points = {h1, h2, v1, v2};
                            wp_pub->publish(marker);
                        }
                    }
                } else if (navi_status == PASS_STRUCT_CROSS) {
                    float dy = cur_pos.y - last_wp_pos.y;
                    float dz = cur_pos.z - last_wp_pos.z;
                    if (dy * dy + dz * dz > 1) {
                        RCLCPP_INFO(this->get_logger(), "intersection passed");
                        navi_status = SEARCH_STRUCT_CROSS;
                        mission_idx++;
                    }
                }
                if (move_status == MOVE_RIGHT || move_status == MOVE_LEFT) {
                    float vel_r = 0.2f;
                    float vel_d = 0;
                    float vel_f = 0;
                    uint16_t type_mask = 0xdc7;
                    if (move_status == MOVE_LEFT) vel_r = -0.2f;
                    if (hori_p.x != 0) {
                        // find the intersection point of the hori struct line and the plane y = 0
                        float t = -hori_p.y / hori_v.y;
                        float z = hori_p.z + hori_v.z * t;
                        float x = hori_p.x + hori_v.x * t;
                        if (last_struct_dist == 0 || fabsf(x - last_struct_dist) < 0.6f) { // our dist to struct will not change that large
                            last_struct_dist = x;
                            hori_line_x.push(x);
                            if (hori_line_x.full()) {
                                auto xx = hori_line_x.data_copy();
                                std::nth_element(xx.begin(), xx.begin() + 2, xx.end());
                                float mid_x = xx[2];
                                if (mid_x > FAR_DIST_M) {
                                    vel_f = 0.12f;
                                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too far, move closer");
                                } else if (mid_x < CLOSE_DIST_M) {
                                    vel_f = -0.12f;
                                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too close, move away");
                                }
                            }
                            hori_line_z.push(z);
                            if (hori_line_z.full()) {
                                auto zz = hori_line_z.data_copy();
                                std::nth_element(zz.begin(), zz.begin() + 2, zz.end());
                                float mid_z = zz[2];
                                if (mid_z > 0.1f) {
                                    vel_d = -0.15f;
                                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too low, move up");
                                } else if (mid_z < -0.2f) {
                                    vel_d = 0.1f;
                                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too high, move down");
                                }
                            }
                            /*float vx, vy;
                            if (hori_v.y < 0) {
                                vx = -hori_v.x;
                                vy = -hori_v.y;
                            } else {
                                vx = hori_v.x;
                                vy = hori_v.y;
                            }
                            float angle_y_hori = acosf(vy);
                            if (angle_y_hori > 0.15f) align_confirm_cnt++; else align_confirm_cnt = 0;
                            if (align_confirm_cnt > 1 && yaw_adj_cd == 0) {
                                yaw_adj_cd = 10;
                                align_confirm_cnt = 0;
                                //printf("angle_y_hori %f %f\n", angle_y_hori, vx);
                                if (vx > 0) tgt_yaw = cur_yaw + angle_y_hori; else tgt_yaw = cur_yaw - angle_y_hori;
                                RCLCPP_INFO_STREAM(this->get_logger(), "adjust heading " << (vx > 0 ? std::string("cw ") : std::string("ccw ")) <<  angle_y_hori * 180 / M_PI << " from " << cur_yaw * 180 / M_PI << " to " << tgt_yaw * 180 / M_PI);
                            }*/
                            if (fabsf(hori_line_angle) > 0.15f && yaw_adj_cd == 0) {
                                yaw_adj_cd = 30;
                                tgt_yaw = cur_yaw + hori_line_angle;
                                RCLCPP_INFO(this->get_logger(), "adjust heading from %f to %f", cur_yaw, tgt_yaw);
                            }
                        }
                    }
                    if (yaw_adj_cd > 0) type_mask = 0x9c7;
                    if (fc_prx_too_close) {
                        vel_f = -0.15f;
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "sonar: too close, move away");
                    }
                    mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tp.tv_sec*1000+(uint32_t)(tp.tv_nsec*0.000001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, type_mask, 0, 0, 0, vel_f, vel_r, vel_d, 0, 0, 0, tgt_yaw, 0);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    write(uart_fd_, buf, len);

                    auto twist_stamped = geometry_msgs::msg::TwistStamped();
                    twist_stamped.header.stamp = this->get_clock()->now();
                    twist_stamped.header.frame_id = "body";
                    twist_stamped.twist.linear.x = vel_f;
                    twist_stamped.twist.linear.y = -vel_r;
                    twist_stamped.twist.linear.z = -vel_d;
                    vel_pub->publish(twist_stamped);
                } else if (move_status == MOVE_UP || move_status == MOVE_DOWN) {
                    float vel_f = 0;
                    float vel_r = 0;
                    float vel_d = 0.2f;
                    if (move_status == MOVE_UP) vel_d = -0.2f;
                    if (vert_p.x != 0) {
                        float t = -vert_p.z / vert_v.z;
                        float x = vert_p.x + vert_v.x * t;
                        float y = vert_p.y + vert_v.y * t;
                        if (last_struct_dist == 0 || fabsf(x - last_struct_dist) < 0.6f) { // our dist to struct will not change that large
                            last_struct_dist = x;
                            vert_line_x.push(x);
                            if (vert_line_x.full()) {
                                auto xx = vert_line_x.data_copy();
                                std::nth_element(xx.begin(), xx.begin() + 2, xx.end());
                                float mid_x = xx[2];
                                if (mid_x > FAR_DIST_M) {
                                    vel_f = 0.12f;
                                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too far, move closer");
                                } else if (mid_x < CLOSE_DIST_M) {
                                    vel_f = -0.12f;
                                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too close, move away");
                                }
                            }
                            vert_line_y.push(y);
                            if (vert_line_y.full()) {
                                auto yy = vert_line_y.data_copy();
                                std::nth_element(yy.begin(), yy.begin() + 2, yy.end());
                                float mid_y = yy[2];
                                if (mid_y > 0.2f) {
                                    vel_r = -0.12f;
                                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too right, move left");
                                } else if (mid_y < -0.2f) {
                                    vel_r = 0.12f;
                                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "too left, move right");
                                }
                            }
                        }
                    }
                    if (fc_prx_too_close) {
                        vel_f = -0.15f;
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "sonar: too close, move away");
                    }
                    mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tp.tv_sec*1000+(uint32_t)(tp.tv_nsec*0.000001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, vel_f, vel_r, vel_d, 0, 0, 0, 0, 0);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    write(uart_fd_, buf, len);

                    auto twist_stamped = geometry_msgs::msg::TwistStamped();
                    twist_stamped.header.stamp = this->get_clock()->now();
                    twist_stamped.header.frame_id = "body";
                    twist_stamped.twist.linear.x = vel_f;
                    twist_stamped.twist.linear.y = -vel_r;
                    twist_stamped.twist.linear.z = -vel_d;
                    vel_pub->publish(twist_stamped);
                } else if (move_status == LAND) {
                    mavlink_msg_set_mode_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, COPTER_MODE_LAND);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    write(uart_fd_, buf, len);
                } else if (move_status == HOVER) {
                    if (hori_p.x == 0) {
                        mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tp.tv_sec*1000+(uint32_t)(tp.tv_nsec*0.000001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd_, buf, len);
                    } else {
                        // find the intersection point of the hori struct line and the plane y = 0
                        float t = -hori_p.y / hori_v.y;
                        float z = hori_p.z + hori_v.z * t;
                        float x = hori_p.x + hori_v.x * t;
                        float vel_f = 0;
                        float vel_d = 0;
                        if (x < CLOSE_DIST_M) {
                            vel_f = -0.12f;
                            RCLCPP_INFO(this->get_logger(), "too close, move away");
                        } else if (x > FAR_DIST_M) {
                            vel_f = 0.12f;
                            RCLCPP_INFO(this->get_logger(), "too far, move closer");
                        }
                        if (z > 0.2f) {
                            vel_d = -0.12f;
                            RCLCPP_INFO(this->get_logger(), "too low, move up");
                        } else if (z < -0.2f) {
                            vel_d = 0.12f;
                            RCLCPP_INFO(this->get_logger(), "too high, move down");
                        }
                        mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tp.tv_sec*1000+(uint32_t)(tp.tv_nsec*0.000001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, vel_f, 0, vel_d, 0, 0, 0, 0, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd_, buf, len);
                    }
                }
            }
        }

        void timer_callback() {
            static int timesync_counter = 3;
            unsigned char buf[1024];
            mavlink_status_t status = {};
            mavlink_message_t msg;
            struct timeval tv;
            struct timespec tp;
            int len;
            int avail = read(uart_fd_, buf, sizeof(buf));
            for (int i = 0; i < avail; i++) {
                if (mavlink_parse_char(0, buf[i], &msg, &status)) {
                    if (parse_error != status.parse_error) {
                        parse_error = status.parse_error;
                        printf("mavlink parse_error %d\n", parse_error);
                    }
                    if (packet_rx_drop_count != status.packet_rx_drop_count) {
                        packet_rx_drop_count = status.packet_rx_drop_count;
                        printf("mavlink drop %d\n", packet_rx_drop_count);
                    }
                    if (msg.sysid == 255) continue;
                    //printf("recv msg ID %d, seq %d\n", msg.msgid, msg.seq);
                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        mavlink_heartbeat_t hb;
                        mavlink_msg_heartbeat_decode(&msg, &hb);
                        if (msg.sysid != mav_sysid) {
                            mav_sysid = msg.sysid;
                            printf("found MAV %d\n", msg.sysid);

                            gettimeofday(&tv, NULL);
                            mavlink_msg_system_time_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);

                            mavlink_msg_set_gps_global_origin_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 247749434, 1210443077, 100000, tv.tv_sec*1000000+tv.tv_usec);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }
                        if (hb.custom_mode == COPTER_MODE_GUIDED) {
                            if (mission_idx == -1) {
                                RCLCPP_INFO(this->get_logger(), "mission start");
                                mission_idx = 0;
                                navi_status = SEARCH_STRUCT_CROSS;
                                move_status = HOVER;
                            }
                        } else {
                            mission_idx = -1;
                        }
                        if (!att_rcved) {
                            mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE, 100'000, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }
                        if (!dist_sensor_rcved) {
                            mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_DISTANCE_SENSOR, 100'000, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }
                        // this is no voltage sensor on current warehouse drone
                        /*if (!batt_rcved) {
                            mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_BATTERY_STATUS, 1000'000, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }*/
                        /*if (hb.system_status == MAV_STATE_STANDBY) {
                            // if is armable = 1
                            auto m = std_msgs::msg::Int32();
                            m.data = 1;
                            is_armable_pub->publish(m);
                        } else {
                            auto m = std_msgs::msg::Int32();
                            m.data = 0;
                            is_armable_pub->publish(m);
                        }*/
                        if (timesync_counter > 3) {
                            timesync_counter = 0;
                            clock_gettime(CLOCK_MONOTONIC, &tp);
                            mavlink_msg_timesync_pack(mav_sysid, MY_COMP_ID, &msg, 0, (int64_t)tp.tv_sec * 1000000000 + tp.tv_nsec, mav_sysid, 1);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        } else timesync_counter++;
                    } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                        mavlink_statustext_t txt;
                        mavlink_msg_statustext_decode(&msg, &txt);
                        printf("fc: %s\n", txt.text);
                    } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                        att_rcved = true;
                        mavlink_attitude_t att;
                        mavlink_msg_attitude_decode(&msg, &att);
                        cur_yaw = att.yaw;
                        auto m = std_msgs::msg::Float32();
                        m.data = att.roll;
                        roll_pub->publish(m);
                    } else if (msg.msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR) {
                        dist_sensor_rcved = true;
                        mavlink_distance_sensor_t dist_sensor;
                        mavlink_msg_distance_sensor_decode(&msg, &dist_sensor);
                        //printf("prx dist %d cm\n", dist_sensor.current_distance);
                        if (dist_sensor.current_distance < (CLOSE_DIST_M * 100)) fc_prx_too_close = true; else fc_prx_too_close = false;
                        auto rng = sensor_msgs::msg::Range();
                        rng.header.frame_id = "body";
                        rng.header.stamp = this->get_clock()->now();
                        rng.radiation_type = 0;
                        rng.field_of_view = 1.0;
                        rng.min_range = 0.2;
                        rng.max_range = 7.5;
                        rng.range = dist_sensor.current_distance * 0.01;
                        sonar_pub->publish(rng);
                    /*} else if (msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
                        batt_rcved = true;
                        mavlink_battery_status_t batt;
                        mavlink_msg_battery_status_decode(&msg, &batt);
                        float voltage = batt.voltages[0] / 1000.0;
                        printf("battery %fv\n", voltage);
                        auto m = std_msgs::msg::Float32();
                        m.data = voltage;
                        voltage_pub->publish(m);*/
                    } else if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
                        mavlink_timesync_t sync;
                        mavlink_msg_timesync_decode(&msg, &sync);
                        if (sync.tc1 > 0) {
                            time_offset_ns = sync.ts1 - sync.tc1;
                            printf("time offset: %ld ns\n", time_offset_ns);
                        }
                    }
                }
            }
        }

        int uart_fd_;
        int parse_error = 0;
        int packet_rx_drop_count = 0;
        uint8_t mav_sysid = 0;
        struct timespec tp_intersect = {0, 0};
        bool att_rcved = false;
        float cur_yaw = 0;
        std::vector<std::array<int, 3>> missions;
        int mission_idx = -1;
        geometry_msgs::msg::Point cur_pos;
        geometry_msgs::msg::Point last_wp_pos;
        int navi_status = SEARCH_STRUCT_CROSS;
        int move_status = HOVER;
        int intersect_confirm_cnt = 0;
        int yaw_adj_cd = 0;
        float tgt_yaw = 0;
        bool dist_sensor_rcved = false;
        bool fc_prx_too_close = false;
        float last_struct_dist = 0;
        struct timeval tv_vert_line = {0, 0};
        float vert_line_p1u = 0;
        int64_t time_offset_ns = 0;
        float hori_line_angle = 0;
        bool batt_rcved = false;
        MyCircularBuffer<float, 5> hori_line_angles;
        MyCircularBuffer<float, 5> hori_line_x;
        MyCircularBuffer<float, 5> hori_line_z;
        MyCircularBuffer<float, 5> vert_line_x;
        MyCircularBuffer<float, 5> vert_line_y;
        Eigen::Quaternionf heading_cor = Eigen::Quaternionf::Identity();
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub;
        //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub;
        //rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr is_armable_pub;
        //rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_cor_pub;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr intersec_sub;
        rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr vert_hori_line_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wp_pub;
        rclcpp::TimerBase::SharedPtr uart_timer;
};

int main(int argc, char *argv[]) {
    int uart_fd;
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;

    if (argc > 1)
        uart_fd = open(argv[1], O_RDWR| O_NOCTTY | O_NONBLOCK);
    else
        uart_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (uart_fd < 0) {
        printf("can not open serial port\n");
        return 1;
    }

    if(tcgetattr(uart_fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return 1;
    }
    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B1500000);
    cfsetospeed(&tty, B1500000);
    // Save tty settings, also checking for error
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    printf("hello\n");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MavRosNode>(uart_fd);
    if (argc > 2)
        node->read_mission(argv[2]);
    else
        node->read_mission("missions.csv");
    rclcpp::spin(node);
    rclcpp::shutdown();

    close(uart_fd);

    printf("bye\n");

    return 0;
}
