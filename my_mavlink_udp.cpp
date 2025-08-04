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
#include "mavlink/ardupilotmega/mavlink.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "nav_msgs/msg/odometry.hpp"

// ROS coordinate system, x axis = vehicle front

#define MY_COMP_ID 191
#define MY_NUM_PFDS 3
#define SERVER_PATH "/tmp/chobits_server"
#define SERVER_PATH2 "/tmp/chobits_server2"

#define MAX_WP_DIST_M 8

#define SEARCH_STRUCT_CROSS 1
#define PASS_STRUCT_CROSS 2
#define MOVE_UP 1
#define MOVE_RIGHT 2
#define MOVE_DOWN 3
#define MOVE_LEFT 4
#define LAND 5
#define HOVER 6

#define CLOSE_DIST_M 0.5f
#define FAR_DIST_M 0.9f

using namespace std::chrono_literals;

struct __attribute__((packed)) lines_3d {
//where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line.
    float hori_x;
    float hori_y;
    float hori_z;
    float hori_vx;
    float hori_vy;
    float hori_vz;
};

class MavRosNode : public rclcpp::Node {
    public:
        MavRosNode(int uart_fd) : Node("mavlink_ros"), uart_fd_(uart_fd) {
            roll_pub = this->create_publisher<std_msgs::msg::Float32>("roll", rclcpp::QoS(1).best_effort().durability_volatile());
            voltage_pub = this->create_publisher<std_msgs::msg::Float32>("voltage", rclcpp::QoS(1).best_effort().durability_volatile());
            sonar_pub = this->create_publisher<sensor_msgs::msg::Range>("sonar", rclcpp::QoS(1).best_effort().durability_volatile());
            vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("tgt_vel", rclcpp::QoS(1).best_effort().durability_volatile());
            intersect_type_pub = this->create_publisher<std_msgs::msg::Int32>("intersect_type", 1);
            is_armable_sub = this->create_publisher<std_msgs::msg::Int32>("is_armable", 1);
            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(1).best_effort().durability_volatile(), [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); });
            intersec_sub = this->create_subscription<geometry_msgs::msg::Point>("templateCOG", 1, [this](const geometry_msgs::msg::Point::SharedPtr msg) { intersect_callback(msg); });
            hori_line_sub = this->create_subscription<geometry_msgs::msg::Polygon>("hori_line", 1, [this](const geometry_msgs::msg::Polygon::SharedPtr msg) { hori_line_callback(msg); });
            vert_line_sub = this->create_subscription<geometry_msgs::msg::Polygon>("vert_line_polygon", 1, [this](const geometry_msgs::msg::Polygon::SharedPtr msg) { vert_line_callback(msg); });
            uart_timer = this->create_wall_timer(2ms, [this](){ timer_callback(); });
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
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
            mavlink_message_t msg;
            float covar[21] = {0};
            float q[4];
            int len;
            unsigned char buf[256];
            geometry_msgs::msg::Pose *pose = &odom_msg->pose.pose;
            geometry_msgs::msg::Vector3 *v = &odom_msg->twist.twist.linear;
            cur_pos = pose->position;
            if (mav_sysid != 0 && time_offset_ns != 0) {
                q[0] = pose->orientation.w;
                q[1] = pose->orientation.x;
                q[2] = -pose->orientation.y;
                q[3] = -pose->orientation.z;
                int64_t odom_fc_us = ((odom_msg->header.stamp.sec * 1000000000LL + odom_msg->header.stamp.nanosec) - time_offset_ns) / 1000;
                mavlink_msg_att_pos_mocap_pack(mav_sysid, MY_COMP_ID, &msg, odom_fc_us, q, pose->position.x, -pose->position.y, -pose->position.z, covar);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
                mavlink_msg_vision_speed_estimate_pack(mav_sysid, MY_COMP_ID, &msg, odom_fc_us, v->x, -v->y, -v->z, covar, 0);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
            }
        }

        void intersect_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
            //printf("intersection %f %f\n", msg->x, msg->y);
            intersect_cog[0] = msg->x;
            intersect_cog[1] = msg->y;
            gettimeofday(&tv_intersect, NULL);
        }

        void vert_line_callback(const geometry_msgs::msg::Polygon::SharedPtr poly_msg) {
            vert_line_p1u = poly_msg->points[0].x;
            gettimeofday(&tv_vert_line, NULL);
        }

        void hori_line_callback(const geometry_msgs::msg::Polygon::SharedPtr poly_msg) {
            unsigned char buf[256];
            mavlink_message_t msg;
            struct timeval tv;
            int len;
            auto hori_p = poly_msg->points[0];
            auto hori_v = poly_msg->points[1];
            if (yaw_adj_cd > 0) yaw_adj_cd--;
            if (mission_idx >= 0 && mission_idx < (int)missions.size()) {
                if (navi_status == SEARCH_STRUCT_CROSS) {
                    gettimeofday(&tv, NULL);
                    if (((tv.tv_sec - tv_intersect.tv_sec) * 1'000'000 + tv.tv_usec - tv_intersect.tv_usec) < 500'000) {
                        if ((move_status == MOVE_LEFT && intersect_cog[0] < 0.2) || (move_status == MOVE_RIGHT && intersect_cog[0] > 0.8) || (move_status == MOVE_UP && intersect_cog[1] < 0.2)) {
                            slow_down = true;
                            RCLCPP_INFO(this->get_logger(), "intersection detected on edge, slowing down");
                        } else {
                            slow_down = false;
                            RCLCPP_INFO(this->get_logger(), "arrival at waypoint %d", mission_idx);
                            navi_status = PASS_STRUCT_CROSS;
                            move_status = missions[mission_idx][0];
                            last_wp_pos = cur_pos;
                            last_struct_dist = 0;
                            // AP_NOTIFY_TONE_LOUD_WP_COMPLETE
                            // to noisy, cannot hear it
                            /*mavlink_msg_play_tune_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, "MFT200L8G>C3", "");
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);*/
                        }
                    } else {
                        float dx = cur_pos.x - last_wp_pos.x;
                        float dy = cur_pos.y - last_wp_pos.y;
                        float dz = cur_pos.z - last_wp_pos.z;
                        float max_dist = missions[mission_idx][2] * 0.01f;
                        if ((max_dist > 0) && ((dx * dx + dy * dy + dz * dz) > (max_dist * max_dist))) {
                            RCLCPP_WARN(this->get_logger(), "exceed wp dist");
                            move_status = HOVER;
                        }
                    }
                } else if (navi_status == PASS_STRUCT_CROSS) {
                    gettimeofday(&tv, NULL);
                    if (((tv.tv_sec - tv_intersect.tv_sec) * 1'000'000 + tv.tv_usec - tv_intersect.tv_usec) > 2'000'000) {
                        RCLCPP_INFO(this->get_logger(), "intersection passed");
                        navi_status = SEARCH_STRUCT_CROSS;
                        mission_idx++;
                        if (mission_idx < (int)missions.size()) {
                            auto m = std_msgs::msg::Int32();
                            m.data = missions[mission_idx][1];
                            intersect_type_pub->publish(m);
                        }
                    }
                }
                if (move_status == MOVE_RIGHT || move_status == MOVE_LEFT) {
                    float vel_r = 0.2f;
                    float vel_d = 0;
                    float vel_f = 0;
                    uint16_t type_mask = 0xdc7;
                    if (move_status == MOVE_LEFT) vel_r = -0.2f;
                    if (slow_down) vel_r = vel_r * 0.6f;
                    if (hori_p.x == 0) {
                    } else {
                        // find the intersection point of the hori struct line and the plane y = 0
                        float t = -hori_p.y / hori_v.y;
                        float z = hori_p.z + hori_v.z * t;
                        float x = hori_p.x + hori_v.x * t;
                        if (last_struct_dist == 0 || fabsf(x - last_struct_dist) < 0.6f) { // our dist to struct will not change that large
                            last_struct_dist = x;
                            if (z > 0.2f) low_confirm_cnt++; else low_confirm_cnt = 0;
                            if (z < -0.2f) high_confirm_cnt++; else high_confirm_cnt = 0;
                            if (x < CLOSE_DIST_M) close_confirm_cnt++; else close_confirm_cnt = 0;
                            if (x > FAR_DIST_M) far_confirm_cnt++; else far_confirm_cnt = 0;
                            if (low_confirm_cnt > 1) {
                                vel_d = -0.12f;
                                RCLCPP_INFO(this->get_logger(), "too low, move up");
                            } else if (high_confirm_cnt > 1) {
                                vel_d = 0.12f;
                                RCLCPP_INFO(this->get_logger(), "too high, move down");
                            }
                            if (close_confirm_cnt > 1) {
                                adj_cnt++;
                                vel_f = -0.12f;
                                RCLCPP_INFO(this->get_logger(), "too close, move away");
                            } else if (far_confirm_cnt > 1) {
                                adj_cnt++;
                                vel_f = 0.12f;
                                RCLCPP_INFO(this->get_logger(), "too far, move closer");
                            }
                            if (adj_cnt > 5) {
                                adj_cnt = 0;
                                far_confirm_cnt = 0;
                                close_confirm_cnt = 0;
                            }

                            float vx, vy;
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
                            }
                        }
                    }
                    if (yaw_adj_cd > 0) type_mask = 0x9c7;
                    if (fc_prx_too_close) {
                        vel_f = -0.15f;
                        RCLCPP_INFO(this->get_logger(), "sonar: too close, move away");
                    }
                    gettimeofday(&tv, NULL);
                    mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+(uint32_t)(tv.tv_usec*0.001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, type_mask, 0, 0, 0, vel_f, vel_r, vel_d, 0, 0, 0, tgt_yaw, 0);
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
                    if (slow_down) vel_d = vel_d * 0.6f;
                    gettimeofday(&tv, NULL);
                    if (((tv.tv_sec - tv_vert_line.tv_sec) * 1'000'000 + tv.tv_usec - tv_vert_line.tv_usec) < 500'000) {
                        if (vert_line_p1u < 0.3) {
                            vel_r = -0.12f;
                        } else if (vert_line_p1u > 0.7) {
                            vel_r = 0.12f;
                        }
                    }
                    if (fc_prx_too_close) {
                        vel_f = -0.15f;
                        RCLCPP_INFO(this->get_logger(), "sonar: too close, move away");
                    }
                    gettimeofday(&tv, NULL);
                    mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+(uint32_t)(tv.tv_usec*0.001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, vel_f, vel_r, vel_d, 0, 0, 0, 0, 0);
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
                        gettimeofday(&tv, NULL);
                        mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+(uint32_t)(tv.tv_usec*0.001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd_, buf, len);
                    } else {
                        // find the intersection point of the hori struct line and the plane y = 0
                        float t = -hori_p.y / hori_v.y;
                        float x = hori_p.x + hori_v.x * t;
                        float vel_f = 0;
                        if (x < CLOSE_DIST_M) {
                            vel_f = -0.12f;
                        } else if (x > FAR_DIST_M) {
                            vel_f = 0.12f;
                        }
                        gettimeofday(&tv, NULL);
                        mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+(uint32_t)(tv.tv_usec*0.001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, vel_f, 0, 0, 0, 0, 0, 0, 0);
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
                                auto m = std_msgs::msg::Int32();
                                m.data = missions[0][1];
                                intersect_type_pub->publish(m);
                                last_wp_pos = cur_pos;
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
                        if (hb.system_status == MAV_STATE_STANDBY) {
                            // if is armable = 1
                            auto m = std_msgs::msg::Int32();
                            m.data = 1;
                            is_armable_sub->publish(m);
                        } else {
                            auto m = std_msgs::msg::Int32();
                            m.data = 0;
                            is_armable_sub->publish(m);
                        }
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
                    } else if (msg.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
                        std::cout << "check";
                        mavlink_battery_status_t batt;
                        mavlink_msg_battery_status_decode(&msg, &batt);
                        auto m = std_msgs::msg::Float32();
                        m.data = batt.voltages[0] / 1000.0;
                        voltage_pub->publish(m);
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
        struct timeval tv_intersect = {0, 0};
        float intersect_cog[2] = {0, 0};
        bool att_rcved = false;
        float cur_yaw = 0;
        std::vector<std::array<int, 3>> missions;
        int mission_idx = -1;
        geometry_msgs::msg::Point cur_pos;
        geometry_msgs::msg::Point last_wp_pos;
        int navi_status = SEARCH_STRUCT_CROSS;
        int move_status = HOVER;
        int low_confirm_cnt = 0;
        int high_confirm_cnt = 0;
        int close_confirm_cnt = 0;
        int far_confirm_cnt = 0;
        int align_confirm_cnt = 0;
        int yaw_adj_cd = 0;
        float tgt_yaw = 0;
        unsigned int adj_cnt = 0;
        bool dist_sensor_rcved = false;
        bool fc_prx_too_close = false;
        bool slow_down = false;
        float last_struct_dist = 0;
        struct timeval tv_vert_line = {0, 0};
        float vert_line_p1u = 0;
        int64_t time_offset_ns = 0;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr intersect_type_pub;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr is_armable_sub;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr intersec_sub;
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr hori_line_sub;
        rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr vert_line_sub;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
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
