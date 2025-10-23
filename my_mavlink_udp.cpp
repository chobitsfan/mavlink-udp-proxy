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
#include <Eigen/Dense>
#include "mavlink/ardupilotmega/mavlink.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

// ROS coordinate system, x axis = vehicle front

#define MY_COMP_ID 191
#define MY_NUM_PFDS 1

class MavRosNode : public rclcpp::Node {
    public:
        MavRosNode(int uart_fd) : Node("mavlink_ros"), uart_fd_(uart_fd) {
            avd_dir_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("avoid_direction", rclcpp::QoS(1).best_effort().durability_volatile(), [this](const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg) { avd_callback(twist_msg); });
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(1).best_effort().durability_volatile(), [this](const nav_msgs::msg::Odometry::SharedPtr odom_msg) { odom_callback(odom_msg); });
            tgt_p_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_point", rclcpp::QoS(1).best_effort().durability_volatile());
            tgt_dir_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("target_direction", rclcpp::QoS(1).best_effort().durability_volatile());
            uart_timer_ = this->create_wall_timer(2ms, [this](){ timer_callback(); });
        }

    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
            mavlink_message_t msg;
            float covar[21] = {0};
            float q[4];
            unsigned int len;
            geometry_msgs::msg::Pose *pose = &odom_msg->pose.pose;
            geometry_msgs::msg::Vector3 *v = &odom_msg->twist.twist.linear;
            if (mav_sysid != 0 && time_offset_ns != 0) {
                q[0] = pose->orientation.w;
                q[1] = pose->orientation.x;
                q[2] = -pose->orientation.y;
                q[3] = -pose->orientation.z;
                int64_t odom_fc_us = (((int64_t)(odom_msg->header.stamp.sec) * 1000000000 + odom_msg->header.stamp.nanosec) - time_offset_ns) / 1000;
                //printf("odom_us, odom_fc_us %ld %ld\n", odom_msg->header.stamp.sec * 1000000000L + odom_msg->header.stamp.nanosec, odom_fc_us);
                mavlink_msg_att_pos_mocap_pack(mav_sysid, MY_COMP_ID, &msg, odom_fc_us, q, pose->position.x, -pose->position.y, -pose->position.z, covar);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
                mavlink_msg_vision_speed_estimate_pack(mav_sysid, MY_COMP_ID, &msg, odom_fc_us, v->x, -v->y, -v->z, covar, 0);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
            }
        }

        void avd_callback(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg) {
            struct timeval tv;
            mavlink_message_t msg;
            unsigned int len;
            if (in_guided) {
                gettimeofday(&tv, NULL);
                mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+(uint32_t)(tv.tv_usec*0.001), mav_sysid, 1, MAV_FRAME_BODY_FRD, 0xdc7, 0, 0, 0, twist_msg->twist.linear.x, -twist_msg->twist.linear.y, -twist_msg->twist.linear.z, 0, 0, 0, 0, 0);
                len = mavlink_msg_to_send_buffer(buf, &msg);
                write(uart_fd_, buf, len);
            }
        }

        void timer_callback() {
            static int parse_error = 0;
            static int packet_rx_drop_count = 0;
            static int timesync_counter = 4;
            unsigned int len;
            ssize_t avail;
            mavlink_status_t status;
            mavlink_message_t msg;
            struct timespec tp;

            memset(&status, 0, sizeof(status));

            avail = read(uart_fd_, buf, 1024);
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

                            struct timeval tv;
                            gettimeofday(&tv, NULL);
                            mavlink_msg_system_time_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);

                            mavlink_msg_set_gps_global_origin_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 247749434, 1210443077, 100000, tv.tv_sec*1000000+tv.tv_usec);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }
                        if (hb.custom_mode == COPTER_MODE_GUIDED) {
#if 0
                            if (!in_guided) {
                                struct timespec tp;
                                mavlink_message_t msg;
                                clock_gettime(CLOCK_MONOTONIC, &tp);
                                mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tp.tv_sec*1000+tp.tv_nsec/1000000, mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0xdf8, 50.0f, 0, -0.5f, 0, 0, 0, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd_, buf, len);
                            }
                            in_guided = true;
#else
                            if (!in_guided) {
                                Eigen::Vector3f tgt_body(30, 0 ,0);
                                tgt_local = cur_att * tgt_body;
                                struct timespec tp;
                                clock_gettime(CLOCK_MONOTONIC, &tp);
                                geometry_msgs::msg::PointStamped p;
                                p.header.frame_id = "map";
                                p.header.stamp.sec = tp.tv_sec;
                                p.header.stamp.nanosec = tp.tv_nsec;
                                p.point.x = tgt_local.x();
                                p.point.y = tgt_local.y();
                                p.point.z = tgt_local.z();
                                tgt_p_pub_->publish(p);
                            }
                            in_guided = true;
#endif
                        } else {
                            in_guided = false;
                        }
                        if (timesync_counter > 3) {
                            timesync_counter = 0;
                            clock_gettime(CLOCK_MONOTONIC, &tp);
                            mavlink_msg_timesync_pack(mav_sysid, MY_COMP_ID, &msg, 0, (int64_t)tp.tv_sec * 1000000000 + tp.tv_nsec, mav_sysid, 1);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        } else timesync_counter++;
                        if (!att_rcved) {
                            mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 50'000, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }
                        if (!local_pos_rcved) {
                            mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50'000, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd_, buf, len);
                        }
                    } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                        mavlink_statustext_t txt;
                        mavlink_msg_statustext_decode(&msg, &txt);
                        printf("fc: %s\n", txt.text);
                    } else if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
                        mavlink_timesync_t sync;
                        mavlink_msg_timesync_decode(&msg, &sync);
                        if (sync.tc1 > 0) {
                            time_offset_ns = sync.ts1 - sync.tc1;
                            printf("time offset: %ld ns\n", time_offset_ns);
                        }
                    } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {
                        att_rcved = true;
                        mavlink_attitude_quaternion_t att;
                        mavlink_msg_attitude_quaternion_decode(&msg, &att);
                        cur_att.w() = att.q1;
                        cur_att.x() = att.q2;
                        cur_att.y() = -att.q3;
                        cur_att.z() = -att.q4;
                    } else if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                        local_pos_rcved = true;
                        if (in_guided) {
                            mavlink_local_position_ned_t local_pos;
                            mavlink_msg_local_position_ned_decode(&msg, &local_pos);
                            Eigen::Vector3f cur_pos_local(local_pos.x, -local_pos.y , -local_pos.z);
                            Eigen::Vector3f tgt_dir_local = tgt_local - cur_pos_local;
                            if (tgt_dir_local.squaredNorm() < 1) { // close enough
                                mavlink_msg_set_mode_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, COPTER_MODE_BRAKE);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd_, buf, len);
                            } else {
                                Eigen::Vector3f tgt_dir_body = cur_att.conjugate() * tgt_dir_local;
                                tgt_dir_body.normalize();
                                struct timespec tp;
                                clock_gettime(CLOCK_MONOTONIC, &tp);
                                geometry_msgs::msg::TwistStamped twist_msg;
                                twist_msg.header.frame_id = "body";
                                twist_msg.header.stamp.sec = tp.tv_sec;
                                twist_msg.header.stamp.nanosec = tp.tv_nsec;
                                twist_msg.twist.linear.x = tgt_dir_body.x();
                                twist_msg.twist.linear.y = tgt_dir_body.y();
                                twist_msg.twist.linear.z = tgt_dir_body.z();
                                tgt_dir_pub_->publish(twist_msg);
                            }
                        }
                    }
                }
            }
        }

        int uart_fd_;
        unsigned char buf[1024];
        uint8_t mav_sysid = 0;
        bool in_guided = false;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr tgt_p_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr tgt_dir_pub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr avd_dir_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::TimerBase::SharedPtr uart_timer_;
        int64_t time_offset_ns = 0;
        bool att_rcved = false;
        bool local_pos_rcved = false;
        Eigen::Quaternionf cur_att;
        Eigen::Vector3f tgt_local{10, 0, 0};
};

int main(int argc, char *argv[]) {
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;

    rclcpp::init(argc, argv);

    int uart_fd;
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

    printf("uart ok\n");

    auto node = std::make_shared<MavRosNode>(uart_fd);
    rclcpp::spin(node);
    rclcpp::shutdown();

    close(uart_fd);

    printf("bye\n");

    return 0;
}
