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
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include <sys/wait.h>
#include <poll.h>
#include <time.h>
#include "mavlink/ardupilotmega/mavlink.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

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

struct timeval tv_intersect = {0, 0};
float intersect_cog[2] = {0, 0};

struct __attribute__((packed)) lines_3d {
//where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line.
    float hori_x;
    float hori_y;
    float hori_z;
    float hori_vx;
    float hori_vy;
    float hori_vz;
};

float angle_between_vectors(float v1x, float v1y, float v1z, float v2x, float v2y, float v2z) {
    return acosf((v1x * v2x + v1y * v2y + v1z * v2z) / (sqrtf(v1x * v1x + v1y * v1y + v1z * v1z) * sqrtf(v2x * v2x + v2y * v2y + v2z * v2z)));
}

void intersect_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    //printf("intersection %f %f\n", msg->x, msg->y);
    intersect_cog[0] = msg->x;
    intersect_cog[1] = msg->y;
    gettimeofday(&tv_intersect, NULL);
}

int main(int argc, char *argv[]) {
    struct pollfd pfds[MY_NUM_PFDS];
    struct timeval tv;
    int retval, uart_fd;
    unsigned int len;
    unsigned char buf[1024];
    ssize_t avail;
    mavlink_status_t status;
    mavlink_message_t msg;
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    struct sockaddr_un ipc_addr, ipc_addr2;
    uint8_t mav_sysid = 0;
    int ipc_fd, ipc_fd2;
    int parse_error = 0, packet_rx_drop_count = 0;
    bool att_rcved = false;
    float cur_yaw = 0;
    int missions[] = {
        MOVE_UP,
        MOVE_LEFT,
        MOVE_DOWN,
        MOVE_LEFT,
        MOVE_UP,
        MOVE_RIGHT,
        MOVE_DOWN,
        MOVE_RIGHT,
        MOVE_UP,
        MOVE_LEFT,
        MOVE_DOWN,
        MOVE_LEFT,
        MOVE_UP,
        MOVE_RIGHT,
        MOVE_DOWN,
        MOVE_RIGHT,
        LAND,
    };
    int mission_idx = -1;
    float cur_vio_x = 0, cur_vio_y = 0, cur_vio_z = 0;
    float wp_vio_x = 0, wp_vio_y = 0, wp_vio_z = 0;
    struct lines_3d detected_structs;
    struct lines_3d last_detected_structs;
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

    memset(&last_detected_structs, 0, sizeof(last_detected_structs));

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mavlink_udp");
    auto navi_pub = node->create_publisher<std_msgs::msg::String>("navi", 1);
    auto roll_pub = node->create_publisher<std_msgs::msg::Float32>("roll", 1);
    auto sonar_pub = node->create_publisher<sensor_msgs::msg::Range>("sonar", 1);
    auto vel_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("tgt_vel", 1);
    auto intersec_sub = node->create_subscription<geometry_msgs::msg::Point>("templateCOG", 1, intersect_callback);

    auto rng = sensor_msgs::msg::Range();
    rng.header.frame_id = "body";
    rng.radiation_type = 0;
    rng.field_of_view = 1.0;
    rng.min_range = 0.2;
    rng.max_range = 7.5;

    if (argc > 1)
        uart_fd = open(argv[1], O_RDWR| O_NOCTTY);
    else
        uart_fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY);
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

    if ((ipc_fd = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&ipc_addr, 0, sizeof(ipc_addr));
    ipc_addr.sun_family = AF_UNIX;
    strcpy(ipc_addr.sun_path, SERVER_PATH);
    unlink(SERVER_PATH);
    if (bind(ipc_fd, (const struct sockaddr *)&ipc_addr, sizeof(ipc_addr)) < 0) {
        printf("bind local failed\n");
        return 1;
    }

    if ((ipc_fd2 = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&ipc_addr2, 0, sizeof(ipc_addr2));
    ipc_addr2.sun_family = AF_UNIX;
    strcpy(ipc_addr2.sun_path, SERVER_PATH2);
    unlink(SERVER_PATH2);
    if (bind(ipc_fd2, (const struct sockaddr *)&ipc_addr2, sizeof(ipc_addr2)) < 0) {
        printf("bind local failed\n");
        return 1;
    }

    pfds[0].fd= uart_fd;
    pfds[0].events = POLLIN;
    pfds[1].fd= ipc_fd;
    pfds[1].events = POLLIN;
    pfds[2].fd= ipc_fd2;
    pfds[2].events = POLLIN;

    printf("hello\n");

    memset(&status, 0, sizeof(status));

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        retval = poll(pfds, MY_NUM_PFDS, 10);
        if (retval > 0) {
            if (pfds[0].revents & POLLIN) {
                avail = read(uart_fd, buf, 1024);
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
                                write(uart_fd, buf, len);

                                mavlink_msg_set_gps_global_origin_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 247749434, 1210443077, 100000, tv.tv_sec*1000000+tv.tv_usec);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                            if (hb.custom_mode == COPTER_MODE_GUIDED) {
                                if (mission_idx == -1) {
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "mission start";
                                    navi_pub->publish(txt);
                                    mission_idx = 0;
                                    navi_status = SEARCH_STRUCT_CROSS;
                                    move_status = HOVER;
                                } else {
                                    float x_diff = cur_vio_x - wp_vio_x;
                                    float y_diff = cur_vio_y - wp_vio_y;
                                    float z_diff = cur_vio_z - wp_vio_z;
                                    if ((x_diff * x_diff + y_diff * y_diff + z_diff * z_diff) > (MAX_WP_DIST_M * MAX_WP_DIST_M)) {
                                        auto txt = std_msgs::msg::String();
                                        txt.data = "exceed MAX_WP_DIST_M";
                                        navi_pub->publish(txt);
                                        mavlink_msg_set_mode_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, COPTER_MODE_BRAKE);
                                        len = mavlink_msg_to_send_buffer(buf, &msg);
                                        write(uart_fd, buf, len);
                                    }
                                }
                            } else {
                                mission_idx = -1;
                            }
                            if (!att_rcved) {
                                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE, 100'000, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                            if (!dist_sensor_rcved) {
                                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_DISTANCE_SENSOR, 100'000, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
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
                            rng.header.stamp = node->get_clock()->now();
                            rng.range = dist_sensor.current_distance * 0.01;
                            sonar_pub->publish(rng);
                        }
                    }
                }
            }
            if (pfds[1].revents & POLLIN) {
                float pose[10];
                if (recv(ipc_fd, pose, sizeof(pose), 0) > 0) {
                    float covar[21] = {0};
                    pose[2]=-pose[2];
                    pose[3]=-pose[3];
                    if (mav_sysid != 0) {
                        gettimeofday(&tv, NULL);
                        mavlink_msg_att_pos_mocap_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, pose, pose[4], -pose[5], -pose[6], covar);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd, buf, len);
                        mavlink_msg_vision_speed_estimate_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, pose[7], -pose[8], -pose[9], covar, 0);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd, buf, len);
                    }
                    cur_vio_x = pose[4];
                    cur_vio_y = pose[5];
                    cur_vio_z = pose[6];
                }
            }
            if (pfds[2].revents & POLLIN) {
                if (recv(ipc_fd2, &detected_structs, sizeof(detected_structs), 0) > 0) {
                    if (yaw_adj_cd > 0) yaw_adj_cd--;
                    if (mission_idx >= 0 && (unsigned int)mission_idx < (sizeof(missions) / sizeof(missions[0]))) {
                        if (navi_status == SEARCH_STRUCT_CROSS) {
                            gettimeofday(&tv, NULL);
                            if (((tv.tv_sec - tv_intersect.tv_sec) * 1'000'000 + tv.tv_usec - tv_intersect.tv_usec) < 500'000) {
                                if ((move_status == MOVE_LEFT && intersect_cog[0] < 0.2) || (move_status == MOVE_RIGHT && intersect_cog[0] > 0.8) || (move_status == MOVE_UP && intersect_cog[1] < 0.2)) {
                                    slow_down = true;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "intersection detected on edge, slowing down";
                                    navi_pub->publish(txt);
                                } else {
                                    slow_down = false;
                                    auto txt = std_msgs::msg::String();
                                    txt.data = "arrival at waypoint " + std::to_string(mission_idx);
                                    navi_pub->publish(txt);
                                    navi_status = PASS_STRUCT_CROSS;
                                    move_status = missions[mission_idx];
                                    // AP_NOTIFY_TONE_LOUD_WP_COMPLETE
                                    // to noisy, cannot hear it
                                    /*mavlink_msg_play_tune_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, "MFT200L8G>C3", "");
                                    len = mavlink_msg_to_send_buffer(buf, &msg);
                                    write(uart_fd, buf, len);*/
                                    wp_vio_x = cur_vio_x;
                                    wp_vio_y = cur_vio_y;
                                    wp_vio_z = cur_vio_z;
                                }
                            }
                        } else if (navi_status == PASS_STRUCT_CROSS) {
                            gettimeofday(&tv, NULL);
                            if (((tv.tv_sec - tv_intersect.tv_sec) * 1'000'000 + tv.tv_usec - tv_intersect.tv_usec) > 1'500'000) {
                                auto txt = std_msgs::msg::String();
                                txt.data = "intersection passed";
                                navi_pub->publish(txt);
                                navi_status = SEARCH_STRUCT_CROSS;
                                if (mission_idx >= 0) mission_idx++;
                            }
                        }
                        if (move_status == MOVE_RIGHT || move_status == MOVE_LEFT) {
                            float vel_r = 0.2f;
                            float vel_d = 0;
                            float vel_f = 0;
                            uint16_t type_mask = 0xdc7;
                            if (move_status == MOVE_LEFT) vel_r = -0.2f;
                            if (slow_down) vel_r = vel_r * 0.6f;
                            if (detected_structs.hori_x == 0) {
                            } else {
                                // find the intersection point of the hori struct line and the plane y = 0
                                float t = -detected_structs.hori_y / detected_structs.hori_vy;
                                float z = detected_structs.hori_z + detected_structs.hori_vz * t;
                                float x = detected_structs.hori_x + detected_structs.hori_vx * t;
                                if (last_struct_dist == 0 || fabsf(x - last_struct_dist) < 0.6f) {
                                    last_struct_dist = x;
                                    if (z > 0.2f) low_confirm_cnt++; else low_confirm_cnt = 0;
                                    if (z < -0.2f) high_confirm_cnt++; else high_confirm_cnt = 0;
                                    if (x < CLOSE_DIST_M) close_confirm_cnt++; else close_confirm_cnt = 0;
                                    if (x > FAR_DIST_M) far_confirm_cnt++; else far_confirm_cnt = 0;
                                    if (low_confirm_cnt > 1) {
                                        vel_d = -0.12f;
                                        auto txt = std_msgs::msg::String();
                                        txt.data = "too low, move up";
                                        navi_pub->publish(txt);
                                    } else if (high_confirm_cnt > 1) {
                                        vel_d = 0.12f;
                                        auto txt = std_msgs::msg::String();
                                        txt.data = "too high, move down";
                                        navi_pub->publish(txt);
                                    }
                                    if (close_confirm_cnt > 1) {
                                        adj_cnt++;
                                        vel_f = -0.12f;
                                        auto txt = std_msgs::msg::String();
                                        txt.data = "too close, move away";
                                        navi_pub->publish(txt);
                                    } else if (far_confirm_cnt > 1) {
                                        adj_cnt++;
                                        vel_f = 0.12f;
                                        auto txt = std_msgs::msg::String();
                                        txt.data = "too far, move close";
                                        navi_pub->publish(txt);
                                    }
                                    if (adj_cnt > 5) {
                                        adj_cnt = 0;
                                        far_confirm_cnt = 0;
                                        close_confirm_cnt = 0;
                                    }

                                    float vx, vy;
                                    if (detected_structs.hori_vy < 0) {
                                        vx = -detected_structs.hori_vx;
                                        vy = -detected_structs.hori_vy;
                                    } else {
                                        vx = detected_structs.hori_vx;
                                        vy = detected_structs.hori_vy;
                                    }
                                    float angle_y_hori = acosf(vy);
                                    if (angle_y_hori > 0.15f) align_confirm_cnt++; else align_confirm_cnt = 0;
                                    if (align_confirm_cnt > 1 && yaw_adj_cd == 0) {
                                        yaw_adj_cd = 10;
                                        align_confirm_cnt = 0;
                                        //printf("angle_y_hori %f %f\n", angle_y_hori, vx);
                                        if (vx > 0) tgt_yaw = cur_yaw + angle_y_hori; else tgt_yaw = cur_yaw - angle_y_hori;
                                        auto txt = std_msgs::msg::String();
                                        txt.data = "adjust heading " + (vx > 0 ? std::string("cw ") : std::string("ccw ")) + std::to_string(angle_y_hori * 180 / M_PI) + " from " + std::to_string(cur_yaw * 180 / M_PI) + " to " + std::to_string(tgt_yaw * 180 / M_PI);
                                        navi_pub->publish(txt);
                                    }
                                }
                            }
                            if (yaw_adj_cd > 0) type_mask = 0x9c7;
                            if (fc_prx_too_close) {
                                vel_f = -0.15f;
                                auto txt = std_msgs::msg::String();
                                txt.data = "sonar: too close, move away";
                                navi_pub->publish(txt);
                            }
                            gettimeofday(&tv, NULL);
                            mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+(uint32_t)(tv.tv_usec*0.001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, type_mask, 0, 0, 0, vel_f, vel_r, vel_d, 0, 0, 0, tgt_yaw, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);

                            auto twist_stamped = geometry_msgs::msg::TwistStamped();
                            twist_stamped.header.stamp = node->get_clock()->now();
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
                            if (fc_prx_too_close) {
                                vel_f = -0.15f;
                                auto txt = std_msgs::msg::String();
                                txt.data = "sonar: too close, move away";
                                navi_pub->publish(txt);
                            }
                            gettimeofday(&tv, NULL);
                            mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+(uint32_t)(tv.tv_usec*0.001), mav_sysid, 1, MAV_FRAME_BODY_OFFSET_NED, 0xdc7, 0, 0, 0, vel_f, vel_r, vel_d, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);

                            auto twist_stamped = geometry_msgs::msg::TwistStamped();
                            twist_stamped.header.stamp = node->get_clock()->now();
                            twist_stamped.header.frame_id = "body";
                            twist_stamped.twist.linear.x = vel_f;
                            twist_stamped.twist.linear.y = -vel_r;
                            twist_stamped.twist.linear.z = -vel_d;
                            vel_pub->publish(twist_stamped);
                        } else if (move_status == LAND) {
                            mavlink_msg_set_mode_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, COPTER_MODE_LAND);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                        }
                    }
                    if (detected_structs.hori_x != 0) {
                        last_detected_structs = detected_structs;
                    }
                }
            }
        }
    }

    close(uart_fd);
    close(ipc_fd);
    close(ipc_fd2);
    unlink(SERVER_PATH);
    unlink(SERVER_PATH2);

    rclcpp::shutdown();

    printf("bye\n");

    return 0;
}
