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
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "mavlink/ardupilotmega/mavlink.h"

#define MY_COMP_ID 191
#define MY_NUM_PFDS 3
#define SERVER_PATH "/tmp/chobits_server"

int main(int argc, char *argv[]) {
    struct pollfd pfds[MY_NUM_PFDS];
    struct timeval tv;
    int retval, uart_fd;
    unsigned int len;
    unsigned char buf[512];
    ssize_t avail;
    mavlink_status_t status;
    mavlink_message_t msg;
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    struct sockaddr_un ipc_addr;
    struct sockaddr_in ipc_addr2;
    uint8_t mav_sysid = 0;
    int ipc_fd, ipc_fd2;
    bool no_hr_imu = true;
    bool no_att_q = true;
    float att_q_x =0, att_q_y = 0, att_q_z = 0, att_q_w = 0;
    int64_t time_offset_us = 0;
    uint64_t last_us = 0;
    int send_goal = 5;
    bool no_local_pos = true;
    float local_n = 0, local_e = 0, local_d = 0;
    float vis_n = 0, vis_e = 0, vis_d = 0;
    float diff_local_vis_n =0, diff_local_vis_e = 0, diff_local_vis_d = 0;

    uart_fd = open("/dev/ttyAMA0", O_RDWR);
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

    if ((ipc_fd2 = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&ipc_addr2, 0, sizeof(ipc_addr2));
    ipc_addr2.sin_family = AF_INET;
    ipc_addr2.sin_port = htons(17500);
    ipc_addr2.sin_addr.s_addr = inet_addr("0.0.0.0");
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

    ros::init(argc, argv, "test_node_outside_catkin_ws");
    ros::NodeHandle ros_nh;
    ROS_INFO("It worked!");

    ros::Publisher imu_pub = ros_nh.advertise<sensor_msgs::Imu>("/chobits/imu", 100);
    ros::Publisher goal_pub = ros_nh.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 1);

    while (ros::ok()) {
        retval = poll(pfds, MY_NUM_PFDS, 5000);
        if (retval > 0) {
            if (pfds[0].revents & POLLIN) {
                avail = read(uart_fd, buf, 512);
                for (int i = 0; i < avail; i++) {
                    if (mavlink_parse_char(0, buf[i], &msg, &status)) {
                        if (msg.sysid == 255) continue;
                        //printf("recv msg ID %d, seq %d\n", msg.msgid, msg.seq);
                        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            if (msg.sysid != mav_sysid) {
                                mav_sysid = msg.sysid;
                                printf("found MAV %d\n", msg.sysid);
                            }
                            if (time_offset_us == 0) {
                                gettimeofday(&tv, NULL);
                                mavlink_msg_timesync_pack(mav_sysid, MY_COMP_ID, &msg, 0, tv.tv_sec*1000000+tv.tv_usec, mav_sysid, 1); //fill timesync with us instead of ns
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);

                                mavlink_msg_system_time_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);

                                mavlink_msg_set_gps_global_origin_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 247749434, 1210443077, 100000, tv.tv_sec*1000000+tv.tv_usec);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                            if (no_hr_imu) {
                                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_HIGHRES_IMU, 10000, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                            if (no_att_q) {
                                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 10000, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                            if (no_local_pos) {
                                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_LOCAL_POSITION_NED, 100000, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                            if (hb.custom_mode == COPTER_MODE_GUIDED) {
                                send_goal--;
                                if (send_goal == 0) {
                                    nav_msgs::Path ros_wps;
                                    geometry_msgs::PoseStamped ros_tgt;
                                    ros_wps.header.stamp = ros::Time::now();
                                    ros_wps.header.frame_id = "world";
                                    ros_tgt.header.stamp = ros::Time::now();
                                    ros_tgt.header.frame_id = "world";
                                    ros_tgt.pose.position.x = 6.5;
                                    ros_tgt.pose.position.y = 0;
                                    ros_tgt.pose.position.z = 1.3;
                                    ros_wps.poses.push_back(ros_tgt);
                                    goal_pub.publish(ros_wps);
                                    diff_local_vis_n = local_n - vis_n;
                                    diff_local_vis_e = local_e - vis_e;
                                    diff_local_vis_d = local_d - vis_d;
                                    printf("diff_local_vis %.2f %.2f %.2f\n", diff_local_vis_n, diff_local_vis_e, diff_local_vis_d);
                                }
                            } else {
                                send_goal = 5;
                            }
                        } else if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) {
                            mavlink_timesync_t ts;
                            mavlink_msg_timesync_decode(&msg, &ts);
                            if (ts.tc1 != 0) {
                                time_offset_us = ts.ts1 - ts.tc1;
                                printf("time offset %ld\n", time_offset_us);
                            }
                        } else if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                            mavlink_statustext_t txt;
                            mavlink_msg_statustext_decode(&msg, &txt);
                            printf("fc: %s\n", txt.text);
   	                } else if (msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
                            no_hr_imu = false;
                            mavlink_highres_imu_t hr_imu;
                            mavlink_msg_highres_imu_decode(&msg, &hr_imu); // time_usec is time since boot
                            if (time_offset_us > 0 && hr_imu.time_usec > last_us) {
                                last_us = hr_imu.time_usec;
                                sensor_msgs::Imu imu_msg;
                                int64_t ts_us = hr_imu.time_usec + time_offset_us;
                                imu_msg.header.stamp.sec = ts_us / 1000000;
                                imu_msg.header.stamp.nsec = (ts_us % 1000000) * 1000;
                                imu_msg.header.frame_id = "world";
                                imu_msg.linear_acceleration.x = hr_imu.xacc;
                                imu_msg.linear_acceleration.y = hr_imu.yacc;
                                imu_msg.linear_acceleration.z = hr_imu.zacc;
                                imu_msg.angular_velocity.x = hr_imu.xgyro;
                                imu_msg.angular_velocity.y = hr_imu.ygyro;
                                imu_msg.angular_velocity.z = hr_imu.zgyro;
                                imu_msg.orientation.w = att_q_w;
                                imu_msg.orientation.x = att_q_x;
                                imu_msg.orientation.y = att_q_y;
                                imu_msg.orientation.z = att_q_z;
                                imu_pub.publish(imu_msg);
                            }
                        } else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {
                            no_att_q = false;
                            mavlink_attitude_quaternion_t att_q;
                            mavlink_msg_attitude_quaternion_decode(&msg, &att_q);
                            att_q_w = att_q.q1;
                            att_q_x = att_q.q2;
                            att_q_y = att_q.q3;
                            att_q_z = att_q.q4;
                        } else if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
                            no_local_pos = false;
                            mavlink_local_position_ned_t local_pos;
                            mavlink_msg_local_position_ned_decode(&msg, &local_pos);
                            local_n = local_pos.x;
                            local_e = local_pos.y;
                            local_d = local_pos.z;
                        }
                    }
                }
            }
            if (pfds[1].revents & POLLIN) {
                float pose[10];
                if (recv(ipc_fd, pose, sizeof(pose), 0) > 0) {
                    vis_n = pose[4];
                    vis_e = -pose[5];
                    vis_d = -pose[6];
                    float covar[21] = {0};
                    pose[2]=-pose[2];
                    pose[3]=-pose[3];
                    gettimeofday(&tv, NULL);
                    mavlink_msg_att_pos_mocap_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, pose, pose[4], -pose[5], -pose[6], covar);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    write(uart_fd, buf, len);
                    gettimeofday(&tv, NULL);
                    mavlink_msg_vision_speed_estimate_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, pose[7], -pose[8], -pose[9], covar, 0);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    write(uart_fd, buf, len);

                    if (pose[4] > 6) {
                        gettimeofday(&tv, NULL);
                        mavlink_msg_set_mode_pack(mav_sysid, MY_COMP_ID, &msg, mav_sysid, 1, 9); //land
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd, buf, len);
                    }
                }
            }
            if (pfds[2].revents & POLLIN) {
                float planner_msg[9];
                if (recv(ipc_fd2, &planner_msg, sizeof(planner_msg), 0) > 0) {
                    gettimeofday(&tv, NULL);
                    mavlink_msg_set_position_target_local_ned_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+tv.tv_usec*0.001, 0, 0, MAV_FRAME_LOCAL_NED, 0xc00, planner_msg[0], -planner_msg[1], -planner_msg[2]+diff_local_vis_d-0.2, planner_msg[3], -planner_msg[4], -planner_msg[5], planner_msg[6], -planner_msg[7], -planner_msg[8], 0, 0);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    write(uart_fd, buf, len);
                }
            }
        }
    }

    close(uart_fd);
    close(ipc_fd);

    printf("bye\n");

    return 0;
}
