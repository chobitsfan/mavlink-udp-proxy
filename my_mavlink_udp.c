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
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include <sys/wait.h>
#include "mavlink/ardupilotmega/mavlink.h"

#define MY_COMP_ID 191
#define TAKEOFF_ALT_M 20

bool gogogo = true;

void sig_handler(int signum) {
    gogogo = false;
}

int main(int argc, char *argv[]) {
    fd_set rfds;
    struct timeval tv;
    int retval, uart_fd;
    unsigned int len;
    unsigned char buf[512];
    ssize_t avail;
    mavlink_status_t status;
    mavlink_message_t msg;
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    //int hb_count = 0;
    int sock_fd, high_fd;
    struct sockaddr_in server;
    struct sockaddr_in to_cv2x;
    char tx_buf[512];
    uint8_t mav_sysid = 0;
    int ipc_fd;
    int wait_rot = 0;
    bool global_pos_rcvd = false;
    float relative_alt_m = 15;
    bool wait_guided = false;
    bool wait_arm = false;
    pid_t tgt_proc = -1;
    bool landing = false;

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    //gettimeofday(&tv, NULL);
    //printf("gettimeofday %d\n", tv.tv_sec);

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
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);
    // Save tty settings, also checking for error
    if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return 1;
    }

    if ((sock_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&server, 0, sizeof(server));
    /* Set up the server name */
    server.sin_family      = AF_INET;            /* Internet Domain    */
    server.sin_port        = htons(5519);  //Server Port
    server.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock_fd, (const struct sockaddr *)&server, sizeof(server)) < 0) {
        printf("bind failed\n");
        return 1;
    }

    if ((ipc_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&server, 0, sizeof(server));
    /* Set up the server name */
    server.sin_family      = AF_INET;            /* Internet Domain    */
    server.sin_port        = htons(17510);  //Server Port
    server.sin_addr.s_addr = inet_addr("127.0.0.1");
    if (bind(ipc_fd, (const struct sockaddr *)&server, sizeof(server)) < 0) {
        printf("bind local failed\n");
        return 1;
    }

    memset(&to_cv2x, 0, sizeof(to_cv2x));
    to_cv2x.sin_family = AF_INET;
    to_cv2x.sin_port = htons(5555);
    to_cv2x.sin_addr.s_addr = inet_addr("192.168.1.100");

    high_fd = sock_fd;
    if (uart_fd > high_fd) high_fd = uart_fd;
    if (ipc_fd > high_fd) high_fd = ipc_fd;

    printf("hello\n");

    while (gogogo) {
        FD_ZERO(&rfds);
        FD_SET(uart_fd, &rfds);
        FD_SET(sock_fd, &rfds);
        FD_SET(ipc_fd, &rfds);

        tv.tv_sec = 10;
        tv.tv_usec = 0;

        retval = select(high_fd + 1, &rfds, NULL, NULL, &tv);
        if (retval > 0) {
            if (FD_ISSET(uart_fd, &rfds)) {
                avail = read(uart_fd, buf, 512);
                //printf("recv %d bytes\n", avail);
                for (int i = 0; i < avail; i++) {
                    if (mavlink_parse_char(0, buf[i], &msg, &status)) {
                        if (msg.sysid == 255) continue;
                        //printf("recv msg ID %d, seq %d\n", msg.msgid, msg.seq);
						if (msg.sysid != mav_sysid) {
							mav_sysid = msg.sysid;
							printf("found MAV %d\n", msg.sysid);
							//server.sin_port = htons(19500 + mav_sysid);
						}
                        //if (gcs_connected) {
                        //    len = mavlink_msg_to_send_buffer(tx_buf, &msg);
                        //    sendto(sock_fd, tx_buf, len, 0, (const struct sockaddr *)&client, sizeof(client));
                        //}
                        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            if (!global_pos_rcvd) {
                                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000, 0, 0, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len);
                            }
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            if (wait_guided) {
                                wait_guided = false;
                                if (hb.custom_mode == COPTER_MODE_GUIDED) {
                                    mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
                                    len = mavlink_msg_to_send_buffer(buf, &msg);
                                    write(uart_fd, buf, len);
                                    wait_arm = true;
                                }
                            } else if (wait_arm) {
                                wait_arm = false;
                                if (hb.base_mode & 128) {
                                    mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M);
                                    len = mavlink_msg_to_send_buffer(buf, &msg);
                                    write(uart_fd, buf, len);
                                }
                            }
                            if (hb.base_mode & 128) {
                                landing = false;
                            } else if (landing) {
                                landing = false;
                                tx_buf[0]='4';
                                tx_buf[1]=',';
                                tx_buf[2]='1';
                                tx_buf[3]=0;
                                sendto(sock_fd, tx_buf, 4, 0, (struct sockaddr*)&to_cv2x, sizeof(to_cv2x));
                            }
                            if (hb.custom_mode == COPTER_MODE_LAND || hb.custom_mode == COPTER_MODE_RTL) {
                                landing = true;
                                if (tgt_proc < 0) {
                                    printf("start apriltag_plnd\n");
                                    tgt_proc = fork();
                                    if (tgt_proc == 0) {
                                        execl("/home/pi/src/apriltag_plnd/apriltag_plnd","/home/pi/src/apriltag_plnd/apriltag_plnd",(char*)NULL);
                                    }
                                }
                            } else {
                                if (tgt_proc > 0) {
                                   printf("kill apriltag_plnd\n");
                                   kill(tgt_proc, SIGINT);
                                   if (waitpid(tgt_proc, NULL, WNOHANG) == tgt_proc) tgt_proc = -1;
                                }
                            }
                            /*hb_count++;
                            if (hb_count > 5) {
                                hb_count = 0;
                                gettimeofday(&tv, NULL);
                                //printf("gettimeofday %d\n", tv.tv_sec);
                                mavlink_msg_system_time_pack(255, 1, &msg, tv.tv_sec * 1000000ULL + tv.tv_usec, 0);
                                len = mavlink_msg_to_send_buffer(tx_buf, &msg);
                                write(uart_fd, tx_buf, len);
                            }*/
                        } else if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                            global_pos_rcvd = true;
                            mavlink_global_position_int_t global_pos_int;
                            mavlink_msg_global_position_int_decode(&msg, &global_pos_int);
                            //printf("%d\n", global_pos_int.relative_alt);
                            if (global_pos_int.relative_alt > 3000) {
								relative_alt_m = global_pos_int.relative_alt * 0.001f;
                            }
                            len = snprintf(tx_buf, sizeof(tx_buf), "1,%.7f,%.7f", global_pos_int.lon*1e-7, global_pos_int.lat*1e-7);
                            sendto(sock_fd, tx_buf, len, 0, (struct sockaddr*)&to_cv2x, sizeof(to_cv2x));
                            //printf(tx_buf);
                            //printf("\n");
                        }
                    }
                }
            }
            if (FD_ISSET(sock_fd, &rfds)) {
                /*len = sizeof(client);
                avail = recvfrom(sock_fd, buf, 512, 0, (struct sockaddr*)&client, &len);
                if (avail > 0) {
		    gcs_connected = true;
                    write(uart_fd, buf, avail);
                }*/
                avail = recv(sock_fd, buf, sizeof(buf), 0);
                if (avail > 0) {
                    buf[avail]=0;
                    printf("rcv [%s] from cv2x\n", buf);
                    if (buf[0] == '2' && buf[1] == ',') {
                        double lat, lon;
                        retval = sscanf((char*)buf+2, "%lf,%lf", &lon, &lat);
                        if (retval == 2) {
                            printf("recv %f,%f from cv2x\n", lon, lat);
                            if (relative_alt_m > (TAKEOFF_ALT_M - 1)) {
								gettimeofday(&tv, NULL);
								mavlink_msg_set_position_target_global_int_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000+(uint32_t)(tv.tv_usec*0.001), 0, 0, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 3576, lat*1e7, lon*1e7, relative_alt_m, 0, 0, 0, 0, 0, 0, 0, 0);
								len = mavlink_msg_to_send_buffer(buf, &msg);
								write(uart_fd, buf, len);
								printf("cmd mav %d %d %f\n", (int)(lat*1e7), (int)(lon*1e7), relative_alt_m);
								char str_buf[50];
								sprintf(str_buf, "cv2x tgt %.2f %.2f", lon, lat);
								mavlink_msg_statustext_pack(mav_sysid, MY_COMP_ID, &msg, MAV_SEVERITY_INFO, str_buf, 0, 0);
								len = mavlink_msg_to_send_buffer(buf, &msg);
								write(uart_fd, buf, len);
                            }
                        }
                    } else if (buf[0] == '3' && buf[1] == ',') {
                        if (buf[2] == '1') {
                            printf("set mav to guided\n");
                            mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_DO_SET_MODE, 0, 1, COPTER_MODE_GUIDED, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                            wait_guided = true;
                            mavlink_msg_statustext_pack(mav_sysid, MY_COMP_ID, &msg, MAV_SEVERITY_INFO, "cv2x takeoff", 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                        } else if (buf[2] == '2') {
                            mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_DO_SET_MODE, 0, 1, COPTER_MODE_LAND, 0, 0, 0, 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                            mavlink_msg_statustext_pack(mav_sysid, MY_COMP_ID, &msg, MAV_SEVERITY_INFO, "cv2x land", 0, 0);
                            len = mavlink_msg_to_send_buffer(buf, &msg);
                            write(uart_fd, buf, len);
                        }
                    }
                }
            }
            if (FD_ISSET(ipc_fd, &rfds)) {
                double tag_pose[6] = {0};
                if (recv(ipc_fd, tag_pose, sizeof(tag_pose), 0) > 0) {
                    if (tag_pose[2] < 4 && tag_pose[3] != 0 && wait_rot == 0) {
			//calc yaw from rotation matrix, https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
                        float yaw_offset = atan2(tag_pose[3], tag_pose[4])*(180/M_PI);
                        //printf("yaw %f\n", yaw_offset);
                        float abs_yaw_offset = fabsf(yaw_offset);
                        if (abs_yaw_offset > 10) {
                            wait_rot = 60;
                            if (yaw_offset > 0) {
                                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_CONDITION_YAW, 0, abs_yaw_offset, 30, 1, 1, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len); 
                            } else {
                                mavlink_msg_command_long_pack(mav_sysid, MY_COMP_ID, &msg, 0, 0, MAV_CMD_CONDITION_YAW, 0, abs_yaw_offset, 30, -1, 1, 0, 0, 0);
                                len = mavlink_msg_to_send_buffer(buf, &msg);
                                write(uart_fd, buf, len); 
                            }
                        }
                    } else {
                        gettimeofday(&tv, NULL);
                        float q[4] = {1, 0, 0, 0};
                        mavlink_msg_landing_target_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, tag_pose[5], MAV_FRAME_BODY_FRD, 0, 0, sqrt(tag_pose[0]*tag_pose[0]+tag_pose[1]*tag_pose[1]+tag_pose[2]*tag_pose[2]), 0, 0, -tag_pose[1], tag_pose[0], tag_pose[2], q, 0, 1);
                        len = mavlink_msg_to_send_buffer(buf, &msg);
                        write(uart_fd, buf, len); 
                    }
                    if (wait_rot > 0) wait_rot--;
                }
            }
        }
    }

    close(uart_fd);
    close(sock_fd);
    close(ipc_fd);
    if (tgt_proc > 0) {
        kill(SIGINT, tgt_proc);
        wait(NULL);
    }
    return 0;
}
