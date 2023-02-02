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
    int high_fd;
    struct sockaddr_in server;
    uint8_t mav_sysid = 0;
    int ipc_fd;
    pid_t tgt_proc = -1;

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

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

    high_fd = ipc_fd;
    if (uart_fd > high_fd) high_fd = uart_fd;

    printf("hello\n");

    while (gogogo) {
        FD_ZERO(&rfds);
        FD_SET(uart_fd, &rfds);
        FD_SET(ipc_fd, &rfds);

        tv.tv_sec = 10;
        tv.tv_usec = 0;

        retval = select(high_fd + 1, &rfds, NULL, NULL, &tv);
        if (retval > 0) {
            if (FD_ISSET(uart_fd, &rfds)) {
                avail = read(uart_fd, buf, 512);
                for (int i = 0; i < avail; i++) {
                    if (mavlink_parse_char(0, buf[i], &msg, &status)) {
                        if (msg.sysid == 255) continue;
                        //printf("recv msg ID %d, seq %d\n", msg.msgid, msg.seq);
			if (msg.sysid != mav_sysid) {
				mav_sysid = msg.sysid;
				printf("found MAV %d\n", msg.sysid);
			}
#if 0
                        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            if (hb.custom_mode == COPTER_MODE_LAND || hb.custom_mode == COPTER_MODE_RTL) {
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
                        }
#endif
                    }
                }
            }
            if (FD_ISSET(ipc_fd, &rfds)) {
                double tag_pose[6] = {0};
                if (recv(ipc_fd, tag_pose, sizeof(tag_pose), 0) > 0) {
                    gettimeofday(&tv, NULL);
                    float q[4] = {1, 0, 0, 0};
                    int tag_id = tag_pose[0];
                    double cam_r = tag_pose[1];
                    double cam_d = tag_pose[2];
                    double cam_f = tag_pose[3];
                    mavlink_msg_landing_target_pack(mav_sysid, MY_COMP_ID, &msg, tv.tv_sec*1000000+tv.tv_usec, tag_id, MAV_FRAME_BODY_FRD, 0, 0, sqrt(cam_r*cam_r+cam_d*cam_d+cam_f*cam_f), 0, 0, -cam_d, cam_r, cam_f, q, 0, 1);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    write(uart_fd, buf, len);
                }
            }
        }
    }

    close(uart_fd);
    close(ipc_fd);
    if (tgt_proc > 0) {
        kill(SIGINT, tgt_proc);
        wait(NULL);
    }
    return 0;
}
