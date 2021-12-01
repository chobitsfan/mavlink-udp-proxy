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
#include "ardupilotmega/mavlink.h"

bool gogogo = true;

void sig_handler(int signum) {
    gogogo = false;
}

int main(int argc, char *argv[]) {
    fd_set rfds;
    struct timeval tv;
    int retval, uart_fd, len;
    unsigned char buf[512];
    ssize_t avail;
    mavlink_status_t status;
    mavlink_message_t msg;
    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    //int hb_count = 0;
    int sock_fd, high_fd;
    struct sockaddr_in server;
    unsigned char tx_buf[512];

    signal(SIGINT, sig_handler);

    //gettimeofday(&tv, NULL);
    //printf("gettimeofday %d\n", tv.tv_sec);

    uart_fd = open("/dev/ttyMSM1", O_RDWR);

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
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
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
    server.sin_port        = htons(atoi(argv[2]));               /* Server Port        */
    server.sin_addr.s_addr = inet_addr(argv[1]); /* Server's Address   */

    high_fd = sock_fd;
    if (uart_fd > high_fd) high_fd = uart_fd;

    printf("hello\n");

    while (gogogo) {
        FD_ZERO(&rfds);
        FD_SET(uart_fd, &rfds);
        FD_SET(sock_fd, &rfds);

        tv.tv_sec = 10;
        tv.tv_usec = 0;

        retval = select(high_fd + 1, &rfds, NULL, NULL, &tv);
        if (retval > 0) {
            if (FD_ISSET(uart_fd, &rfds)) {
                avail = read(uart_fd, buf, 512);
                //printf("recv %d bytes\n", avail);
                for (int i = 0; i < avail; i++) {
                    if (mavlink_parse_char(0, buf[i], &msg, &status)) {
                        printf("recv msg ID %d, seq %d\n", msg.msgid, msg.seq);
                        len = mavlink_msg_to_send_buffer(tx_buf, &msg);
                        sendto(sock_fd, tx_buf, len, 0, (const struct sockaddr *)&server, sizeof(server));
                        /*if (msg.msgid == 0) {
                            hb_count++;
                            if (hb_count > 5) {
                                hb_count = 0;
                                gettimeofday(&tv, NULL);
                                //printf("gettimeofday %d\n", tv.tv_sec);
                                mavlink_msg_system_time_pack(255, 1, &msg, tv.tv_sec * 1000000ULL + tv.tv_usec, 0);
                                len = mavlink_msg_to_send_buffer(tx_buf, &msg);
                                write(uart_fd, tx_buf, len);
                            }
                        }*/
                    }
                }
            }
            if (FD_ISSET(sock_fd, &rfds)) {
                avail = read(sock_fd, buf, 512);
                write(uart_fd, buf, avail);
            }
        }
    }

    return 0;
}
