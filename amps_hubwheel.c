/*!
 * Copyright (c) 2024 Taisyu Shibata
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

int uart_open(const char *portname) {
    int fd;
    struct termios tty;

    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fd < 0) {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B115200); // 115200 baud
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo,
                     // no canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN]  = 0; // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
                                     // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    printf("UART opened successfully.\n");    
    return fd;
}

void uart_close(int fd) {
    close(fd);
}

int uart_write(int fd, const unsigned char *data, int len) {
    printf("Sending command...\n");
    // データ内容をログに出力
    printf("Data being sent: ");
    for (int i = 0; i < len; i++) {
        printf("%02x ", data[i]);
    }
    printf("\n");

    int wlen = write(fd, data, len);
    if (wlen != len) {
        printf("Error from write: %d, %d\n", wlen, errno);
        return -1;
    }
    printf("Command sent.\n");    
    return 0;
}

int uart_read(int fd, unsigned char *buffer, int len) {
    printf("Reading response...\n");
    int rdlen = 0;
    int totalWaitTime = 0;

    while (totalWaitTime < 5000) {
        rdlen = read(fd, buffer, len);
        if (rdlen > 0) {
            // Output the received data
            printf("Read %d bytes\n", rdlen);
            for (int i = 0; i < rdlen; i++) {
                printf(" %02x", buffer[i]);
            }
            printf("\n");
            return rdlen;
        } else if (rdlen < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // ノンブロッキングモードの場合、データがまだない
                printf("Waiting for data...\n");
                usleep(100000);  // 0.1秒待機
                totalWaitTime += 100;
            } else {
                // シリアスなエラーが発生した
                printf("Error from read: %d\n", errno);
                return -1;  // エラーを返して終了
            }
        } else {
            printf("No data received, waiting...\n");
            usleep(100000);  // 0.1秒待機
            totalWaitTime += 100;
        }
    }
    printf("Read timeout occurred.\n");
    return 0;
}

int main() {
    int fd = uart_open("/dev/ttyTHS0"); // Adjust as per your UART port
    if (fd < 0) return -1;

    const unsigned char command[] = {0x01, 0x52, 0x70, 0x19, 0x00, 0x00, 0x00, 0x00, 0x04}; // Example command
    if (uart_write(fd, command, sizeof(command)) < 0) {
        uart_close(fd);
        return -1;
    }

    sleep(1); // Wait for the response

    unsigned char read_buf[100];
    if (uart_read(fd, read_buf, sizeof(read_buf)) <= 0) {
        uart_close(fd);
        return -1;
    }

    uart_close(fd);
    return 0;
}
