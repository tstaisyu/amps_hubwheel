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
#include <stdint.h>

typedef uint8_t byte;

// 上位バイトを取得する関数
uint8_t highByte(uint16_t value) {
    return (uint8_t)(value >> 8);
}

// 下位バイトを取得する関数
uint8_t lowByte(uint16_t value) {
    return (uint8_t)(value & 0xFF);
}

#define MOTOR_ID 0x01
#define OPERATION_MODE_ADDRESS 0x7017
#define EMERGENCY_STOP_ADDRESS 0x701F
#define CONTROL_WORD_ADDRESS 0x7019
#define WRITE_COMMAND 0x51
#define ENABLE_COMMAND 0x52
#define OPERATION_MODE_SPEED_CONTROL 0x00000003
#define DISABLE_EMERGENCY_STOP 0x00000000
#define ENABLE_MOTOR 0x0000000F

int uart_open(const char *portname);
void uart_close(int fd);
int uart_write(int fd, const unsigned char *data, int len);
int uart_read(int fd, unsigned char *buffer, int len);
void sendCommand(int fd, byte motorID, uint16_t address, byte command, uint32_t data);
void initMotor(int fd, byte motorID);

int checkMotorResponse(int fd) {
    unsigned char buffer[100];
    int len = uart_read(fd, buffer, sizeof(buffer));
    if (len > 0) {
        // データを解析して期待する応答であるか確認
        printf("Response received: ");
        for (int i = 0; i < len; i++) {
            printf("%02x ", buffer[i]);
            if ((i + 1) % 10 == 0) { // 10バイトごとに改行
                printf("\n");
            }
        }
        if (len % 10 != 0) { // データの最後が10バイトの倍数でない場合は改行を追加
            printf("\n");
        }

        // モーターからの応答に応じた処理
        switch(buffer[1]) { // 2番目のバイトに応答コードがあると仮定
            case 0x61:
            case 0x62:
            case 0x64:
                printf("Command executed successfully.\n");
                return 1;
            case 0x50:
                printf("Error: Data length is wrong.\n");
                break;
            case 0x58:
                printf("Error: Object address is not writable.\n");
                break;
            case 0x5F:
                printf("Error: Object address does not exist.\n");
                break;
            default:
                printf("Received unknown response code.\n");
        }
    } else {
        printf("No response or error reading response.\n");
    }
    return 0;
}

void sendSpeedCommand(int fd, byte motorID, uint32_t speed) {
    // 速度コマンドを構築 (アドレスとコマンドは適宜調整)
    uint16_t speedCommandAddress = 0x70B2;  // 仮のアドレス
    sendCommand(fd, motorID, speedCommandAddress, WRITE_COMMAND, speed);
}

int main() {
    int fd = uart_open("/dev/ttyTHS0"); // Adjust as per your UART port
    if (fd < 0) return -1;

    initMotor(fd, MOTOR_ID);
    if (checkMotorResponse(fd)) {
        printf("Motor initialized successfully.\n");

        // モータに速度コマンドを送信
        sendSpeedCommand(fd, MOTOR_ID, 1000);  // 速度値は例です
        if (checkMotorResponse(fd)) {
            printf("Speed command accepted.\n");
        } else {
            printf("Failed to send speed command.\n");
        }
    } else {
        printf("Failed to initialize motor.\n");
    }

    uart_close(fd);
    return 0;
}


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


void sendCommand(int fd, byte motorID, uint16_t address, byte command, uint32_t data) {
    byte packet[] = {motorID, command, highByte(address), lowByte(address), 0, (byte)(data >> 24), (byte)(data >> 16), (byte)(data >> 8), (byte)data};
    byte checksum = 0;
    for (int i = 0; i < sizeof(packet); i++) {
        checksum += packet[i];
    }
    uart_write(fd, packet, sizeof(packet));
    uart_write(fd, &checksum, 1);
}

void initMotor(int fd, byte motorID) {
    sendCommand(fd, motorID, OPERATION_MODE_ADDRESS, WRITE_COMMAND, OPERATION_MODE_SPEED_CONTROL);
    usleep(100000);  // 100 ms delay
    sendCommand(fd, motorID, EMERGENCY_STOP_ADDRESS, WRITE_COMMAND, DISABLE_EMERGENCY_STOP);
    usleep(100000);  // 100 ms delay
    sendCommand(fd, motorID, CONTROL_WORD_ADDRESS, ENABLE_COMMAND, ENABLE_MOTOR);
    usleep(100000);  // 100 ms delay
}

