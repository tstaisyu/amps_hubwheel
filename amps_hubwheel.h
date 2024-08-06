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

#ifndef MOTOR_COMMANDS_H
#define MOTOR_COMMANDS_H

#include <stdint.h>

typedef uint8_t byte;

// モーターのID
#define RIGHT_MOTOR_ID 0x01
#define LEFT_MOTOR_ID 0x02

// オペレーションアドレス
#define OPERATION_MODE_ADDRESS 0x7017
#define EMERGENCY_STOP_ADDRESS 0x701F
#define CONTROL_WORD_ADDRESS 0x7019

// コマンドサイズ指定
#define WRITE_COMMAND 0x51
#define ENABLE_COMMAND 0x52
#define VEL_SEND_COMMAND 0x54

// コマンドデータ
#define OPERATION_MODE_SPEED_CONTROL 0x00000003
#define DISABLE_EMERGENCY_STOP 0x00000000
#define ENABLE_MOTOR 0x0000000F

int uart_open(const char *portname);
void uart_close(int fd);
int uart_write(int fd, const unsigned char *data, int len);
int uart_read(int fd, unsigned char *buffer, int len);
void sendCommand(int fd, byte motorID, uint16_t address, byte command, uint32_t data);
void initMotor(int fd, byte motorID);
int checkMotorResponse(int fd);
void sendSpeedCommand(int fd, byte motorID, uint32_t speed);
void initializeMotors(int uart1_fd, int uart2_fd);
void handleMotorCommands(int uart1_fd, int uart2_fd);

#endif // MOTOR_COMMANDS_H