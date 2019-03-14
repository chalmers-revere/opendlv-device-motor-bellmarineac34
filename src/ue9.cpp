/*
 * Copyright (C) 2018 Ola Benderius
 * Based on example code from LabJack.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <sstream>

#ifdef WIN32
#include <winsock.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include "ue9.hpp"

#include <iostream>

/* Ranges */

// -5V to +5V
#define LJ_rgBIP5V 3

// 0V to +5V
#define LJ_rgUNI5V 103

// 0V to +2.5V
#define LJ_rgUNI2P5V 105

// 0V to +1.25Vz
#define LJ_rgUNI1P25V 107

// 0V to +0.625V
#define LJ_rgUNIP625V 109


/* timer clocks: */

// 750 khz
#define LJ_tc750KHZ 0

// system clock
#define LJ_tcSYS 1


/* timer modes */

// 16 bit PWM
#define LJ_tmPWM16 0

// 8 bit PWM
#define LJ_tmPWM8 1

// 32-bit rising to rising edge measurement
#define LJ_tmRISINGEDGES32 2

// 32-bit falling to falling edge measurement
#define LJ_tmFALLINGEDGES32 3

// duty cycle measurement
#define LJ_tmDUTYCYCLE 4

// firmware based rising edge counter
#define LJ_tmFIRMCOUNTER 5

// firmware counter with debounce
#define LJ_tmFIRMCOUNTERDEBOUNCE 6

// frequency output
#define LJ_tmFREQOUT 7

// Quadrature
#define LJ_tmQUAD 8

// stops another timer after n pulses
#define LJ_tmTIMERSTOP 9

// read lower 32-bits of system timer
#define LJ_tmSYSTIMERLOW 10

// read upper 32-bits of system timer
#define LJ_tmSYSTIMERHIGH 11

// 16-bit rising to rising edge measurement
#define LJ_tmRISINGEDGES16 12

// 16-bit falling to falling edge measurement
#define LJ_tmFALLINGEDGES16 13


Ue9::Ue9(std::string const &a_address, int32_t a_port_command, int32_t a_port_stream):
    m_calibration_info(),
    m_address(a_address),
    m_stream_running(false),
    m_port_command(a_port_command),
    m_port_stream(a_port_stream),
    m_socket_command(),
    m_socket_stream(),
    m_stream_channel_count(-1),
    m_stream_resolution(-1),
    m_stream_packet_count(0)
{
  m_socket_command = Connect(a_address, a_port_command);
  ReadCalibrationInfo();
}

Ue9::~Ue9()
{
  Disconnect(m_socket_command);
  delete m_calibration_info;
}

int32_t Ue9::Connect(std::string const &a_address, int32_t a_port)
{
#ifdef WIN32
  WSADATA info;
  struct hostent *he;

  if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
    throw std::runtime_error("Cannot initilize winsock.");
  }
#endif

  int32_t s = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == -1) {
    throw std::runtime_error("Could not create TCP socket.");
  }

  struct sockaddr_in address;
  memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_port = htons(a_port);

#ifdef WIN32
  he = gethostbyname(a_address);
  address.sin_addr = *((struct in_addr *) he->h_addr);
#else
  address.sin_addr.s_addr = inet_addr(a_address.c_str());

  int32_t buffer_size = 128 * 1024;
  setsockopt(s, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(int));
#endif

  int32_t res = connect(s, (struct sockaddr *) &address, sizeof(address));
  if (res < 0) {
    throw std::runtime_error("Could not connect to TCP socket.");
  }

  return s;
}

void Ue9::Disconnect(int32_t a_socket)
{
  if (a_socket != -1) {
#ifdef WIN32
    closesocket(a_socket);
    WSACleanup();
#else
    shutdown(a_socket, SHUT_RDWR);
#endif
  }
}

void Ue9::FlushStream()
{
  uint8_t send_buffer[2];

  send_buffer[0] = (uint8_t) (0x08);
  send_buffer[1] = (uint8_t) (0x08);

  int32_t res = send(m_socket_command, send_buffer, 2, 0);
  if (res < 2) {
    throw std::runtime_error("Failed to send.");
  }

  uint8_t rec_buffer[2];
  res = recv(m_socket_command, rec_buffer, 2, 0);
  if (res < 2) {
    throw std::runtime_error("Failed to receive.");
  }

  if (rec_buffer[0] != (uint8_t) (0x08) || rec_buffer[1] != (uint8_t) (0x08)) {
    throw std::runtime_error("Read buffer, invalid command.");
  }
}

void Ue9::InitStream(int32_t a_channel_count, int32_t a_resolution, int32_t a_scan_interval)
{
  if (m_stream_running) {
    StopStream();
  }

  FlushStream();

  m_socket_stream = Connect(m_address, m_port_stream);
  m_stream_channel_count = a_channel_count;
  m_stream_resolution = a_resolution;

  int32_t send_count = 12 + 2 * a_channel_count;
  uint8_t *send_buffer = (uint8_t *) malloc(sizeof(uint8_t) * send_count);
  
  send_buffer[1] = (uint8_t) (0xF8); // Command byte
  send_buffer[2] = a_channel_count + 3; // Data words : channels + 3
  send_buffer[3] = (uint8_t) (0x11); // Extended command number
  send_buffer[6] = (uint8_t) a_channel_count;
  send_buffer[7] = a_resolution;
  send_buffer[8] = 0; // SettlingTime = 0
  send_buffer[9] = 0; // ScanConfig: scan pulse and external scan 
                                  // trigger disabled, stream clock 
                                  // frequency = 4 MHz
  send_buffer[10] = (uint8_t) (a_scan_interval & 0x00FF);
  send_buffer[11] = (uint8_t) (a_scan_interval / 256);
 
  int32_t channel_num[] = {0, 1, 2, 3, 7, 9, 11, 13};
  for (int32_t i = 0; i < a_channel_count; i++) {
    send_buffer[12 + i * 2] = channel_num[i];
    send_buffer[13 + i * 2] = (uint8_t) (0x08); //BipGain (Bip = bipolar, Gain = 1)
  }

  GetExtendedChecksum(send_buffer, send_count);

  int32_t res = send(m_socket_command, send_buffer, send_count, 0);
  if (res < 20) {
    free(send_buffer);
    throw std::runtime_error("Failed to send.");
  }

  uint8_t rec_buffer[8];
  res = recv(m_socket_command, rec_buffer, 8, 0);
  if (res < 8) {
    free(send_buffer);
    throw std::runtime_error("Failed to receive.");
  }

  uint16_t checksum_total = GetExtendedChecksum16(rec_buffer, 8);
  if ((uint8_t) ((checksum_total / 256) & 0xff) != rec_buffer[5]) {
    free(send_buffer);
    throw std::runtime_error("Read buffer, bad checksum 16 (MSB).");
  }

  if ((uint8_t) (checksum_total & 0xff) != rec_buffer[4]) {
    free(send_buffer);
    throw std::runtime_error("Read buffer, bad checksum 16 (LSB).");
  }

  if (GetExtendedChecksum8(rec_buffer) != rec_buffer[0]) {
    free(send_buffer);
    throw std::runtime_error("Read buffer, bad checksum 8.");
  }

  if (rec_buffer[1] != (uint8_t)(0xF8) || rec_buffer[2] != (uint8_t)(0x01) ||
      rec_buffer[3] != (uint8_t)(0x11) || rec_buffer[7] != (uint8_t)(0x00)) {
    free(send_buffer);
    throw std::runtime_error("Read buffer, invalid command.");
  }

  if (rec_buffer[6] != 0) {
    int32_t error_code = rec_buffer[6];
    free(send_buffer);
    std::stringstream ss;
    ss << "Read buffer, error: " << error_code;
    throw std::runtime_error(ss.str());
  }
}

void Ue9::StartStream(int32_t a_channel_count, int32_t a_resolution,
    int32_t a_scan_interval)
{
  InitStream(a_channel_count, a_resolution, a_scan_interval);

  uint8_t send_buffer[2];

  send_buffer[0] = (uint8_t) (0xA8); // Checksum
  send_buffer[1] = (uint8_t) (0xA8); // Command byte

  int32_t res = send(m_socket_command, send_buffer, 2, 0);
  if (res < 2) {
    throw std::runtime_error("Failed to send.");
  }

  uint8_t rec_buffer[4];
  res = recv(m_socket_command, rec_buffer, 4, 0);
  if (res < 4) {
    throw std::runtime_error("Failed to receive.");
  }

  if (rec_buffer[1] != (uint8_t) (0xA9) || rec_buffer[3] != (uint8_t) (0x00)) {
    throw std::runtime_error("Read buffer, invalid command.");
  }

  if (rec_buffer[2] != 0) {
    int32_t error_code = rec_buffer[2];
    std::stringstream ss;
    ss << "Read buffer, error: " << error_code;
    throw std::runtime_error(ss.str());
  }

  m_stream_running = true;
}

void Ue9::StopStream()
{
  if (!m_stream_running) {
    return;
  }

  uint8_t send_buffer[2];

  send_buffer[0] = (uint8_t)(0xB0); // Checksum
  send_buffer[1] = (uint8_t)(0xB0); // Command byte

  int32_t res = send(m_socket_command, send_buffer, 2, 0);
  if (res < 2) {
    throw std::runtime_error("Failed to send.");
  }

  uint8_t rec_buffer[4];
  res = recv(m_socket_command, rec_buffer, 4, 0);
  if(res < 4) {
    throw std::runtime_error("Failed to receive.");
  }

  if(rec_buffer[1] != (uint8_t) (0xB1) || rec_buffer[3] != (uint8_t) (0x00)) {
    throw std::runtime_error("Read buffer, invalid command.");
  }

  if (rec_buffer[2] != 0) {
    int32_t error_code = rec_buffer[2];
    std::stringstream ss;
    ss << "Read buffer, error: " << error_code;
    throw std::runtime_error(ss.str());
  }

  Disconnect(m_socket_stream);
  FlushStream();
  
  m_stream_running = false;
}

std::vector<std::vector<double> > Ue9::ParseStream(int32_t a_read_multiplier)
{ 
  uint8_t *rec_buffer = (uint8_t *) malloc(sizeof(uint8_t) * 46 * a_read_multiplier);
  
  int32_t res = 0;
  for (int32_t i = 0; i < 46 * a_read_multiplier; i += res) {
    res = recv(m_socket_stream, rec_buffer + i, 46 * a_read_multiplier - i, 0);
    if (res == 0) {
      throw std::runtime_error("Failed to receive.");
    }
  }
  
  int32_t frame_count = a_read_multiplier * 16 / m_stream_channel_count;

  std::vector<std::vector<double> > voltages;
  for (int32_t i = 0; i < frame_count; i++) {
    std::vector<double> frame;
    voltages.push_back(frame);
  }

  int32_t channel_index = 0;
  int32_t frame_index = 0;
  
  for (int32_t i = 0; i < a_read_multiplier; i++) {

    m_stream_packet_count++;
    //std::cout << m_stream_packet_count << std::endl;

    uint16_t checksum_total = GetExtendedChecksum16(rec_buffer + i * 46, 46);
    if ((uint8_t) ((checksum_total / 256) & 0xff) != rec_buffer[i * 46 + 5]) {
      free(rec_buffer);
      throw std::runtime_error("Read buffer, bad checksum 16 (MSB).");
    }

    if ((uint8_t) (checksum_total & 0xff) != rec_buffer[i * 46 + 4]) {
      free(rec_buffer);
      throw std::runtime_error("Read buffer, bad checksum 16 (LSB).");
    }

    checksum_total = GetExtendedChecksum8(rec_buffer + i * 46);
    if (checksum_total != rec_buffer[i * 46]) {
      free(rec_buffer);
      throw std::runtime_error("Read buffer, bad checksum 8.");
    }

    if (rec_buffer[i * 46 + 1] != (uint8_t) (0xF9) || 
        rec_buffer[i * 46 + 2] != (uint8_t) (0x14) ||
        rec_buffer[i * 46 + 3] != (uint8_t) (0xC0) ) {
      free(rec_buffer);
      throw std::runtime_error("Read buffer, invalid command.");
    }

    if(rec_buffer[i * 46 + 11] != 0) {
      int32_t error_code = rec_buffer[i * 46 + 11];
      free(rec_buffer);
      std::stringstream ss;
      ss << "Read buffer, error: " << error_code;
      throw std::runtime_error(ss.str());
    }

    int32_t packet_index_local = (m_stream_packet_count - 1) % 256;
    int32_t packet_index = rec_buffer[i * 46 + 10];
    if (packet_index_local != packet_index) {
      free(rec_buffer);
      std::stringstream ss;
      ss << "Packet count mismatch, was " << packet_index_local <<
          " should be " << packet_index;
      throw std::runtime_error(ss.str());
    }

//    int32_t back_log = rec_buffer[i * 46 + 45] & 0x7F;

    if ((rec_buffer[i * 46 + 45] & 128) == 128) {
//      std::cout << "Comm buffer overflow detected in packet " <<
//          m_stream_packet_count << ", backlog: " << back_log << std::endl;
      throw std::runtime_error("Overflow.");
    }

    for (int32_t j = 12; j < 43; j += 2) {
      uint16_t voltage_bytes = (uint16_t) rec_buffer[i * 46 + j] +
          (uint16_t) rec_buffer[i * 46 + j + 1] * 256;

      double voltage;
      ConvertBinaryToAnalogVoltage((uint8_t) (0x08), m_stream_resolution,
          voltage_bytes, &voltage); 
      
      voltages[frame_index].push_back(voltage);
      channel_index++;

      if (channel_index == m_stream_channel_count) {
        frame_index++;
        channel_index = 0;
      }
    }
  }

  return voltages;
}

void Ue9::GetExtendedChecksum(uint8_t *a_b, int32_t a_n)
{
  uint16_t val = GetExtendedChecksum16(a_b, a_n);
  a_b[4] = (uint8_t) (val & 0xff);
  a_b[5] = (uint8_t) ((val / 256) & 0xff);
  a_b[0] = GetExtendedChecksum8(a_b);
}

uint8_t Ue9::GetChecksum(uint8_t *a_b, int32_t a_n)
{
  uint16_t val1 = 0;
  for (int32_t i = 1; i < a_n; i++) {
    val1 += (uint16_t) a_b[i];
  }

  uint16_t val2 = val1 / 256;
  val1 = (val1 - 256 * val2) + val2;
  val2 = val1 / 256;

  return (uint8_t) ((val1 - 256 * val2) + val2);
}


uint16_t Ue9::GetExtendedChecksum16(uint8_t *a_b, int32_t a_n)
{
  //Sums bytes 6 to n-1 to a unsigned 2 byte value
  int32_t val = 0;
  for (int32_t i = 6; i < a_n; i++) {
    val += (uint16_t) a_b[i];
  }
  return val;
}

uint8_t Ue9::GetExtendedChecksum8(uint8_t *a_b)
{
  /* Sum bytes 1 to 5. Sum quotient and remainder of 256 division. Again, sum
     quotient and remainder of 256 division. Return result as uint8. */

  //Sums bytes 1 to 5. Sums quotient and remainder of 256 division. Again, sums 
  //quotient and remainder of 256 division.
  int32_t val1 = 0;
  for (int32_t i = 1; i < 6; i++) {
    val1 += (uint16_t) a_b[i];
  }

  int32_t val2 = val1 / 256;
  val1 = (val1 - 256 * val2) + val2;
  val2 = val1 / 256;

  return (uint8_t) ((val1 - 256 * val2) + val2);  
}

void Ue9::ReadCalibrationInfo()
{
  uint8_t send_buffer[8];
  uint8_t rec_buffer[136];

  // Reading block 0 from memory
  send_buffer[1] = (uint8_t) (0xF8); // Command byte
  send_buffer[2] = (uint8_t) (0x01); // Number of data words
  send_buffer[3] = (uint8_t) (0x2A); // Extended command number
  send_buffer[6] = (uint8_t) (0x00);

  send_buffer[7] = (uint8_t) (0x00); // Blocknum = 0
  GetExtendedChecksum(send_buffer, 8);

  int32_t res = send(m_socket_command, send_buffer, 8, 0);
  if (res < 8) {
    throw std::runtime_error("Failed to send.");
  }

  res = recv(m_socket_command, rec_buffer, 136, 0);
  if (res < 136) {
    throw std::runtime_error("Failed to receive.");
  }

  if (rec_buffer[1] != (uint8_t) (0xF8) || rec_buffer[2] != (uint8_t) (0x41)
      || rec_buffer[3] != (uint8_t) (0x2A)) {
    throw std::runtime_error(
        "Received buffer at byte 1, 2, or 3 are not 0xA3, 0x01, 0x2A.");
  }

  m_calibration_info = new CalibrationInfo;

  // Block data starts on byte 8 of the buffer
  m_calibration_info->unipolarSlope[0] =
      ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 0);
  m_calibration_info->unipolarOffset[0] =
      ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 8);
  m_calibration_info->unipolarSlope[1] =
      ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 16);
  m_calibration_info->unipolarOffset[1] =
      ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 24);
  m_calibration_info->unipolarSlope[2] =
      ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 32);
  m_calibration_info->unipolarOffset[2] =
      ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 40);
  m_calibration_info->unipolarSlope[3] =
      ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 48);
  m_calibration_info->unipolarOffset[3] =
      ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 56);

  /* reading block 1 from memory */
  send_buffer[7] = (uint8_t) (0x01); //Blocknum = 1
  GetExtendedChecksum(send_buffer, 8);

  res = send(m_socket_command, send_buffer, 8, 0);
  if (res < 8) {
    throw std::runtime_error("Failed to send.");
  }

  res = recv(m_socket_command, rec_buffer, 136, 0);
  if (res < 136) {
    throw std::runtime_error("Failed to receive.");
  }

  if (rec_buffer[1] != (uint8_t) (0xF8) || rec_buffer[2] != (uint8_t) (0x41)
      || rec_buffer[3] != (uint8_t) (0x2A)) {
    throw std::runtime_error(
        "Received buffer at byte 1, 2, or 3 are not 0xA3, 0x01, 0x2A.");
  }

  //block data starts on byte 8 of the buffer
  m_calibration_info->bipolarSlope
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 0);
  m_calibration_info->bipolarOffset
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 8);

  /* reading block 2 from memory */
  send_buffer[7] = (uint8_t) (0x02); //Blocknum = 2
  GetExtendedChecksum(send_buffer, 8);

  res = send(m_socket_command, send_buffer, 8, 0);
  if (res < 8) {
    throw std::runtime_error("Failed to send.");
  }

  res = recv(m_socket_command, rec_buffer, 136, 0);
  if (res < 136) {
    throw std::runtime_error("Failed to receive.");
  }

  if (rec_buffer[1] != (uint8_t) (0xF8) || rec_buffer[2] != (uint8_t) (0x41)
      || rec_buffer[3] != (uint8_t) (0x2A)) {
    throw std::runtime_error(
        "Received buffer at byte 1, 2, or 3 are not 0xA3, 0x01, 0x2A.");
  }

  //block data starts on byte 8 of the buffer
  m_calibration_info->DACSlope[0]
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 0);
  m_calibration_info->DACOffset[0]
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 8);
  m_calibration_info->DACSlope[1]
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 16);
  m_calibration_info->DACOffset[1]
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 24);
  m_calibration_info->tempSlope
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 32);
  m_calibration_info->tempSlopeLow
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 48);
  m_calibration_info->calTemp
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 64);
  m_calibration_info->Vref
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 72);
  m_calibration_info->VrefDiv2
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 88);
  m_calibration_info->VsSlope
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 96);

  /* reading block 3 from memory */
  send_buffer[7] = (uint8_t) (0x03); //Blocknum = 3
  GetExtendedChecksum(send_buffer, 8);

  res = send(m_socket_command, send_buffer, 8, 0);
  if (res < 8) {
    throw std::runtime_error("Failed to send.");
  }

  res = recv(m_socket_command, rec_buffer, 136, 0);
  if (res < 136) {
    throw std::runtime_error("Failed to receive.");
  }

  if (rec_buffer[1] != (uint8_t) (0xF8) || rec_buffer[2] != (uint8_t) (0x41)
      || rec_buffer[3] != (uint8_t) (0x2A)) {
    throw std::runtime_error(
        "Received buffer at byte 1, 2, or 3 are not 0xA3, 0x01, 0x2A.");
  }

  // Block data starts on byte 8 of the buffer
  m_calibration_info->hiResUnipolarSlope
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 0);
  m_calibration_info->hiResUnipolarOffset
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 8);

  /* reading block 4 from memory */
  send_buffer[7] = (uint8_t)(0x04); // Blocknum = 4
  GetExtendedChecksum(send_buffer, 8);

  res = send(m_socket_command, send_buffer, 8, 0);
  if (res < 8) {
    throw std::runtime_error("Failed to send.");
  }

  res = recv(m_socket_command, rec_buffer, 136, 0);
  if (res < 136) {
    throw std::runtime_error("Failed to receive.");
  }

  if (rec_buffer[1] != (uint8_t) (0xF8) || rec_buffer[2] != (uint8_t) (0x41)
      || rec_buffer[3] != (uint8_t) (0x2A)) {
    throw std::runtime_error(
        "Received buffer at byte 1, 2, or 3 are not 0xA3, 0x01, 0x2A.");
  }

  // Block data starts on byte 8 of the buffer
  m_calibration_info->hiResBipolarSlope
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 0);
  m_calibration_info->hiResBipolarOffset
      = ConvertFPuint8ArrayToFPDouble(rec_buffer + 8, 8);
  m_calibration_info->prodID = 9;
}


double Ue9::ConvertFPuint8ArrayToFPDouble(uint8_t *a_buffer, int32_t a_start_index) 
{ 
  uint32_t result_dec = 0;
  uint32_t result_wh = 0;
  for (int32_t i = 0; i < 4; i++)
  {
    result_dec += a_buffer[a_start_index + i] * static_cast<uint32_t>(pow(2.0, (i*8.0)));
    result_wh += a_buffer[a_start_index + i + 4] * static_cast<uint32_t>(pow(2.0, (i*8.0)));
  }
  return (double) ((int) result_wh) + ((double) result_dec) / 4294967296.0;
}

void Ue9::ConvertBinaryToAnalogVoltage(uint8_t a_gain_bip, uint8_t a_resolution,
    uint16_t a_bytes_voltage, double *a_analog_voltage)
{
  double slope;
  double offset;
  if (a_resolution < 18) {
    switch (a_gain_bip) {
      case 0:
        slope = m_calibration_info->unipolarSlope[0];
        offset = m_calibration_info->unipolarOffset[0];
        break;
      case 1:
        slope = m_calibration_info->unipolarSlope[1];
        offset = m_calibration_info->unipolarOffset[1];
        break;
      case 2:
        slope = m_calibration_info->unipolarSlope[2];
        offset = m_calibration_info->unipolarOffset[2];
        break;
      case 3:
        slope = m_calibration_info->unipolarSlope[3];
        offset = m_calibration_info->unipolarOffset[3];
        break;
      case 8:
        slope = m_calibration_info->bipolarSlope;
        offset = m_calibration_info->bipolarOffset;
        break;
      default:
        throw std::runtime_error("Unknown gain/bipolar setting.");
    }
  } else {  //UE9 Pro high res
    switch (a_gain_bip) {
      case 0:
        slope = m_calibration_info->hiResUnipolarSlope;
        offset = m_calibration_info->hiResUnipolarOffset;
        break;
      case 8:
        slope = m_calibration_info->hiResBipolarSlope;
        offset = m_calibration_info->hiResBipolarOffset;
        break;
      default:
        throw std::runtime_error("Unknown gain/bipolar setting.");
    }
  }

  *a_analog_voltage = (slope * a_bytes_voltage) + offset;
}


void Ue9::ConvertAnalogToBinaryVoltage(int32_t a_dac_num, double a_analog_voltage,
    uint16_t *a_bytes_voltage)
{
  double slope;
  double offset;
  double temp_bytes_voltage;
  switch (a_dac_num) {
    case 0:
      slope = m_calibration_info->DACSlope[0];
      offset = m_calibration_info->DACOffset[0];
      break;
    case 1:
      slope = m_calibration_info->DACSlope[1];
      offset = m_calibration_info->DACOffset[1];
      break;
    default:
      throw std::runtime_error("Unknown DAC channel.");
  }

  temp_bytes_voltage = slope * a_analog_voltage + offset;

  //Checking to make sure a_bytes_voltage will be a value between 0 and 4095, 
  //or that a uint16_t overflow does not occur.  A too high a_analog_voltage 
  //(above 5 volts) or too low a_analog_voltage (below 0 volts) will cause a 
  //value not between 0 and 4095.
  if (temp_bytes_voltage < 0) {
    temp_bytes_voltage = 0;
  }

  if (temp_bytes_voltage > 4095) {
    temp_bytes_voltage = 4095;
  }

  *a_bytes_voltage = (uint16_t) temp_bytes_voltage; 
}

void Ue9::ConvertBinaryToAnalogTemperature(int32_t a_power_level,
    uint16_t a_bytes_temperature, double *a_analog_temperature)
{
  double slope;
  switch (a_power_level) {
    case 0: //high power
      slope = m_calibration_info->tempSlope;
      break;
    case 1: //low power
      slope = m_calibration_info->tempSlopeLow;
      break;
    default:
      throw std::runtime_error("Unknown power level.");
  }

  *a_analog_temperature = ((double) a_bytes_temperature) * slope;
}

void Ue9::ReadAin(long a_channel, double *a_voltage, long a_range,
    long a_resolution, long a_settling, long a_binary)
{
  uint8_t ain_gain;
  if (a_range == LJ_rgBIP5V) {
    ain_gain = 8;
  } else if (a_range == LJ_rgUNI5V) {
    ain_gain = 0;
  } else if (a_range == LJ_rgUNI2P5V) {
    ain_gain = 1;
  } else if (a_range == LJ_rgUNI1P25V) {
    ain_gain = 2;
  } else if (a_range == LJ_rgUNIP625V) {
    ain_gain = 3;
  } else {
    throw std::runtime_error("Invalid range.");
  }

  uint8_t io_type, channel, ain_m, ain_h;
  HelperSingleIo(4, (uint8_t) a_channel, ain_gain, (uint8_t) a_resolution,
      (uint8_t) a_settling, &io_type, &channel, NULL, &ain_m, &ain_h);

  uint16_t bytes_vt = ain_m + ain_h * 256;

  if (a_binary != 0) {
    *a_voltage = (double) bytes_vt;
  } else {
    if (a_channel == 133 || a_channel == 141) {
      ConvertBinaryToAnalogTemperature(0, bytes_vt, a_voltage);
    } else {
      ConvertBinaryToAnalogVoltage(ain_gain, (uint8_t) a_resolution, bytes_vt,
          a_voltage);
    }
  }
}


void Ue9::WriteDac(long a_channel, double a_voltage)
{
  uint16_t bytes_voltage;
  ConvertAnalogToBinaryVoltage((uint8_t) a_channel, a_voltage, &bytes_voltage);
  
  uint8_t io_type;
  uint8_t channel;

  HelperSingleIo(5, (uint8_t) a_channel, (uint8_t) (bytes_voltage & (0x00FF)),
      (uint8_t) ((bytes_voltage / 256) + 192), 0, &io_type, &channel, NULL, NULL,
      NULL);
}


void Ue9::ReadDin(long a_channel, long *a_state)
{
  if (a_channel > 22) {
    throw std::runtime_error("Invalid channel");
  }

  uint8_t state;
  HelperDioFeedback((uint8_t) a_channel, 0, &state);

  *a_state = state;
}

void Ue9::WriteDo(long a_channel, long a_state)
{
  if (a_channel > 22) {
    throw std::runtime_error("Invalid channel");
  }
  
  uint8_t state = (uint8_t) a_state;
  HelperDioFeedback((uint8_t) a_channel, 1, &state);
}


void Ue9::InitTcConfig(long *a_enable_timers, long *a_enable_counters,
    long a_timer_clock_base_index, long a_timer_clock_divisor,
    long *a_timer_modes, double *a_timer_values)
{
  uint8_t enable_mask = 128;  //Bit 7: UpdateConfig

  if (a_enable_counters[1] != 0) {
    enable_mask += 16; //Bit 4: Enable Counter1
  }

  if (a_enable_counters[0] |= 0) {
    enable_mask += 8;  //Bit 3: Enable Counter0
  }

  uint16_t timer_count = 0;
  uint16_t timer_stop_count = 0;
  
  uint8_t counter_mode[2];
  uint8_t timer_mode[6];
  uint16_t timer_value[6];

  for (int32_t i = 0; i < 6; i++) {
    if (a_enable_timers[i] != 0 && timer_stop_count == 0) {
      timer_count++;
      timer_mode[i] = (uint8_t) a_timer_modes[i];     //TimerMode
      timer_value[i] = (uint16_t) a_timer_values[i];  //TimerValue
    } else {
      timer_stop_count = 1;
      timer_mode[i] = 0;
      timer_value[i] = 0;
    }
  }
  enable_mask += timer_count;  //Bits 2-0: Number of Timers

  counter_mode[0] = 0;  //Counter0Mode
  counter_mode[1] = 0;  //Counter1Mode

  HelperTimerCounter((uint8_t) a_timer_clock_divisor, enable_mask,
      (uint8_t) a_timer_clock_base_index, 0, timer_mode, timer_value,
      counter_mode, NULL, NULL);
}


void Ue9::ReadTcValues(long *a_update_reset_timers, long *a_reset_counters,
    double *a_timer_values, double *a_counter_values)
{
  uint8_t counter_mode[2];
  uint32_t counter[2];

  uint8_t timer_mode[6];
  uint16_t timer_value[6];
  uint32_t timer[6];

  uint8_t update_reset = 0;
  for (int32_t i = 0; i < 6; i++) {
    update_reset += ((a_update_reset_timers[i] != 0) ? static_cast<uint32_t>(pow(2.0, i)) : 0);
    timer_mode[i] = 0;
    timer_value[i] = 0;
  }

  for (int32_t i = 0; i < 2; i++) {
    update_reset += ((a_reset_counters[i] != 0) ? static_cast<uint32_t>(pow(2.0, 6.0 + i)) : 0);
    counter_mode[i] = 0;
  }

  HelperTimerCounter(0, 0, 0, update_reset, timer_mode, timer_value,
      counter_mode, timer, counter);

  for (int32_t i = 0; i < 6; i++) {
    a_timer_values[i] = timer[i];
  }

  for (int32_t i = 0; i < 2; i++) {
    a_counter_values[i] = counter[i];
  }
}


void Ue9::HelperSingleIo(uint8_t a_in_io_type, uint8_t a_channel,
    uint8_t a_dir_bip_gain_dacl, uint8_t a_state_res_dach, uint8_t a_settling_time,
    uint8_t *a_out_io_type, uint8_t *a_out_channel, uint8_t *a_out_dir_ain_l,
    uint8_t *a_out_state_ain_m, uint8_t *a_out_ain_h)
{
  uint8_t send_buffer[8], rec_buffer[8];

  send_buffer[1] = (uint8_t) (0xA3); // Command byte
  send_buffer[2] = a_in_io_type;
  send_buffer[3] = a_channel;
  send_buffer[4] = a_dir_bip_gain_dacl;
  send_buffer[5] = a_state_res_dach;
  send_buffer[6] = a_settling_time;
  send_buffer[7] = 0; // Reserved

  send_buffer[0] = GetChecksum(send_buffer, 8);

  int32_t res = send(m_socket_command, send_buffer, 8, 0);
  if (res < 8) {
    throw std::runtime_error("Failed to send.");
  }

  res = recv(m_socket_command, rec_buffer, 8, 0);
  if (res < 8) {
    throw std::runtime_error("Failed to receive.");
  }

  if (((uint8_t) GetChecksum(rec_buffer, 8)) != rec_buffer[0]) {
    throw std::runtime_error("Bad checksum.");
  }

  if (rec_buffer[1] != (uint8_t) (0xA3)) {
    throw std::runtime_error("Invalid command byte.");
  }

  if (a_out_io_type != NULL) {
    *a_out_io_type = rec_buffer[2];
  }
  if (a_out_channel != NULL) {
    *a_out_channel = rec_buffer[3];
  }
  if (a_out_dir_ain_l != NULL) {
    *a_out_dir_ain_l = rec_buffer[4];
  }
  if (a_out_state_ain_m != NULL) {
    *a_out_state_ain_m = rec_buffer[5];
  }
  if (a_out_ain_h != NULL) {
    *a_out_ain_h = rec_buffer[6];
  }
}


void Ue9::HelperDioFeedback(uint8_t a_channel, uint8_t a_direction, uint8_t *a_state)
{
  uint8_t send_buffer[34];
  uint8_t rec_buffer[64];

  send_buffer[1] = (uint8_t) (0xF8); // Command byte
  send_buffer[2] = (uint8_t) (0x0E); // Number of data words
  send_buffer[3] = (uint8_t) (0x00); // Extended command number

  for (int32_t i = 6; i < 34; i++) {
    send_buffer[i] = 0;
  }

  uint8_t temp_dir = (a_direction < 1 ? 0 : 1);
  uint8_t temp_state = (*a_state < 1 ? 0 : 1);
  uint8_t temp_byte;

  if (a_channel <=  7) {
    temp_byte = static_cast<uint8_t>(pow(2.0, a_channel));
    send_buffer[6] = temp_byte;
    if (temp_dir) {
      send_buffer[7] = temp_byte;
    }
    if (temp_state) {
      send_buffer[8] = temp_byte;
    }
  } else if (a_channel <= 15) {
    temp_byte = static_cast<uint8_t>(pow(2, a_channel - 8));
    send_buffer[9] = temp_byte;
    if (temp_dir) {
      send_buffer[10] = temp_byte;
    }
    if (temp_state) {
      send_buffer[11] = temp_byte;
    }
  } else if (a_channel <= 19) {
    temp_byte = static_cast<uint8_t>(pow(2.0, a_channel - 16));
    send_buffer[12] = temp_byte;
    if (temp_dir) {
      send_buffer[13] = temp_byte * 16;
    }
    if (temp_state) {
      send_buffer[13] += temp_byte;
    }
  } else if (a_channel <= 22) {
    temp_byte = static_cast<uint8_t>(pow(2, a_channel - 20));
    send_buffer[14] = temp_byte;
    if (temp_dir) {
      send_buffer[15] = temp_byte * 16;
    }
    if (temp_state) {
      send_buffer[15] += temp_byte;
    }
  } else {
    throw std::runtime_error("Invalid channel.");
  }

  GetExtendedChecksum(send_buffer, 34);

  int32_t res = send(m_socket_command, send_buffer, 34, 0);;
  if (res < 34) {
    throw std::runtime_error("Failed to send.");
  }

  res = recv(m_socket_command, rec_buffer, 64, 0);
  if (res < 64) {
    throw std::runtime_error("Failed to receive.");
  }

  uint16_t checksum_total = GetExtendedChecksum16(rec_buffer, 64);
  if ((uint8_t) ((checksum_total / 256) & 0xff) != rec_buffer[5]) {
    throw std::runtime_error("Read buffer, bad checksum 16 (MSB).");
  }

  if ((uint8_t) (checksum_total & 0xff) != rec_buffer[4]) {
    throw std::runtime_error("Read buffer, bad checksum 16 (LSB).");
  }

  if (GetExtendedChecksum8(rec_buffer) != rec_buffer[0]) {
    throw std::runtime_error("Read buffer, bad checksum 8.");
  }

  if (rec_buffer[1] != (uint8_t)(0xF8) || rec_buffer[2] != (uint8_t)(0x1D) ||
      rec_buffer[3] != (uint8_t)(0x00)) {
    throw std::runtime_error("Read buffer, invalid command.");
  }

  if (a_channel <=  7) {
    *a_state = (rec_buffer[7] & temp_byte) ? 1 : 0;
  } else if (a_channel <= 15) {
    *a_state = (rec_buffer[9]&temp_byte) ? 1 : 0;
  } else if (a_channel <= 19) {
    *a_state = (rec_buffer[10]&temp_byte) ? 1 : 0;
  } else if (a_channel <= 22) {
    *a_state = (rec_buffer[11]&temp_byte) ? 1 : 0;
  }
}


void Ue9::HelperTimerCounter(uint8_t a_timer_clock_divisor, uint8_t a_enable_mask,
    uint8_t a_timer_clock_base, uint8_t a_update_reset, uint8_t *a_timer_mode,
    uint16_t *a_timer_value, uint8_t *a_counter_mode, uint32_t *a_out_timer,
    uint32_t *a_out_counter)
{
  uint8_t send_buffer[30];
  uint8_t rec_buffer[40];

  send_buffer[1] = (uint8_t) (0xF8); // Command byte
  send_buffer[2] = (uint8_t) (0x0C); // Number of data words
  send_buffer[3] = (uint8_t) (0x18); // Extended command number

  send_buffer[6] = a_timer_clock_divisor;
  send_buffer[7] = a_enable_mask;
  send_buffer[8] = a_timer_clock_base;

  send_buffer[9] = a_update_reset;

  for (int32_t i = 0; i < 6; i++) {
    send_buffer[10 + i * 3] = a_timer_mode[i];
    send_buffer[11 + i * 3] = (uint8_t) (a_timer_value[i] & 0x00FF);
    send_buffer[12 + i * 3] = (uint8_t) ((a_timer_value[i] & 0xFF00) / 256);
  }

  for (int32_t i = 0; i < 2; i++) {
    send_buffer[28 + i] = a_counter_mode[i];
  }

  GetExtendedChecksum(send_buffer, 30);

  int32_t res = send(m_socket_command, send_buffer, 30, 0);;
  if (res < 30) {
    throw std::runtime_error("Failed to send.");
  }

  res = recv(m_socket_command, rec_buffer, 40, 0);
  if (res < 40) {
    throw std::runtime_error("Failed to receive.");
  }

  uint16_t checksum_total = GetExtendedChecksum16(rec_buffer, 40);
  if ((uint8_t) ((checksum_total / 256) & 0xff) != rec_buffer[5]) {
    throw std::runtime_error("Read buffer, bad checksum 16 (MSB).");
  }

  if ((uint8_t) (checksum_total & 0xff) != rec_buffer[4]) {
    throw std::runtime_error("Read buffer, bad checksum 16 (LSB).");
  }

  if (GetExtendedChecksum8(rec_buffer) != rec_buffer[0]) {
    throw std::runtime_error("Read buffer, bad checksum 8.");
  }

  if (rec_buffer[1] != (uint8_t)(0xF8) || rec_buffer[2] != (uint8_t)(0x11) ||
      rec_buffer[3] != (uint8_t)(0x18)) {
    throw std::runtime_error("Read buffer, invalid command.");
  }

  if (a_out_timer != NULL) {
    for (int32_t i = 0; i < 6; i++) {
      a_out_timer[i] = 0;
      for (int32_t j = 0; j < 4; j++) {
        a_out_timer[i] += rec_buffer[8 + j + i * 4] * static_cast<uint32_t>(pow(2.0, 8.0 * j));
      }
    }
  }

  if (a_out_counter != NULL) {
    for (int32_t i = 0; i < 2; i++) {
      a_out_counter[i] = 0;
      for (int32_t j = 0; j < 4; j++) {
        a_out_counter[i] += rec_buffer[32 + j + i * 4] * static_cast<uint32_t>(pow(2, 8 * j));
      }
    }
  }
}
