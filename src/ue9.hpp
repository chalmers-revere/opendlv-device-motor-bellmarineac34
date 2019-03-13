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

#ifndef LABJACK_UE9_HPP
#define LABJACK_UE9_HPP

#include <string>
#include <vector>

class Ue9 {
 public:
  struct CalibrationInfo {
    uint8_t prodID;
    double unipolarSlope[4];
    double unipolarOffset[4];
    double bipolarSlope;
    double bipolarOffset;
    double DACSlope[2];
    double DACOffset[2];
    double tempSlope;
    double tempSlopeLow;
    double calTemp;
    double Vref;
    double VrefDiv2;
    double VsSlope;
    double hiResUnipolarSlope;
    double hiResUnipolarOffset;
    double hiResBipolarSlope;
    double hiResBipolarOffset;
  };

  Ue9(std::string const &, int32_t, int32_t);
  virtual ~Ue9();
  void StartStream(int32_t, int32_t, int32_t);
  void StopStream();
  std::vector<std::vector<double> > ParseStream(int32_t);
  void ReadAin(long, double *, long, long, long, long);
  void WriteDo(long, long);
  void WriteDac(long, double);

 private:
  Ue9(Ue9 const &);
  Ue9 &operator=(Ue9 const &);

  int Connect(std::string const &, int32_t);
  void Disconnect(int32_t);
  void FlushStream();
  uint8_t GetChecksum(uint8_t *, int32_t);
  void GetExtendedChecksum(uint8_t *, int32_t);
  uint8_t GetExtendedChecksum8(uint8_t *);
  uint16_t GetExtendedChecksum16(uint8_t *, int32_t);
  void InitStream(int32_t, int32_t, int32_t);
  void ReadCalibrationInfo();
  double ConvertFPuint8ArrayToFPDouble(uint8_t *, int32_t);
  void ConvertBinaryToAnalogVoltage(uint8_t, uint8_t, uint16_t, double *);
  void ConvertAnalogToBinaryVoltage(int, double, uint16_t *);
  void ConvertBinaryToAnalogTemperature(int, uint16_t, double *);
  void ReadDin(long, long *);
  void InitTcConfig(long *, long *, long, long, long *, double *);
  void ReadTcValues(long *, long *, double *, double *);
  void HelperSingleIo(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t *, uint8_t *,
      uint8_t *, uint8_t *, uint8_t *);
  void HelperDioFeedback(uint8_t, uint8_t, uint8_t *);
  void HelperTimerCounter(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t *, uint16_t *,
      uint8_t *, uint32_t *, uint32_t *);

  CalibrationInfo *m_calibration_info;
  std::string m_address;
  bool m_stream_running;
  int32_t m_port_command;
  int32_t m_port_stream;
  int32_t m_socket_command;
  int32_t m_socket_stream;
  int32_t m_stream_channel_count;
  int32_t m_stream_resolution;
  long m_stream_packet_count;
};

#endif
