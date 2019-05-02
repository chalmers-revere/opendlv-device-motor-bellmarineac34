/*
 * Copyright (C) 2019 Ola Benderius
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

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>

#ifdef __linux__
#include <linux/if.h>
#include <linux/can.h>
#endif

#include <unistd.h>

#include <cstring>

#include <iostream>
#include <string>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid")) {
    std::cerr << argv[0] 
      << " interfaces to the Bellmarine AC-34 motor controller using CAN." 
      << std::endl;
    std::cerr << "Usage:   " << argv[0] << " " 
      << "--can=<name of the CAN interface> "
      << "--cid=<OpenDLV session> [--sender-stamp-offset] [--verbose]" 
      << std::endl;
    std::cerr << "Example: " << argv[0] << " --can=can0 --cid=111 --verbose" 
      << std::endl;
    retCode = 1;
  } else {
    bool const verbose{commandlineArguments.count("verbose") != 0};
    uint16_t const cid = std::stoi(commandlineArguments["cid"]);
    std::string const canDev{(commandlineArguments["can"].size() != 0) 
      ? commandlineArguments["can"] : "can0"};
    int32_t const senderStampOffset{
      (commandlineArguments["sender-stamp-offset"].size() != 0) 
        ? std::stoi(commandlineArguments["sender-stamp-offset"]) : 0};

    float pedalPosition = 0.0f;
    std::mutex pedalPositionMutex;

    auto onPedalPositionRequest{[&pedalPosition, &pedalPositionMutex, 
      &senderStampOffset, &verbose](cluon::data::Envelope &&envelope)
      {
        int32_t senderStamp = envelope.senderStamp();
        if (senderStamp == senderStampOffset) {
          std::lock_guard<std::mutex> lock(pedalPositionMutex);
          auto msg = 
            cluon::extractMessage<opendlv::proxy::PedalPositionRequest>(
                std::move(envelope));
          if (msg.position() > -1.0f && msg.position() < 1.0f) {
            pedalPosition = msg.position();
          } else {
            std::cerr << "Pedal position out of range [-1.0, 1.0]." 
              << std::endl;
          }
          if (verbose) {
            std::cout << "Pedal position request set to " << pedalPosition 
               << std::endl;
          }
        }
      }};

    cluon::OD4Session od4{cid};
    od4.dataTrigger(opendlv::proxy::PedalPositionRequest::ID(),
        onPedalPositionRequest);

#ifdef __linux__
    struct sockaddr_can address;
#endif
    int32_t socketCan;

    std::cerr << "Opening " << canDev << "... ";
#ifdef __linux__
    socketCan = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketCan < 0) {
      std::cerr << "failed." << std::endl;
      std::cerr << "Error while creating socket: " << strerror(errno) 
        << std::endl;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, canDev.c_str());
    if (0 != ioctl(socketCan, SIOCGIFINDEX, &ifr)) {
      std::cerr << "failed." << std::endl;
      std::cerr << "Error while getting index for " << canDev << ": " 
        << strerror(errno) << std::endl;
      return retCode;
    }

    memset(&address, 0, sizeof(address));
    address.can_family = AF_CAN;
    address.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketCan, reinterpret_cast<struct sockaddr *>(&address),
          sizeof(address)) < 0) {
      std::cerr << "failed." << std::endl;
      std::cerr << "Error while binding socket: " << strerror(errno)
        << std::endl;
      return retCode;
    }
    std::cerr << "done." << std::endl;
#else
    std::cerr << "failed (SocketCAN not available on this platform). " 
      << std::endl;
    return retCode;
#endif

    float const rpmToAngularVelocityFactor = 3.14159f / 30.0f;

    while (od4.isRunning() && socketCan > -1) {
#ifdef __linux__
      {
        struct timeval to;
        to.tv_sec = 0;
        to.tv_usec = 20000;

        setsockopt(socketCan, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));

        struct can_frame frame;
        int32_t nbytes = read(socketCan, &frame, sizeof(struct can_frame));
        if ((nbytes > 0) && (nbytes == sizeof(struct can_frame))) {

          struct timeval socketTimeStamp;
          if (0 != ioctl(socketCan, SIOCGSTAMP, &socketTimeStamp)) {
            cluon::data::TimeStamp now{cluon::time::now()};
            socketTimeStamp.tv_sec = now.seconds();
            socketTimeStamp.tv_usec = now.microseconds();
          }

          cluon::data::TimeStamp sampleTimeStamp;
          sampleTimeStamp.seconds(socketTimeStamp.tv_sec);
          sampleTimeStamp.microseconds(socketTimeStamp.tv_usec);


          if (frame.can_dlc == 8) {
            int16_t motorRpm = (frame.data[1] << 8) + frame.data[0];
            uint16_t keyswitchVoltageUint = (frame.data[3] << 8) 
              + frame.data[2];
            uint16_t batteryCurrentUint = (frame.data[5] << 8) 
              + frame.data[4];
            uint16_t rmsCurrentUint = (frame.data[7] << 8) + frame.data[6];

            float motorAngularVelocity = rpmToAngularVelocityFactor 
              * motorRpm;
            float keyswitchVoltage = static_cast<float>(keyswitchVoltageUint) 
              / 100.0f;
            float batteryCurrent = static_cast<float>(batteryCurrentUint) 
              / 10.0f;
            float rmsCurrent = static_cast<float>(rmsCurrentUint) / 10.0f;

            cluon::data::TimeStamp ts = cluon::time::now();

            opendlv::proxy::WheelSpeedReading wheelSpeedReading;
            wheelSpeedReading.wheelSpeed(motorAngularVelocity);
            od4.send(wheelSpeedReading, ts, senderStampOffset);
            
            opendlv::proxy::VoltageReading voltageReading;
            voltageReading.voltage(keyswitchVoltage);
            od4.send(voltageReading, ts, senderStampOffset);
            
            opendlv::proxy::ElectricCurrentReading rmsCurrentReading;
            rmsCurrentReading.electricCurrent(rmsCurrent);
            od4.send(rmsCurrentReading, ts, senderStampOffset);

            opendlv::proxy::ElectricCurrentReading batteryCurrentReading;
            batteryCurrentReading.electricCurrent(batteryCurrent);
            od4.send(batteryCurrentReading, ts, senderStampOffset + 1);

            if (verbose) {
              std::cout << "Motor RPM: " << motorRpm << " Keyswitch voltage: "
                << keyswitchVoltage << " Battery current: " << batteryCurrent
                << " RMS current: " << rmsCurrent << std::endl;
            }
          }
        }
      }

      {
        std::lock_guard<std::mutex> lock(pedalPositionMutex);
        int16_t motorRpmInt = static_cast<int16_t>(22000.0f * pedalPosition);
        int8_t buffer[] = {static_cast<int8_t>(motorRpmInt), 
          static_cast<int8_t>(motorRpmInt >> 8), 0, 0, 0, 0, 0, 0};

        struct can_frame frame;
        frame.can_id = 0x200;
        frame.can_dlc = 8;
        memcpy(frame.data, buffer, frame.can_dlc);
        int32_t n = ::write(socketCan, &frame, 
            sizeof(struct can_frame));

        if (!(0 < n)) {
          std::clog << "[SocketCANDevice] Write error (" << errno << "): '" 
            << strerror(errno) << "'" << std::endl;
        }
      }
#endif
    }

    std::clog << "Closing " << canDev << "... ";
    if (socketCan > -1) {
      close(socketCan);
    }
    std::clog << "done." << std::endl;

    retCode = 0;
  }
  return retCode;
}
