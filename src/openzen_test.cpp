//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

// request always the C++14 interface of OpenZen, even if the compiler
// can support he C++17 interface
#define OPENZEN_CXX14
#include "OpenZen.h"

#include <iostream>

using namespace zen;

/**
 * This example demonstrates the C++ interface of the OpenZen library. This interface
 * does also work for C++ compilers which do not support the C++17 standard yet.
 * 
 * Check our docs for more https://lpresearch.bitbucket.io/openzen/latest/getting_started.html
 */
int main(int argc, char* argv[])
{
    // enable resonable log output for OpenZen
    ZenSetLogLevel(ZenLogLevel_Info);

    // create OpenZen Clien
    auto clientPair = make_client();
    auto& clientError = clientPair.first;
    auto& client = clientPair.second;

    if (clientError) {
        std::cout << "Cannot create OpenZen client" << std::endl;
        return clientError;
    }

    // connect to sensor on IO System by the sensor name
    // more on https://lpresearch.bitbucket.io/openzen/latest/io_systems.html
    // auto sensorPair = client.obtainSensorByName("WindowsDevice", "\\\\.\\COM7", 921600);
    auto sensorPair = client.obtainSensorByName("Bluetooth", "00:04:3E:6F:38:05");
    // auto sensorPair = client.obtainSensorByName("SiUsb", "lpmscu2000573", 921600);
    auto& obtainError = sensorPair.first;
    auto& sensor = sensorPair.second;
    if (obtainError)
    {
        std::cout << "Cannot connect to sensor" << std::endl;
        client.close();
        return obtainError;
    }

    // check that the sensor has an IMU component
    auto imuPair = sensor.getAnyComponentOfType(g_zenSensorType_Imu);
    auto& hasImu = imuPair.first;
    auto imu = imuPair.second;

    if (!hasImu)
    {
        std::cout << "Connected sensor has no IMU" << std::endl;
        client.close();
        return ZenError_WrongSensorType;
    }

    // set and get current streaming frequency
    auto error = imu.setInt32Property(ZenImuProperty_SamplingRate, 100);
    if (error) {
        std::cout << "Error setting streaming frequency" << std::endl;
        client.close();
        return error;
    }

    auto freqPair = imu.getInt32Property(ZenImuProperty_SamplingRate);
    if (freqPair.first) {
        std::cout << "Error fetching streaming frequency" << std::endl;
        client.close();
        return freqPair.first;
    }
    std::cout << "Streaming frequency: " << freqPair.second << std::endl;

    // toggle on/off of a particular data output (linAcc is not ON by default)
    error = imu.setBoolProperty(ZenImuProperty_OutputLinearAcc, true);
    if (error) {
        std::cout << "Error toggling ON linear acc data output" << std::endl;
        client.close();
        return error;
    }

    // readout up to 200 samples from the IMU
    // note that there are 2 gyro fields in the IMU data structure (ZenImuData struct in include/ZenTypes.h)
    // please refer to your sensor's manual for correct retrieval of gyro data
    for (int i = 0; i <100; i++) {
        auto event = client.waitForNextEvent();
        if (event.second.component.handle == imu.component().handle) {
            std::cout << "> Lin Acceleration: \t x = " << event.second.data.imuData.linAcc[0]
                << "\t y = " << event.second.data.imuData.linAcc[1]
                << "\t z = " << event.second.data.imuData.linAcc[2] << std::endl;

            // depending on sensor, gyro data is outputted to g1, g2 or both
            // read more on https://lpresearch.bitbucket.io/openzen/latest/getting_started.html#id1
            std::cout << "> Raw Gyro 1: \t\t x = " << event.second.data.imuData.g1Raw[0]
                << "\t y = " << event.second.data.imuData.g1Raw[1]
                << "\t z = " << event.second.data.imuData.g1Raw[2] << std::endl;
            
            std::cout << "> RPY: \t\t x = " << event.second.data.imuData.r[0]
                << "\t y = " << event.second.data.imuData.r[1]
                << "\t z = " << event.second.data.imuData.r[2] << std::endl;
                
            std::cout << "> Quat: \t\t w = " << event.second.data.imuData.q[0]
                << "\t x = " << event.second.data.imuData.q[1]
                << "\t y = " << event.second.data.imuData.q[2]
                << "\t z = " << event.second.data.imuData.q[3] << std::endl;
        }
    }

    client.close();
    std::cout << "Sensor connection closed" << std::endl;
    return 0;
}
