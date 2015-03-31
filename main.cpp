//
//  main.cpp
//
//  Created by siwiak on 3/30/15.
//  Copyright (c) 2015 siwiak. All rights reserved.

// This code outputs EMG, Orientation, Accelerometer, and Gyroscope data. EMG streaming is only supported for one Myo at a time.
// usage: selct-myo

// OSC code from oscpack: http://www.rossbencina.com/code/oscpack

// Gesture Code from hello-myo, emg-data-sample, multiple-myos:
// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.

#define _USE_MATH_DEFINES
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <myo/myo.hpp>

// stop oscpack sprintf warnings
#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

// add OSCpack
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

#define OUTPUT_BUFFER_SIZE 1024

UdpTransmitSocket* transmitSocket;


class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : emgSamples(), roll_w(0), pitch_w(0), yaw_w(0)
    {
    }
    
    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo! Let's clean up some leftover state.
        emgSamples.fill(0);
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        onArm = false;
        
    }
    
    // onEmgData() is called whenever a paired Myo provides new EMG data and EMG streaming is enabled.
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
    {
        for (int i = 0; i < 8; i++) {
            emgSamples[i] = emg[i];
            
        }
        
        // OSC code for onEmgData
        osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
        p << osc::BeginMessage("/myo/emg")
        << emgSamples[0] << emgSamples[1] << emgSamples[2] << emgSamples[3] << emgSamples[4] << emgSamples[5] << emgSamples[6] << emgSamples[7] << osc::EndMessage;
        transmitSocket->Send(p.Data(), p.Size());
    }
    
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;
        
        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                          1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        
        // Convert the floating point angles in radians to a scale from 0 to 18.
        roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
        
        // OSC code for onOrientationData
        osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
        p << osc::BeginMessage("/myo/orient")
        << roll_w << pitch_w << yaw_w << osc::EndMessage;
        transmitSocket->Send(p.Data(), p.Size());
        
    }
    
    // onAccelerometerData is called whenever the Myo device provides new x-y-z data, which is represented as units of g's.
    void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& accel)
    {
        a_x = accel.x();
        a_y = accel.y();
        a_z = accel.z();
        
        // OSC code for onAccelerometerData
        osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
        p << osc::BeginMessage("/myo/accel")
        << a_x << a_y << a_z << osc::EndMessage;
        transmitSocket->Send(p.Data(), p.Size());
    }
    
    // onGyroscopeData is called whenever the Myo device provides new gyro data, which is represented as units of degrees/second.
    void onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float>& gyro)
    {
        g_x = gyro.x();
        g_y = gyro.y();
        g_z = gyro.z();
        
        // OSC code for onGyroscopeData
        osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
        p << osc::BeginMessage("/myo/gyro")
        << g_x << g_y << g_z << osc::EndMessage;
        transmitSocket->Send(p.Data(), p.Size());
    }
    
    // onArmRecognized() is called whenever Myo has recognized a setup gesture after someone has put it on his or her
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmRecognized(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection)
    {
        onArm = true;
        whichArm = arm;
        
        // OSC code for onArmRecognized
        osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
        p << osc::BeginMessage("/myo/onarm")
        << (whichArm == myo::armLeft ? "L" : "R") << osc::EndMessage;
        transmitSocket->Send(p.Data(), p.Size());
    }
    
    // onArmLost() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmLost(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
        
        // OSC code for onArmLost
        osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
        p << osc::BeginMessage("/myo/onarmlost")
        << osc::EndMessage;
        transmitSocket->Send(p.Data(), p.Size());
    }
    
    
    
    // We define this function to print the current values that were updated by the on...() functions above.
    void print()
    {
        // Clear the current line
        std::cout << '\r';
        
        // Print out the orientation. Orientation data is always available, even if no arm is currently recognized.
        std::cout   << '[' << std::string(roll_w, '*') << std::string(18 - roll_w, ' ') << ']'
        << '[' << std::string(pitch_w, '*') << std::string(18 - pitch_w, ' ') << ']'
        << '[' << std::string(yaw_w, '*') << std::string(18 - yaw_w, ' ') << ']';
        
        if (onArm) {
            
            // Print out the EMG data.
            for (size_t i = 0; i < emgSamples.size(); i++) {
                std::ostringstream oss;
                oss << static_cast<int>(emgSamples[i]);
                std::string emgString = oss.str();
                
                std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
            }//
            std::cout << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']' ;
        } else {
            // Print out a placeholder for the arm and pose when Myo doesn't currently know which arm it's on.
            std::cout << "[?]" << '[' << std::string(14, ' ') << ']';
        }
        
        std::cout << std::flush;
    }
    
    // The values of this array is set by functions above.
    bool onArm;
    myo::Arm whichArm;
    
    int roll_w, pitch_w, yaw_w;
    float w, x, y, z, roll, pitch, yaw, a_x, a_y, a_z, g_x, g_y, g_z;
    std::array<int8_t, 8> emgSamples;
    char buffer[OUTPUT_BUFFER_SIZE];
    
};

int main(int argc, char** argv)
{
    // We catch any exceptions that might occur below -- see the catch statement for more details.
    try {
        
        if (argc != 3 && argc != 2 && argc != 1)
        {
            std::cout << "\nusage: " << argv[0] << " [IP address] <port>\n\n" <<
            "selct-myo sends OSC output over UDP from the input of a Thalmic Myo armband.\n" <<
            "IP address defaults to 127.0.0.1/localhost\n\n" <<
            "by Diana Siwiak";
            exit(0);
        }
        
        if (argc == 1)
        {
            int port = 7777;
            std::cout << "Sending Myo OSC to 127.0.0.1:7777\n";
            transmitSocket = new UdpTransmitSocket(IpEndpointName("127.0.0.1", port));
        }
        else if (argc == 2)
        {
            std::cout << "Sending Myo OSC to 127.0.0.1:" << argv[1] << "\n";
            transmitSocket = new UdpTransmitSocket(IpEndpointName("127.0.0.1", atoi(argv[1])));
        }
        else if (argc == 3)
        {
            std::cout << "Sending Myo OSC to " << argv[1] << ":" << argv[2] << "\n";
            transmitSocket = new UdpTransmitSocket(IpEndpointName(argv[1], atoi(argv[2])));
        }
        
        // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
        // publishing your application. The Hub provides access to one or more Myos.
        myo::Hub hub("com.djsdjs.selct-myo");
        
        std::cout << "Attempting to find a Myo..." << std::endl;
        
        // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
        // immediately.
        // waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
        // if that fails, the function will return a null pointer.
        myo::Myo* myo = hub.waitForMyo(10000);
        
        // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
        if (!myo) {
            throw std::runtime_error("Unable to find a Myo!");
        }
        
        // We've found a Myo.
        std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
        
        // Next we enable EMG streaming on the found Myo.
        myo->setStreamEmg(myo::Myo::streamEmgEnabled);
        
        // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
        DataCollector collector;
        
        // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
        // Hub::run() to send events to all registered device listeners.
        hub.addListener(&collector);
        
        // Finally we enter our main loop.
        while (1) {
            // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
            // In this case, we wish to update our display 50 times a second, so we run for 1000/20 milliseconds.
            hub.run(1000/20);
            // After processing events, we call the print() member function we defined above to print out the values we've
            // obtained from any events that have occurred.
            collector.print();
        }
        
        // If a standard exception occurred, we print out its message and exit.
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
