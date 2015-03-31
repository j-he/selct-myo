# [selct-myo](https://ecs.victoria.ac.nz/Groups/SELCT/)

OSC bridge for Thalmic Myo gesture control armband for Mac OS X 10.9+.

By [Diana Siwiak](http://dianasiwiak.com)

selct-myo is a C++ application designed to take emg/mmg, accelerometer, gyroscope and magnetometer data from the [Thalmic Labs Myo](https://www.thalmic.com/en/myo/) armband and output it over OSC.

The binary for OS X is included in the Build directory (/Build/Products/Debug/selct-myo.osx.bin).

------

## Downloads recent as of 31 March 2015

Download and install latest [Myo Connect 0.9.1] (https://developer.thalmic.com/downloads) and [Myo Firmware 1.2.955] (https://developer.thalmic.com/downloads)

Download [selct-myo] (http://github.com/diana-js/selct-myo/)

------

## Usage

$ cd selct-myo

$ ./selct-myo [IP address] [OSC port]

This will output the following OSC data to localhost over specified (or default: 7777) port

```
/myo/emg emg[0]_int8 emg[1]_int8 emg[2]_int8 emg[3]_int8 emg[4]_int8 emg[5]_int8 emg[6]_int8 emg[7]_int8

/myo/accel X_vector3 Y_vector3 Z_vector3

/myo/gyro X_vector3 Y_vector3 Z_vector3

/myo/orientation X_quaternion Y_quaternion Z_quaternion W_quaternion roll pitch yaw

/myo/onarm L or R

/myo/onarmlost {timestamp}
```

------