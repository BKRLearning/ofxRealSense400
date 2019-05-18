#pragma once

#include <string>
#include <vector>
#include <map>
#include "rs.h"
#include "rs.hpp"
#include "ofxRealSense.h"

class ofxRealSense2;

class ofxRealSenseContext {

 public:
    ofxRealSenseContext();
    ~ofxRealSenseContext();

    /// \section Main

    /// init the freenect context
    bool init();

    /// clear the freenect context
    /// closes all currently connected devices
    void clear();

    /// is the context inited?
    bool isInited();

    /// open a kinect device
    /// an id of -1 will open the first available
    bool open(ofxRealSense2& realSense, int id=-1);

    /// open a kinect device by it's unique serial number
    bool open(ofxRealSense2& realSense, string serial);

    // open a realsense device recording
    bool openFromFile(ofxRealSense2& realSense, string filename);

    /// close a kinect device
    void close(ofxRealSense2& realSense);

    /// closes all currently connected kinects
    void closeAll();

    /// \section Util

    /// (re)build the list of devices
    void buildDeviceList();

    /// print the device list
    void listDevices(bool verbose=false);

    // get a vector of all serial numbers
    vector<string> getAvailableSerials();

    /// get the total number of devices
    int numTotal();

    /// get the number of available devices (not connected)
    int numAvailable();

    /// get the number of currently connected devices
    int numConnected();

    /// get the realsense object from a device pointer
    /// returns NULL if not found
    ofxRealSense2* getRealSense(rs2::device* dev);

    /// get the realsense object from a serial number
    /// returns NULL if not found
    ofxRealSense2* getRealSense(string serial);

    /// get the deviceList index from an id
    /// returns -1 if not found
    int getDeviceIndex(int id);

    /// get the deviceList index from an id
    /// returns -1 if not found
    int getDeviceIndex(string serial);

    /// get the deviceList id from an index
    /// returns -1 if not found
    int getDeviceId(unsigned int index);

    /// get the deviceList id from a serial
    /// returns -1 if not found
    int getDeviceId(string serial);

    /// is a device with this id already connected?
    bool isConnected(int id);

    /// is a device with this serial already connected?
    bool isConnected(string serial);

    map<string, bool> getDeviceStatusMap(map<string, string>);

    /// get the id of the next available device,
    /// returns -1 if nothing found
    int nextAvailableId();

    /// get the serial number of the next available device,
    /// returns an empty string "" if nothing found
    string nextAvailableSerial();

    /// get the raw pointer
    rs2::context* getContext() {return realSenseContext;}

    // for auto-enumeration
    struct RealSensePair{
        string serial;	///< unique serial number
        int id;			///< freenect bus id
    };

  //private:

    std::vector<rs2::device> realSenseDevices; // may not be used, just as backup
    bool bInited;						///< has the context been initialized?
    rs2::context* realSenseContext;    ///< real sense context handle
    std::vector<RealSensePair> deviceList;	///< list of available devices, sorted by serial lexicographically
    std::map<int,ofxRealSense2*> realSenses;   ///< the connected kinects
//    rs2::error* e;
};
