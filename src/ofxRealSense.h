/*==============================================================================

    Copyright (c) 2018, Brooklyn Research

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

    ----------------------------------------------------------------------------

    This project uses libfreenect, copyrighted by the Open Kinect Project using
    the Apache License v2. See the file "APACHE20" in libs/libfreenect.

    See http://www.openkinect.org & https://github.com/OpenKinect/libfreenect
    for documentation

==============================================================================*/
#pragma once

#include <vector>
#include <cmath>
#include <string>
#include <map>
#include "ofMain.h"
#include "rs.h"
#include "rs.hpp"
#include "rsutil.h"

#if defined(_MSC_VER) || defined(_WIN32) || defined(WIN32) || defined(__MINGW32__)
    // do windows stuff
#else
    // mac and linux need this
    #include <libusb.h>
#endif

#include "ofxBase3DVideo.h"
#include "ofxRealSenseContext.h"
#include "Filter.h"

class ofxRealSenseContext;

class ofxRealSense2 : public ofxBase3DVideo, protected ofThread {
    public:
        ofxRealSense2();
        virtual ~ofxRealSense2();

        bool init(bool infrared=false, bool video=true, bool texture=true);

        bool initDepth(int width, int height);
        bool initColor(int width, int height);
        bool initInfrared(int width, int height);

        void initFilters();

        void toggleFilter(string name, bool value);

        void setFilterOption(const string name, int optionEnum, float value);

        void clear();

        void setRegistration(bool bUseRegistration=false);

        bool open(int deviceIndex=-1);

        bool open(string serial);

        bool openFromFile(string filename);

        void close();

        /// is the connection currently open?
        bool isConnected() const;
        bool isInitialized() const;

        /// is the current frame new?
        bool isFrameNew() const;
        bool isFrameNewVideo() const;
        bool isFrameNewDepth() const;

        bool setPixelFormat(ofPixelFormat pixelFormat);
        ofPixelFormat getPixelFormat() const;

        static std::string getOptionName(rs2_option opt);

        static void printAllDevices(const rs2::context& ctx);

        static std::string getDeviceName(const rs2::device& dev);

        static void printDeviceInformation(const rs2::device& dev);

        static std::string getSensorName(const rs2::sensor& sensor);

        static void printAllSensorOptions(const rs2::device& dev);

        static void printSensorOptions(const rs2::sensor& sensor);

        static float getDepthScale(rs2::device device);

        static void setHighDensityPreset(const rs2::pipeline_profile& profile);
        void setHighAccuracyPreset();

        static void removeBackground(ofImage& dstFrame, const rs2::depth_frame& srcFrame,
                float depthScale, float clippingDistance);

        void update();

        /// get the calulated distance for a depth point
        float getDistanceAt(int x, int y) const;
        float getDistanceAt(const ofPoint & p) const;

        /// calculates the coordinate in the world for the depth point (perspective calculation)
        ///
        /// center of image is (0.0)
        ofVec3f getWorldCoordinateAt(int cx, int cy) const;
        ofVec3f getWorldCoordinateAt(float cx, float cy, float wz) const;

        ofColor getColorAt(int x, int y) const;
        ofColor getColorAt(const ofPoint & p) const;

        ofPixels & getPixels();
        const ofPixels & getPixels() const;

        ofPixels & getDepthPixels();
//        const ofPixels & getDepthPixels() const;           ///< grayscale values

        ofPixels & getIRPixels();
        const ofPixels & getIRPixels() const;           ///< grayscale values

        /// get the pixels of the most recent depth frame
        const ofPixels & getDepthPixels() const;           ///< grayscale values
        ofShortPixels & getRawDepthPixels();    ///< raw 11 bit values
        const ofShortPixels & getRawDepthPixels() const;    ///< raw 11 bit values

        /// get the distance in millimeters to a given point as a float array
        ofFloatPixels & getDistancePixels();
        const ofFloatPixels & getDistancePixels() const;

        /// get the video (ir or rgb) texture
        ofTexture& getTexture();
        const ofTexture& getTexture() const;

        // get the grayscale depth value
        ofTexture& getDepthTexture();
        const ofTexture& getDepthTexture() const;

        // set the near value of the pixels in the grayscale depth image to white

        // bEnabled = true:  pixels closer to the camera are brighter (default)
        // bEnabled = false: pixels closer to the camera are darker
        void enableDepthNearValueWhite(bool bEnabled=true);
        bool isDepthNearValueWhite() const;

        // set the clipping planes for the depth calculations in millimeters

        // these are used for the depth value (12bit) -> grayscale (1 byte) conversion
        // ie setting a short range will give you greater sensitivity from 0-255

        // default is 50cm - 4m
        // note: you won't get any data < 50cm and distances > 4m start to get noisy
        //void setDepthClipping(float nearClip=500, float farClip=4000);

        //Get values needed to set up clipping controls
        float getClippingOptionMin() const;
        float getClippingOptionMax() const;
        float getClippingOptionStep() const;

        //Get the current values
        float getNearClipping() const;
        float getFarClipping() const;

        //Setting individually since we have separate controls
        void setNearClipping(float value);
        void setFarClipping(float value);

        std::vector<Filter> filters;

        //rs2::frame_queue rawDepthQueue;
        rs2::frame_queue filteredDepthQueue;
        rs2::frame_queue videoQueue;
        rs2::frame_queue infraredQueue;

        /// enable/disable frame loading into textures on update()
        void setUseTexture(bool bUse);
        bool isUsingTexture() const;

        /// draw the color video
        void draw(float x, float y, float w, float h) const;
        void draw(float x, float y) const;
        void draw(const ofPoint& point) const;
        void draw(const ofRectangle& rect) const;

        /// draw the color depth map
        void drawDepth(float x, float y, float w, float h) const;
        void drawDepth(float x, float y) const;
        void drawDepth(const ofPoint& point) const;
        void drawDepth(const ofRectangle& rect) const;

        // draw the infrared video
        void drawIR(float x, float y, float w, float h) const;
        void drawIR(float x, float y) const;
        void drawIR(const ofPoint& point) const;
        void drawIR(const ofRectangle& rect) const;

        // pointcloud work
        void generatePointCloud();
        void drawPointCloud(float width, float height, rs2::points& points);

        /// get the device id
        /// returns -1 if not connected
        int getDeviceId() const;

        string getSerial() const;

        int depthWidth;
        int depthHeight;
        float getDepthHeight() const;
        float getDepthWidth() const;
        int colorWidth;
        int colorHeight;
        float getColorWidth() const;
        float getColorHeight() const;

        /// \section Static global kinect context functions

        /// print the device list
        static void listDevices();

        static vector<string> getAvailableSerials();

        /// get the total number of devices
        static int numTotalDevices();

        /// get the number of available devices (not connected)
        static int numAvailableDevices();

        /// get the number of currently connected devices
        static int numConnectedDevices();

        /// is a device already connected?
        static bool isDeviceConnected(int id);
        static bool isDeviceConnected(string serial);

        /// get the id of the next available device,
        /// returns -1 if nothing found
        static int nextAvailableId();

        /// get the serial number of the next available device,
        /// returns an empty string "" if nothing found
        static string nextAvailableSerial();

        /// set the time to wait when not getting data before attempting to re-open device.
        static void setReconnectWaitTime(float waitTime);

    protected:
        int deviceId;	///< -1 when not connected
        string serial;	///< unique serial number, "" when not connected

        bool bUseTexture;
        ofTexture depthTex; ///< the depth texture
        ofTexture videoTex; ///< the RGB texture
        ofTexture infraredTex;
        bool bGrabberInited;

        bool bUseDepth;
        bool bUseInfrared;


        ofPixels videoPixels;
        ofPixels depthPixels;
        ofPixels infraredPixels;
        ofShortPixels depthPixelsRaw;
        ofFloatPixels distancePixels;

        float timeSinceOpen;
        static float reconnectWaitTime;
        int lastDeviceIndex;
        bool bGotDataDepth;
        bool bGotDataVideo;
        bool bFirstUpdate;
        int tryCount;

//    private:

        friend class ofxRealSenseContext;
        friend class PostProcessingThread;

        // global statics shared between realsense intances
        static ofxRealSenseContext realSenseContext;

        rs2::device* realSenseDevice;

        ofShortPixels depthPixelsRawIntra;	///< depth back
        ofPixels videoPixelsIntra;			///< rgb back
        ofShortPixels depthPixelsRawBack;	///< depth back
        ofPixels videoPixelsBack;			///< rgb back

        vector<unsigned char> depthLookupTable;
        void updateDepthLookupTable();
        void updateDepthPixels();

        bool bIsFrameNewVideo, bIsFrameNewDepth;
      	bool bNeedsUpdateVideo, bNeedsUpdateDepth;
      	bool bGrabVideo;
      	bool bUseRegistration;
      	bool bNearWhite;

      	float nearClipping, farClipping;

      	bool bIsVideoInfrared;  ///< is the video image infrared or RGB?
      	int videoBytesPerPixel; ///< how many bytes per pixel in the video image
      	ofPixelFormat pixelFormat;

        // obviously need a different analogue for this
        /// libfreenect callbacks
        static void grabDepthFrame(rs2::device* dev, void* depth, uint32_t timestamp);
        static void grabVideoFrame(rs2::device* dev, void* video, uint32_t timestamp);

        /// thread function
        void threadedFunction();

        rs2::colorizer color_map;
        rs2::config config;
        rs2::pipeline pipe;
        std::map<int, rs2::frame> frames_per_stream;
        rs2::pipeline_profile profile;
        rs2_intrinsics intr;

        rs2::frameset allset;
        rs2::frame depth;
        rs2::frame color;
        rs2::frame infrared;

        rs2::pointcloud pointCloud;
        rs2::points points;

        rs2::decimation_filter decimationFilter;
        rs2::spatial_filter spatialFilter;
        rs2::temporal_filter temporalFilter;
        rs2::hole_filling_filter holeFillingFilter;

        ofImage colorImage;
        ofImage depthImage;
        ofImage infraredImage;
};
