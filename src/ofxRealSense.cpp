/*==============================================================================

    Copyright (c) 2018 Brooklyn Research

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


==============================================================================*/
#include <iostream>
#include <iomanip>
#include "ofxRealSense.h"
#include "ofMain.h"

// context static
ofxRealSenseContext ofxRealSense2::realSenseContext;
float ofxRealSense2::reconnectWaitTime = 3.0;


ofxRealSense2::ofxRealSense2() {
  ofLogVerbose("ofxRealSense2") << "creating ofxRealSense2";

  deviceId = -1;
  serial = "";

  bUseTexture = true;
  bGrabVideo = true;
  bUseDepth = true;

  // set defaults
  bGrabberInited = false;

  bNeedsUpdateVideo = false;
  bNeedsUpdateDepth = false;
  bIsFrameNewVideo = false;
  bIsFrameNewDepth = false;

  bIsVideoInfrared = false;
  videoBytesPerPixel = 3;

  realSenseDevice = NULL;

  lastDeviceIndex = -1;
  tryCount = 0;
  timeSinceOpen = 0;
  bGotDataVideo = false;
  bGotDataDepth = false;
  bFirstUpdate = true;

  bUseRegistration = false;
  bNearWhite = true;

  pixelFormat = OF_PIXELS_RGB;

  nearClipping = 0.0;
  farClipping = 6.0;

  depthWidth = 1280;
  depthHeight = 720;

  colorWidth = 1280;
  colorHeight = 720;
}

ofxRealSense2::~ofxRealSense2() {
  // close? clear?
  close();
}

bool ofxRealSense2::init(bool infrared, bool video, bool texture) {

    bIsVideoInfrared = infrared;
    bGrabVideo = video;
    bUseTexture = texture;

    // set configuration
    if (bGrabVideo) {
        config.enable_stream(RS2_STREAM_COLOR, colorWidth, colorHeight, RS2_FORMAT_RGB8, 30);
        colorImage.allocate(colorWidth, colorHeight, OF_IMAGE_COLOR);
        videoPixels.set(0);
        videoPixelsBack.set(0);
    }

    if (bUseDepth) {
        config.enable_stream(RS2_STREAM_DEPTH, depthWidth, depthHeight, RS2_FORMAT_Z16, 30);
        depthImage.allocate(depthWidth, depthHeight, OF_IMAGE_COLOR);
        depthPixels.allocate(depthWidth, depthHeight, OF_IMAGE_COLOR);
        depthPixels.set(0);
        distancePixels.set(0);
        initFilters();
    }

    if (bUseInfrared) {
        config.enable_stream(RS2_STREAM_INFRARED, depthWidth, depthHeight, RS2_FORMAT_Y8, 30);
        infraredImage.allocate(depthWidth, depthHeight, OF_IMAGE_GRAYSCALE);
        depthPixelsRaw.set(0);
        depthPixelsRawBack.set(0);
    }

    if (bUseTexture) {
        depthTex.allocate(depthPixels);
        videoTex.allocate(videoPixels);
        infraredTex.allocate(infraredPixels);
    }

    if (!realSenseContext.isInited()) {

        if (!realSenseContext.init()) {
            return false;
        }
    }

    bGrabberInited = true;
    return bGrabberInited;
}

bool ofxRealSense2::initDepth(int width, int height) {
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
    depthImage.allocate(width, height, OF_IMAGE_COLOR);
    depthWidth = width;
    depthHeight = height;
    depthPixels.set(0);
    distancePixels.set(0);

    if (!realSenseContext.isInited()) {

        if (!realSenseContext.init()) {
            return false;
        }
    }

    color_map.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0.f);
    color_map.set_option(RS2_OPTION_VISUAL_PRESET, 3.f);
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);

    bGrabberInited = true;
    return bGrabberInited;
}

bool ofxRealSense2::initColor(int width, int height) {
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, 30);
    colorImage.allocate(width, height, OF_IMAGE_COLOR);
    colorWidth = width;
    colorHeight = height;
    videoPixels.set(0);
    videoPixelsBack.set(0);

    if (!realSenseContext.isInited()) {

        if (!realSenseContext.init()) {
            return false;
        }
    }
    bGrabberInited = true;
    return bGrabberInited;
}

bool ofxRealSense2::initInfrared(int width, int height) {
    config.enable_stream(RS2_STREAM_INFRARED, width, height, RS2_FORMAT_Y8, 30);
    infraredImage.allocate(width, height, OF_IMAGE_GRAYSCALE);
    depthPixelsRaw.set(0);
    depthPixelsRawBack.set(0);

    if (!realSenseContext.isInited()) {

        if (!realSenseContext.init()) {
            return false;
        }
    }
    bGrabberInited = true;
    return bGrabberInited;
}

void ofxRealSense2::clear() {
    if (isConnected()) {
        ofLogWarning("ofxRealSense2") << "clear(): do not call clear while ofxRealSense2 is running!";
        return;
    }

    depthPixelsRaw.clear();
    depthPixelsRawBack.clear();

    videoPixels.clear();
    videoPixelsBack.clear();

    depthPixels.clear();
    distancePixels.clear();

    depthTex.clear();
    videoTex.clear();

    bGrabberInited = false;
}

void ofxRealSense2::initFilters() {
    decimationFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1.0);
    // Decimation - reduces depth frame density
    filters.emplace_back("decimation", decimationFilter);

    // complicates the filter logic and doesn't seem useful
    // rs2::disparity_transform depth_to_disparity(true);
    // filters.emplace_back(depth_to_disparity, false);

    // Spatial    - edge-preserving spatial smoothing
    filters.emplace_back("spatial", spatialFilter);

    // Temporal   - reduces temporal noise
    filters.emplace_back("temporal", temporalFilter);

    // Hole Filling
    filters.emplace_back("hole filling", holeFillingFilter);
}

void ofxRealSense2::toggleFilter(string name, bool value) {
    for (auto&& filter : filters) {
        if (filter.name == name) {
            filter.is_enabled = value;
            cout << "Toggling " << name << " filter" << endl;
        }
    }
}

void ofxRealSense2::setFilterOption(const string name, int optionEnum, float value) {
    for (auto&& filter : filters) {
        if (filter.name == name) {
            cout << "Setting " << name << " option " << (rs2_option)optionEnum << endl;
            filter.filterBlock.set_option((rs2_option)optionEnum, value);
        }
    }
}

void ofxRealSense2::setRegistration(bool bUseRegistration) {
	this->bUseRegistration = bUseRegistration;
}

bool ofxRealSense2::open(int deviceIndex) {
    if (!bGrabberInited) {
        ofLogWarning("ofxRealSense2") << "open(): cannot open, init not called";
        return false;
    }

    // we need to find the device id from the device list index.
    // as the first device id could be 1 and not 0 when the device list is sorted by serial
    int deviceIDFromIndex = -1;
    if (deviceIndex > -1) {
        deviceIDFromIndex = realSenseContext.getDeviceId(deviceIndex);
    }

    if (!realSenseContext.open(*this, deviceIDFromIndex)) {
        return false;
    }

    lastDeviceIndex = deviceIndex;
    timeSinceOpen = ofGetElapsedTimef();
    bGotDataVideo = false;
    bGotDataDepth = false;
    bFirstUpdate = true;

    startThread(); // blocking, not verbose

    return true;
}

bool ofxRealSense2::open(string serial) {
    if (!bGrabberInited) {
        ofLogVerbose("ofxRealSense") << "open(): cannot open, init not called";
        return false;
    }

    if (!realSenseContext.open(*this, serial)) {
        return false;
    }

    lastDeviceIndex = realSenseContext.getDeviceIndex(serial);
    timeSinceOpen = ofGetElapsedTimef();
    bGotDataVideo = false;
    bGotDataDepth = false;
    bFirstUpdate = true;

    startThread(); // blocking, not verbose

    return true;
}

bool ofxRealSense2::openFromFile(string filename) {
    if (!bGrabberInited) {
        ofLogVerbose("ofxRealSense") << "open(): cannot open, init not called";
        return false;
    }

    if (!realSenseContext.openFromFile(*this, filename)) {
        return false;
    }

    lastDeviceIndex = realSenseContext.getDeviceIndex(serial);
    timeSinceOpen = ofGetElapsedTimef();
    bGotDataVideo = false;
    bGotDataDepth = false;
    bFirstUpdate = true;

    startThread(); // blocking, not verbose

    return true;
}

void ofxRealSense2::close() {
    if (isThreadRunning()) {
        stopThread();
        ofSleepMillis(10);
        waitForThread(false, 5000);
    }

    deviceId = -1;
    serial = "";
    bIsFrameNewVideo = false;
    bNeedsUpdateVideo = false;
    bIsFrameNewDepth = false;
    bNeedsUpdateDepth = false;

    realSenseContext.close(*this);
}

//---------------------------------------------------------------------------
bool ofxRealSense2::isConnected() const {
    return isThreadRunning();
}

//---------------------------------------------------------------------------
bool ofxRealSense2::isInitialized() const {
    return realSenseContext.isInited();
}

//--------------------------------------------------------------------
bool ofxRealSense2::isFrameNew()  const {
    return isFrameNewVideo() || isFrameNewDepth();
}

//--------------------------------------------------------------------
bool ofxRealSense2::isFrameNewVideo() const {
    return bIsFrameNewVideo;
}

//--------------------------------------------------------------------
bool ofxRealSense2::isFrameNewDepth() const {
    return bIsFrameNewDepth;
}

//--------------------------------------------------------------------
bool ofxRealSense2::setPixelFormat(ofPixelFormat pixelFormat) {
    if (!bIsVideoInfrared && pixelFormat == OF_PIXELS_RGB) {
        return true;
    } else if (pixelFormat == OF_PIXELS_GRAY) {
        return true;
    } else {
        return false;
    }
}

//--------------------------------------------------------------------
ofPixelFormat ofxRealSense2::getPixelFormat() const {
    if (!bIsVideoInfrared) {
        return OF_PIXELS_RGB;
    } else {
        return OF_PIXELS_GRAY;
    }
}

//----------------------------------------------------------
void ofxRealSense2::update() {
    if (!bGrabberInited) {
        return;
    }

    bIsFrameNewDepth = false;

    // - Start handle reconnection

    // we need to do timing for reconnection based on the first update call
    // as a project with a long setup call could exceed the reconnectWaitTime and create a false positive
    // we also need to not try reconnection if the camera is tilting as this can shutoff the data coming in and cause a false positive.
    if (bFirstUpdate) {
        timeSinceOpen = ofGetElapsedTimef();
        bFirstUpdate = false;
    }

    // if we aren't grabbing the video stream we don't need to check for video
    bool bVideoOkay = true;
    if (bGrabVideo) {
        bVideoOkay = bGotDataVideo;
        if (bNeedsUpdateVideo) {
            bVideoOkay = true;
            bGotDataVideo = true;
        }
    }

    if (bNeedsUpdateDepth) {
        bGotDataDepth = true;
    }

    if (videoQueue.poll_for_frame(&color)) {
        if (color.get_data()) {
            videoPixels.setFromPixels((unsigned char *)color.get_data(), colorWidth, colorHeight, OF_IMAGE_COLOR);
            videoTex.loadData(videoPixels);
        }
    }

    rs2::frame f;
    if (filteredDepthQueue.poll_for_frame(&f)) { // Try to take the depth from the queue
        depth = color_map.process(f);     // Colorize the depth frame with a color map
        depthWidth = depth.as<rs2::video_frame>().get_width(); // dimensions change if we decimate
        depthHeight = depth.as<rs2::video_frame>().get_height();
    }

    if (depth) {
        if (depth.get_data()) {
//          depthPixelsRaw.setFromPixels((unsigned char *)depth.get_data(), DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
            depthPixels.setFromPixels((unsigned char *)depth.get_data(), depthWidth, depthHeight, OF_IMAGE_COLOR);
            depthTex.loadData(depthPixels);
            bIsFrameNewDepth = true;
        }
    }

    if (infraredQueue.poll_for_frame(&infrared)) {
        if (infrared.get_data()) {
            infraredPixels.setFromPixels((unsigned char *)infrared.get_data(), depthWidth, depthHeight, OF_IMAGE_GRAYSCALE);
            infraredTex.loadData(infraredPixels);
        }
    }
}

//------------------------------------
float ofxRealSense2::getDistanceAt(int x, int y)  const {

    rs2::depth_frame temp = allset.get_depth_frame();
    return temp.get_distance(x, y);

//    return depthPixelsRaw[y * width + x];
}

//------------------------------------
float ofxRealSense2::getDistanceAt(const ofPoint & p)  const {
    return getDistanceAt(p.x, p.y);
}

//------------------------------------
ofVec3f ofxRealSense2::getWorldCoordinateAt(int x, int y)  const {
    return getWorldCoordinateAt(x, y, getDistanceAt(x, y));
}

//------------------------------------
ofVec3f ofxRealSense2::getWorldCoordinateAt(float cx, float cy, float wz)  const {
    float point[3];
    float pixel[2]{ cx, cy };

    rs2_deproject_pixel_to_point(point, &intr, pixel, wz);
    return ofVec3f(point[0], point[1], point[2]);
}

//------------------------------------
ofColor ofxRealSense2::getColorAt(int x, int y)  const {
    int index = (y * colorWidth + x) * videoBytesPerPixel;
    ofColor c;
    c.r = videoPixels[index + 0];
    c.g = videoPixels[index + (videoBytesPerPixel-1)/2];
    c.b = videoPixels[index + (videoBytesPerPixel-1)];
    c.a = 255;

    return c;
}

//------------------------------------
ofColor ofxRealSense2::getColorAt(const ofPoint & p)  const {
    return getColorAt(p.x, p.y);
}

ofPixels & ofxRealSense2::getPixels() {
    return videoPixels;
}

ofPixels & ofxRealSense2::getDepthPixels() {
    return depthPixels;
}

ofShortPixels & ofxRealSense2::getRawDepthPixels() {
    return depthPixelsRaw;
}

ofFloatPixels & ofxRealSense2::getDistancePixels() {
    return distancePixels;
}

const ofPixels & ofxRealSense2::getPixels() const {
    return videoPixels;
}

const ofPixels & ofxRealSense2::getDepthPixels() const {
    return depthPixels;
}

const ofShortPixels & ofxRealSense2::getRawDepthPixels() const {
    return depthPixelsRaw;
}

const ofFloatPixels & ofxRealSense2::getDistancePixels() const {
    return distancePixels;
}

//------------------------------------
ofTexture& ofxRealSense2::getTexture() {
    if (!videoTex.isAllocated()) {
        ofLogWarning("ofxRealSense2") << "getTexture(): device " << deviceId << " video texture not allocated";
    }
    return videoTex;
}

//---------------------------------------------------------------------------
ofTexture& ofxRealSense2::getDepthTexture() {
    if (!depthTex.isAllocated()) {
        ofLogWarning("ofxRealSense2") << "getDepthTexture(): device " << deviceId << " depth texture not allocated";
    }
    return depthTex;
}

//------------------------------------
const ofTexture& ofxRealSense2::getTexture() const {
    if (!videoTex.isAllocated()) {
        ofLogWarning("ofxRealSense2") << "getTexture(): device " << deviceId << " video texture not allocated";
    }
    return videoTex;
}

//---------------------------------------------------------------------------
const ofTexture& ofxRealSense2::getDepthTexture() const {
    if (!depthTex.isAllocated()) {
        ofLogWarning("ofxRealSense2") << "getDepthTexture(): device " << deviceId << " depth texture not allocated";
    }
    return depthTex;
}

//---------------------------------------------------------------------------
void ofxRealSense2::enableDepthNearValueWhite(bool bEnabled) {
    bNearWhite = bEnabled;
    updateDepthLookupTable();
}

//---------------------------------------------------------------------------
bool ofxRealSense2::isDepthNearValueWhite() const {
    return bNearWhite;
}

void ofxRealSense2::setHighAccuracyPreset() {
    auto sensor = profile.get_device().first<rs2::depth_sensor>();
    std::cout << "Setting Depth Preset: " << std::string(sensor.get_option_value_description(RS2_OPTION_VISUAL_PRESET, 3.0f)) << endl;
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, 3.0f);
}

//---------------------------------------------------------------------------
float ofxRealSense2::getClippingOptionMin() const {
    auto range = color_map.get_option_range(RS2_OPTION_MIN_DISTANCE);

    return range.min;
}

//---------------------------------------------------------------------------
float ofxRealSense2::getClippingOptionMax() const {
    auto range = color_map.get_option_range(RS2_OPTION_MAX_DISTANCE);

    return range.max;
}

float ofxRealSense2::getClippingOptionStep() const {
    auto range = color_map.get_option_range(RS2_OPTION_MAX_DISTANCE); // same for both

    return range.step;
}

//---------------------------------------------------------------------------
float ofxRealSense2::getNearClipping() const {
    return nearClipping;
}

//---------------------------------------------------------------------------
float ofxRealSense2::getFarClipping() const {
    return farClipping;
}

//---------------------------------------------------------------------------
void ofxRealSense2::setNearClipping(float value) {

    if (value < farClipping) {
        color_map.set_option(RS2_OPTION_MIN_DISTANCE, value);
        nearClipping = value;
    } else {
        ofLogWarning("ofxRealSense2") << "Tried to set nearClipping > farClipping";
    }
}

//---------------------------------------------------------------------------
void ofxRealSense2::setFarClipping(float value) {

    if (value > nearClipping) {
        color_map.set_option(RS2_OPTION_MAX_DISTANCE, value);
        farClipping = value;
    } else {
        ofLogWarning("ofxRealSense2") << "Tried to set farClipping < nearClipping";
    }
}

//---------------------------------------------------------------------------
void ofxRealSense2::setUseTexture(bool bUse) {
    bUseTexture = bUse;
}

//------------------------------------
bool ofxRealSense2::isUsingTexture() const {
    return bUseTexture;
}

//----------------------------------------------------------
void ofxRealSense2::draw(float _x, float _y, float _w, float _h) const {
    if (bUseTexture && bGrabVideo) {
        videoTex.draw(_x, _y, _w, _h);
    }
}

//----------------------------------------------------------
void ofxRealSense2::draw(float _x, float _y) const {
    draw(_x, _y, static_cast<float>(depthWidth), static_cast<float>(depthHeight));
}

//----------------------------------------------------------
void ofxRealSense2::draw(const ofPoint & point) const {
    draw(point.x, point.y);
}

//----------------------------------------------------------
void ofxRealSense2::draw(const ofRectangle & rect) const {
    draw(rect.x, rect.y, rect.width, rect.height);
}

//----------------------------------------------------------
void ofxRealSense2::drawDepth(float _x, float _y, float _w, float _h) const {
    if (bUseTexture) {
        depthTex.draw(_x, _y, _w, _h);
    }
}

//---------------------------------------------------------------------------
void ofxRealSense2::drawDepth(float _x, float _y) const {
    drawDepth(_x, _y, static_cast<float>(depthWidth), static_cast<float>(depthHeight));
}

//----------------------------------------------------------
void ofxRealSense2::drawDepth(const ofPoint & point) const {
    drawDepth(point.x, point.y);
}

//----------------------------------------------------------
void ofxRealSense2::drawDepth(const ofRectangle & rect) const {
    drawDepth(rect.x, rect.y, rect.width, rect.height);
}

//----------------------------------------------------------
void ofxRealSense2::drawIR(float _x, float _y, float _w, float _h) const {
    infraredTex.draw(_x, _y, _w, _h);
}
//---------------------------------------------------------------------------
void ofxRealSense2::drawIR(float _x, float _y) const {
    drawIR(_x, _y, static_cast<float>(depthWidth), static_cast<float>(depthHeight));
}
//----------------------------------------------------------
void ofxRealSense2::drawIR(const ofPoint & point) const {
    drawIR(point.x, point.y);
}
//----------------------------------------------------------
void ofxRealSense2::drawIR(const ofRectangle & rect) const {
    drawIR(rect.x, rect.y, rect.width, rect.height);
}
//----------------------------------------------------------
void ofxRealSense2::generatePointCloud() {
    if (depth) {
        points = pointCloud.calculate(depth);

        if (color) {
            pointCloud.map_to(color);
        } else if (infrared) {
            pointCloud.map_to(infrared);
        }
    }
}
//----------------------------------------------------------
void ofxRealSense2::drawPointCloud(float width, float height, rs2::points& points) {
    if (depth) {
        points = pointCloud.calculate(depth);

        if (color) {
            pointCloud.map_to(color);
        } else if (infrared) {
            pointCloud.map_to(infrared);
        }
    }
}

//---------------------------------------------------------------------------
int ofxRealSense2::getDeviceId() const {
    return deviceId;
}

//---------------------------------------------------------------------------
string ofxRealSense2::getSerial() const {
    return serial;
}

//----------------------------------------------------------
float ofxRealSense2::getDepthHeight() const {
    return static_cast<float>(depthHeight);
}

//---------------------------------------------------------------------------
float ofxRealSense2::getDepthWidth() const {
    return static_cast<float>(depthWidth);
}

//----------------------------------------------------------
float ofxRealSense2::getColorHeight() const {
    return static_cast<float>(colorHeight);
}

//---------------------------------------------------------------------------
float ofxRealSense2::getColorWidth() const {
    return static_cast<float>(colorWidth);
}

//----------------------------------------------------------
void ofxRealSense2::listDevices() {
    realSenseContext.listDevices();
}

//----------------------------------------------------------
vector<string> ofxRealSense2::getAvailableSerials() {
    return realSenseContext.getAvailableSerials();
}

//---------------------------------------------------------------------------
int ofxRealSense2::numTotalDevices() {
    return realSenseContext.numTotal();
}

//---------------------------------------------------------------------------
int ofxRealSense2::numAvailableDevices() {
    return realSenseContext.numAvailable();
}

//---------------------------------------------------------------------------
int ofxRealSense2::numConnectedDevices() {
    return realSenseContext.numConnected();
}

//---------------------------------------------------------------------------
bool ofxRealSense2::isDeviceConnected(int id) {
    return realSenseContext.isConnected(id);
}

//---------------------------------------------------------------------------
bool ofxRealSense2::isDeviceConnected(string serial) {
    return realSenseContext.isConnected(serial);
}

//---------------------------------------------------------------------------
int ofxRealSense2::nextAvailableId() {
    return realSenseContext.nextAvailableId();
}

//---------------------------------------------------------------------------
string ofxRealSense2::nextAvailableSerial() {
    return realSenseContext.nextAvailableSerial();
}

//---------------------------------------------------------------------------
void ofxRealSense2::setReconnectWaitTime(float waitTime) {
    reconnectWaitTime = waitTime;
}


/* ***** PRIVATE ***** */

//---------------------------------------------------------------------------
void ofxRealSense2::updateDepthLookupTable() {
    unsigned char nearColor = bNearWhite ? 255 : 0;
    unsigned char farColor = bNearWhite ? 0 : 255;
    unsigned int maxDepthLevels = 10001;
    depthLookupTable.resize(maxDepthLevels);
    depthLookupTable[0] = 0;
    for (unsigned int i = 1; i < maxDepthLevels; i++) {
        depthLookupTable[i] = ofMap(i, nearClipping, farClipping, nearColor, farColor, true);
    }
}

//----------------------------------------------------------
void ofxRealSense2::updateDepthPixels() {
    int n = depthWidth * depthHeight;
    for (int i = 0; i < n; i++) {
        distancePixels[i] = depthPixelsRaw[i];
    }
    for (int i = 0; i < n; i++) {
        depthPixels[i] = depthLookupTable[depthPixelsRaw[i]];
    }
}

//---------------------------------------------------------------------------
void ofxRealSense2::grabDepthFrame(rs2::device *dev, void *depth, uint32_t timestamp) {

    ofxRealSense2* realSense = realSenseContext.getRealSense(dev);

    if (realSense->realSenseDevice == dev) {
        realSense->lock();
        swap(realSense->depthPixelsRawBack, realSense->depthPixelsRawIntra);
        realSense->bNeedsUpdateDepth = true;
        realSense->unlock();
    // is there an analogue
    // freenect_set_depth_buffer(kinect->kinectDevice,kinect->depthPixelsRawBack.getData());
    }
}

//---------------------------------------------------------------------------
void ofxRealSense2::grabVideoFrame(rs2::device *dev, void *video, uint32_t timestamp) {

    ofxRealSense2* realSense = realSenseContext.getRealSense(dev);

    if (realSense->realSenseDevice == dev) {
        realSense->lock();
        swap(realSense->videoPixelsBack, realSense->videoPixelsIntra);
        realSense->bNeedsUpdateVideo = true;
        realSense->unlock();
    // see if there is an analogue
    // freenect_set_video_buffer(kinect->kinectDevice,kinect->videoPixelsBack.getData());
    }
}

//---------------------------------------------------------------------------
void ofxRealSense2::threadedFunction() {
    while (isThreadRunning()) {

        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::depth_frame depth_frame = data.get_depth_frame(); // Take the depth frame from the frameset
        if (!depth_frame) // Should not happen but if the pipeline is configured differently
            return;       //  it might not provide depth and we don't want to crash

        rs2::frame filtered = depth_frame; // Does not copy the frame, only adds a reference

        for (auto&& filter : filters) {
            if (filter.is_enabled) {
                filtered = filter.filterBlock.process(filtered);
            }
        }

        // Push filtered & original data to their respective queues
        // Note, pushing to two different queues might cause the application to display
        //  original and filtered pointclouds from different depth frames
        //  To make sure they are synchronized you need to push them together or add some
        //  synchronization mechanisms

        // int w = filtered.as<rs2::video_frame>().get_width();
        // ofLog(OF_LOG_WARNING, "FILTERED WIDTH " + to_string(w));
        filteredDepthQueue.enqueue(filtered);
        // rawDepthQueue.enqueue(depth_frame);

        // pass through video and ir frames unaltered
        if (bGrabVideo)
            videoQueue.enqueue(data.get_color_frame());
        if (bUseInfrared)
            infraredQueue.enqueue(data.get_infrared_frame());
    }
}
