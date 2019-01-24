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



ofxRealSense2::ofxRealSense2(){
  ofLogVerbose("ofxRealSense2") << "creating ofxRealSense2";

  deviceId = -1;
  serial = "";

  bUseTexture = true;
  bGrabVideo = true;

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

  //setDepthClipping();

  nearClipping = 0.0;
  farClipping = 6.0;

}

ofxRealSense2::~ofxRealSense2(){
  // close? clear?
}

bool ofxRealSense2::init(bool infrared, bool video, bool texture) {

    bIsVideoInfrared = infrared;
  	bGrabVideo = video;
    bUseTexture = texture;

    // set configuration
    if(bGrabVideo){
        config.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_RGB8, 30);
        colorImage.allocate(COLOR_WIDTH, COLOR_HEIGHT, OF_IMAGE_COLOR);
        videoPixels.set(0);
	    videoPixelsBack.set(0);
    }

    if(bUseDepth){
        config.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, 30);
        depthImage.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
        depthPixels.set(0);
        distancePixels.set(0);
    }

    if(bUseInfrared) {
        config.enable_stream(RS2_STREAM_INFRARED, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Y8, 30);
        infraredImage.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
        depthPixelsRaw.set(0);
        depthPixelsRawBack.set(0);
    }

    if(bUseTexture) {
        depthTex.allocate(depthPixels);
        videoTex.allocate(videoPixels);
        infraredTex.allocate(infraredPixels);
    }

    if(!realSenseContext.isInited()) {

  		if(!realSenseContext.init()) {
  			return false;
  		}
  	}

    color_map.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0.f);
    color_map.set_option(RS2_OPTION_VISUAL_PRESET, 3.f);
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);

    bGrabberInited = true;
    return bGrabberInited;
}

void ofxRealSense2::clear() {
  if(isConnected()) {
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

void ofxRealSense2::setRegistration(bool bUseRegistration) {
	this->bUseRegistration = bUseRegistration;
}

bool ofxRealSense2::open(int deviceIndex) {
    if(!bGrabberInited) {
        ofLogWarning("ofxRealSense2") << "open(): cannot open, init not called";
        return false;
    }

    //we need to find the device id from the device list index.
    //as the first device id could be 1 and not 0 when the device list is sorted by serial
    int deviceIDFromIndex = -1;
    if( deviceIndex > -1 ){
        deviceIDFromIndex = realSenseContext.getDeviceId(deviceIndex);
    }

    if(!realSenseContext.open(*this, deviceIDFromIndex)) {
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
	if(!bGrabberInited) {
        ofLogVerbose("ofxRealSense") << "open(): cannot open, init not called";
		return false;
	}

	if(!realSenseContext.open(*this, serial)) {
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
	if(isThreadRunning()) {
		stopThread();
		ofSleepMillis(10);
		waitForThread(false,5000);
	}

	deviceId = -1;
	serial = "";
	bIsFrameNewVideo = false;
	bNeedsUpdateVideo = false;
	bIsFrameNewDepth = false;
	bNeedsUpdateDepth = false;
}

//---------------------------------------------------------------------------
bool ofxRealSense2::isConnected() const{
	return isThreadRunning();
}

//---------------------------------------------------------------------------
bool ofxRealSense2::isInitialized() const{
	return realSenseContext.isInited();
}

//--------------------------------------------------------------------
bool ofxRealSense2::isFrameNew()  const{
	return isFrameNewVideo() || isFrameNewDepth();
}

//--------------------------------------------------------------------
bool ofxRealSense2::isFrameNewVideo() const{
	return bIsFrameNewVideo;
}

//--------------------------------------------------------------------
bool ofxRealSense2::isFrameNewDepth() const{
	return bIsFrameNewDepth;
}

//--------------------------------------------------------------------
bool ofxRealSense2::setPixelFormat(ofPixelFormat pixelFormat){
	if(!bIsVideoInfrared && pixelFormat==OF_PIXELS_RGB){
		return true;
	}else if(pixelFormat == OF_PIXELS_GRAY){
		return true;
	}else{
		return false;
	}
}

//--------------------------------------------------------------------
ofPixelFormat ofxRealSense2::getPixelFormat() const{
	if(!bIsVideoInfrared){
		return OF_PIXELS_RGB;
	}else{
		return OF_PIXELS_GRAY;
	}
}

//----------------------------------------------------------
void ofxRealSense2::update() {
	if(!bGrabberInited) {
		return;
	}

    // - Start handle reconnection

    //we need to do timing for reconnection based on the first update call
    //as a project with a long setup call could exceed the reconnectWaitTime and create a false positive
    //we also need to not try reconnection if the camera is tilting as this can shutoff the data coming in and cause a false positive.
    if( bFirstUpdate){
        timeSinceOpen = ofGetElapsedTimef();
        bFirstUpdate = false;
    }

    //if we aren't grabbing the video stream we don't need to check for video
    bool bVideoOkay = true;
    if( bGrabVideo ){
        bVideoOkay = bGotDataVideo;
        if( bNeedsUpdateVideo ){
            bVideoOkay = true;
            bGotDataVideo = true;
        }
    }

    if( bNeedsUpdateDepth ){
        bGotDataDepth = true;
    }

    //try reconnect if we don't have color coming in or if we don't have depth coming in
//    if( (!bVideoOkay || !bGotDataDepth ) && tryCount < 5 && ofGetElapsedTimef() - timeSinceOpen > reconnectWaitTime ){
//        close();
//        ofLogWarning("ofxKinect") << "update(): device " << lastDeviceIndex << " isn't delivering data. depth: " << bGotDataDepth << " color: " << bGotDataVideo <<"  , reconnecting tries: " << tryCount+1;
//        realSenseContext.buildDeviceList();
//        open(lastDeviceIndex);
//        tryCount++;
//        timeSinceOpen = ofGetElapsedTimef();
//    }

    // - End handle reconnection

    rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
    
    // aligns the depth and color frame
    rs2_stream align_to = RS2_STREAM_COLOR;
    rs2::align align(align_to);
    rs2::frameset processed = align.process(data);
    
    allset = processed;
    
    //color_map.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 1.f);
    //color_map.set_option(RS2_OPTION_MAX_DISTANCE, 2.7f); //TESTING

    depth = color_map.process(data.get_depth_frame()); // Find and colorize the depth data
    color = data.get_color_frame();            // Find the color data
    infrared = data.get_infrared_frame();
    intr = data.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    
    if(color){
        if(color.get_data()){
            videoPixels.setFromPixels((unsigned char *)color.get_data(), COLOR_WIDTH, COLOR_HEIGHT, OF_IMAGE_COLOR);
            videoTex.loadData(videoPixels);
        }
    }
    
    if(depth) {
        if(depth.get_data())
        {
//            depthPixelsRaw.setFromPixels((unsigned char *)depth.get_data(), DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
            depthPixels.setFromPixels((unsigned char *)depth.get_data(), DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);
            depthTex.loadData(depthPixels);
        }
    }

    if(infrared) {
        infraredPixels.setFromPixels((unsigned char *)infrared.get_data(), DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
        infraredTex.loadData(infraredPixels);
    }
    
}

//------------------------------------
float ofxRealSense2::getDistanceAt(int x, int y)  const{
    
    rs2::depth_frame temp = allset.get_depth_frame();
    return temp.get_distance(x, y);

//    return depthPixelsRaw[y * width + x];
}

//------------------------------------
float ofxRealSense2::getDistanceAt(const ofPoint & p)  const{
	return getDistanceAt(p.x, p.y);
}

//------------------------------------
ofVec3f ofxRealSense2::getWorldCoordinateAt(int x, int y)  const{
	return getWorldCoordinateAt(x, y, getDistanceAt(x, y));
}

//------------------------------------
ofVec3f ofxRealSense2::getWorldCoordinateAt(float cx, float cy, float wz)  const{
    float point[3];
    float pixel[2]{ cx, cy };
    
    rs2_deproject_pixel_to_point(point, &intr, pixel, wz);
	return ofVec3f(point[0], point[1], point[2]);
}

//------------------------------------
ofColor ofxRealSense2::getColorAt(int x, int y)  const{
	int index = (y * width + x) * videoBytesPerPixel;
	ofColor c;
	c.r = videoPixels[index + 0];
	c.g = videoPixels[index + (videoBytesPerPixel-1)/2];
	c.b = videoPixels[index + (videoBytesPerPixel-1)];
	c.a = 255;
    
	return c;
}

//------------------------------------
ofColor ofxRealSense2::getColorAt(const ofPoint & p)  const{
	return getColorAt(p.x, p.y);
}

ofPixels & ofxRealSense2::getPixels(){
	return videoPixels;
}

ofPixels & ofxRealSense2::getDepthPixels(){
	return depthPixels;
}

ofShortPixels & ofxRealSense2::getRawDepthPixels(){
	return depthPixelsRaw;
}

ofFloatPixels & ofxRealSense2::getDistancePixels(){
	return distancePixels;
}

const ofPixels & ofxRealSense2::getPixels() const{
	return videoPixels;
}

const ofPixels & ofxRealSense2::getDepthPixels() const{
	return depthPixels;
}

const ofShortPixels & ofxRealSense2::getRawDepthPixels() const{
	return depthPixelsRaw;
}

const ofFloatPixels & ofxRealSense2::getDistancePixels() const{
	return distancePixels;
}

//------------------------------------
ofTexture& ofxRealSense2::getTexture(){
	if(!videoTex.isAllocated()){
		ofLogWarning("ofxRealSense2") << "getTexture(): device " << deviceId << " video texture not allocated";
	}
	return videoTex;
}

//---------------------------------------------------------------------------
ofTexture& ofxRealSense2::getDepthTexture(){
	if(!depthTex.isAllocated()){
		ofLogWarning("ofxRealSense2") << "getDepthTexture(): device " << deviceId << " depth texture not allocated";
	}
	return depthTex;
}

//------------------------------------
const ofTexture& ofxRealSense2::getTexture() const{
	if(!videoTex.isAllocated()){
		ofLogWarning("ofxRealSense2") << "getTexture(): device " << deviceId << " video texture not allocated";
	}
	return videoTex;
}

//---------------------------------------------------------------------------
const ofTexture& ofxRealSense2::getDepthTexture() const{
	if(!depthTex.isAllocated()){
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
bool ofxRealSense2::isDepthNearValueWhite() const{
	return bNearWhite;
}

/*---------------------------------------------------------------------------
void ofxRealSense2::setDepthClipping(float nearClip, float farClip) {
	nearClipping = nearClip;
	farClipping = farClip;
	updateDepthLookupTable();
}*/

//---------------------------------------------------------------------------
float ofxRealSense2::getClippingOptionMin() const{

    //auto depthSensor = profile.get_device().first<rs2::depth_sensor>();
    auto range = color_map.get_option_range(RS2_OPTION_MIN_DISTANCE);

    return range.min;
}

//---------------------------------------------------------------------------
float ofxRealSense2::getClippingOptionMax() const{

    //auto depthSensor = profile.get_device().first<rs2::depth_sensor>();
    auto range = color_map.get_option_range(RS2_OPTION_MAX_DISTANCE);

    return range.max;
}

float ofxRealSense2::getClippingOptionStep() const{

    //auto depthSensor = profile.get_device().first<rs2::depth_sensor>();
    auto range = color_map.get_option_range(RS2_OPTION_MAX_DISTANCE); //same for both

    return range.step;
}

//---------------------------------------------------------------------------
float ofxRealSense2::getNearClipping() const{
    return nearClipping;
}

//---------------------------------------------------------------------------
float ofxRealSense2::getFarClipping() const{
    return farClipping;
}

//---------------------------------------------------------------------------
void ofxRealSense2::setNearClipping(float value) {

    if (value < farClipping) {
        //auto depthSensor = profile.get_device().first<rs2::depth_sensor>();
        color_map.set_option(RS2_OPTION_MIN_DISTANCE, value);
        nearClipping = value;
    } else {
        ofLogWarning("ofxRealSense2") << "Tried to set nearClipping > farClipping";
    }
}

//---------------------------------------------------------------------------
void ofxRealSense2::setFarClipping(float value) {

    if (value > nearClipping) {
        //auto depthSensor = profile.get_device().first<rs2::depth_sensor>();
        color_map.set_option(RS2_OPTION_MAX_DISTANCE, value);
        farClipping = value;
    } else {
        ofLogWarning("ofxRealSense2") << "Tried to set farClipping < nearClipping";
    }
}

//---------------------------------------------------------------------------
void ofxRealSense2::setUseTexture(bool bUse){
	bUseTexture = bUse;
}

//------------------------------------
bool ofxRealSense2::isUsingTexture() const{
	return bUseTexture;
}

//----------------------------------------------------------
void ofxRealSense2::draw(float _x, float _y, float _w, float _h) const{
	if(bUseTexture && bGrabVideo) {
		videoTex.draw(_x, _y, _w, _h);
	}
    // test
//    colorImage.draw(_x, _y, _w, _h);
}

//----------------------------------------------------------
void ofxRealSense2::draw(float _x, float _y) const{
	draw(_x, _y, (float)width, (float)height);
}

//----------------------------------------------------------
void ofxRealSense2::draw(const ofPoint & point) const{
	draw(point.x, point.y);
}

//----------------------------------------------------------
void ofxRealSense2::draw(const ofRectangle & rect) const{
	draw(rect.x, rect.y, rect.width, rect.height);
}

//----------------------------------------------------------
void ofxRealSense2::drawDepth(float _x, float _y, float _w, float _h) const{
	if(bUseTexture) {
		depthTex.draw(_x, _y, _w, _h);
	}
}

//---------------------------------------------------------------------------
void ofxRealSense2::drawDepth(float _x, float _y) const{
	drawDepth(_x, _y, (float)width, (float)height);
}

//----------------------------------------------------------
void ofxRealSense2::drawDepth(const ofPoint & point) const{
	drawDepth(point.x, point.y);
}

//----------------------------------------------------------
void ofxRealSense2::drawDepth(const ofRectangle & rect) const{
	drawDepth(rect.x, rect.y, rect.width, rect.height);
}

//----------------------------------------------------------
void ofxRealSense2::drawIR(float _x, float _y, float _w, float _h) const{
    infraredTex.draw(_x, _y, _w, _h);
//    infraredImage.draw(_x, _y, _w, _h);
}
//---------------------------------------------------------------------------
void ofxRealSense2::drawIR(float _x, float _y) const{
    drawIR(_x, _y, (float)DEPTH_WIDTH, (float)DEPTH_HEIGHT);
}
//----------------------------------------------------------
void ofxRealSense2::drawIR(const ofPoint & point) const{
    drawIR(point.x, point.y);
}
//----------------------------------------------------------
void ofxRealSense2::drawIR(const ofRectangle & rect) const{
    drawIR(rect.x, rect.y, rect.width, rect.height);
}
//----------------------------------------------------------
void ofxRealSense2::generatePointCloud(){
    if(depth){
        points = pointCloud.calculate(depth);
        
        if (color) {
            pointCloud.map_to(color);
        }
        else if (infrared) {
            pointCloud.map_to(infrared);
        }
    }
}
//----------------------------------------------------------
void ofxRealSense2::drawPointCloud(float width, float height, rs2::points& points){
    if(depth){
        points = pointCloud.calculate(depth);
        
        if (color) {
            pointCloud.map_to(color);
        }
        else if (infrared) {
            pointCloud.map_to(infrared);
        }
    }
}

//---------------------------------------------------------------------------
int ofxRealSense2::getDeviceId() const{
	return deviceId;
}

//---------------------------------------------------------------------------
string ofxRealSense2::getSerial() const{
	return serial;
}

//----------------------------------------------------------
float ofxRealSense2::getHeight() const{
	return (float) height;
}

//---------------------------------------------------------------------------
float ofxRealSense2::getWidth() const{
	return (float) width;
}

//----------------------------------------------------------
void ofxRealSense2::listDevices() {
	realSenseContext.listDevices();
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
	for(unsigned int i = 1; i < maxDepthLevels; i++) {
		depthLookupTable[i] = ofMap(i, nearClipping, farClipping, nearColor, farColor, true);
	}
}

//----------------------------------------------------------
void ofxRealSense2::updateDepthPixels() {
	int n = width * height;
	for(int i = 0; i < n; i++) {
		distancePixels[i] = depthPixelsRaw[i];
	}
	for(int i = 0; i < n; i++) {
		depthPixels[i] = depthLookupTable[depthPixelsRaw[i]];
	}
}

//---------------------------------------------------------------------------
void ofxRealSense2::grabDepthFrame(rs2::device *dev, void *depth, uint32_t timestamp) {

	ofxRealSense2* realSense = realSenseContext.getRealSense(dev);

	if(realSense->realSenseDevice == dev) {
		realSense->lock();
		swap(realSense->depthPixelsRawBack,realSense->depthPixelsRawIntra);
		realSense->bNeedsUpdateDepth = true;
		realSense->unlock();
    // is there an analogue
		// freenect_set_depth_buffer(kinect->kinectDevice,kinect->depthPixelsRawBack.getData());
    }
}

//---------------------------------------------------------------------------
void ofxRealSense2::grabVideoFrame(rs2::device *dev, void *video, uint32_t timestamp) {

	ofxRealSense2* realSense = realSenseContext.getRealSense(dev);

	if(realSense->realSenseDevice == dev) {
		realSense->lock();
		swap(realSense->videoPixelsBack,realSense->videoPixelsIntra);
		realSense->bNeedsUpdateVideo = true;
		realSense->unlock();
    // see if there is an analogue
		// freenect_set_video_buffer(kinect->kinectDevice,kinect->videoPixelsBack.getData());
	}
}

//---------------------------------------------------------------------------
void ofxRealSense2::threadedFunction(){

  // is there an analogue ? Is this the config ?
	// freenect_frame_mode videoMode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, bIsVideoInfrared?FREENECT_VIDEO_IR_8BIT:FREENECT_VIDEO_RGB);
	// freenect_set_video_mode(kinectDevice, videoMode);
	// freenect_frame_mode depthMode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, bUseRegistration?FREENECT_DEPTH_REGISTERED:FREENECT_DEPTH_MM);
	// freenect_set_depth_mode(kinectDevice, depthMode);

	ofLogVerbose("ofxRealSense2") << "device " << deviceId << " " << serial << " connection opened";

  // is there an analogue ? is this the pipeline start?
	// freenect_start_depth(kinectDevice);
	if(bGrabVideo) {
    // is there an analogue ? is this the pipeline start?
		// freenect_start_video(kinectDevice);
	}


  // is there an analogue to this ?
	// freenect_stop_depth(kinectDevice);
	// freenect_stop_video(kinectDevice);


  realSenseContext.close(*this);
	ofLogVerbose("ofxRealSense2") << "device " << deviceId << " connection closed";
}

// //---------------------------------------------------------------------------
// // ofxRealSenseContext
// //---------------------------------------------------------------------------
//
// //---------------------------------------------------------------------------

void print(const rs2_extrinsics& extrinsics)
{
    stringstream ss;
    ss << "Rotation Matrix:\n";
    
    for (auto i = 0 ; i < 3 ; ++i)
    {
        for (auto j = 0 ; j < 3 ; ++j)
        {
            ss << left << setw(15) << setprecision(5) << extrinsics.rotation[j*3 +i];
        }
        ss << endl;
    }
    
    ss << "\nTranslation Vector: ";
    for (auto i = 0 ; i < sizeof(extrinsics.translation)/sizeof(extrinsics.translation[0]) ; ++i)
        ss << setprecision(15) << extrinsics.translation[i] << "  ";
    
    cout << ss.str() << endl << endl;
}

void print(const rs2_motion_device_intrinsic& intrinsics)
{
    stringstream ss;
    ss << "Bias Variances: ";
    
    for (auto i = 0 ; i < sizeof(intrinsics.bias_variances)/sizeof(intrinsics.bias_variances[0]) ; ++i)
        ss << setprecision(15) << intrinsics.bias_variances[i] << "  ";
    
    ss << "\nNoise Variances: ";
    for (auto i = 0 ; i < sizeof(intrinsics.noise_variances)/sizeof(intrinsics.noise_variances[0]) ; ++i)
        ss << setprecision(15) << intrinsics.noise_variances[i] << "  ";
    
    ss << "\nData: " << std::endl;
    for (auto i = 0 ; i < sizeof(intrinsics.data)/sizeof(intrinsics.data[0]) ; ++i)
    {
        for (auto j = 0 ; j < sizeof(intrinsics.data[0])/sizeof(intrinsics.data[0][0]) ; ++j)
            ss << std::setw(13) << setprecision(10) << intrinsics.data[i][j] << "  ";
        ss << "\n";
    }
    
    
    cout << ss.str() << endl << endl;
}

void print(const rs2_intrinsics& intrinsics)
{
    stringstream ss;
    ss << left << setw(14) << "Width: "      << "\t" << intrinsics.width  << endl <<
    left << setw(14) << "Height: "     << "\t" << intrinsics.height << endl <<
    left << setw(14) << "PPX: "        << "\t" << setprecision(15)  << intrinsics.ppx << endl <<
    left << setw(14) << "PPY: "        << "\t" << setprecision(15)  << intrinsics.ppy << endl <<
    left << setw(14) << "Fx: "         << "\t" << setprecision(15)  << intrinsics.fx  << endl <<
    left << setw(14) << "Fy: "         << "\t" << setprecision(15)  << intrinsics.fy  << endl <<
    left << setw(14) << "Distortion: " << "\t" << rs2_distortion_to_string(intrinsics.model) << endl <<
    left << setw(14) << "Coeffs: ";
    
    for (auto i = 0 ; i < sizeof(intrinsics.coeffs)/sizeof(intrinsics.coeffs[0]) ; ++i)
        ss << "\t" << setprecision(15) << intrinsics.coeffs[i] << "  ";
    
    cout << ss.str() << endl << endl;
}

bool safe_get_intrinsics(const rs2::video_stream_profile& profile, rs2_intrinsics& intrinsics)
{
    bool ret = false;
    try
    {
        intrinsics = profile.get_intrinsics();
        ret = true;
    }
    catch(...)
    {}
    
    return ret;
}

bool safe_get_motion_intrinsics(const rs2::motion_stream_profile& profile, rs2_motion_device_intrinsic& intrinsics)
{
    bool ret = false;
    try
    {
        intrinsics = profile.get_motion_intrinsics();
        ret = true;
    }
    catch(...)
    {}
    return ret;
}

struct stream_and_resolution{
    rs2_stream stream;
    int stream_index;
    int width;
    int height;
    string stream_name;
    
    bool operator <(const stream_and_resolution& obj) const
    {
        return (std::make_tuple(stream, stream_index, width, height) < std::make_tuple(obj.stream, obj.stream_index, obj.width, obj.height));
    }
};

struct stream_and_index{
    rs2_stream stream;
    int stream_index;
    
    bool operator <(const stream_and_index& obj) const
    {
        return (std::make_tuple(stream, stream_index) < std::make_tuple(obj.stream, obj.stream_index));
    }
};


bool operator ==(const rs2_intrinsics& lhs,
                 const rs2_intrinsics& rhs)
{
    return lhs.width == rhs.width &&
    lhs.height == rhs.height &&
    lhs.ppx == rhs.ppx &&
    lhs.ppy == rhs.ppy &&
    lhs.fx == rhs.fx &&
    lhs.fy == rhs.fy &&
    lhs.model == rhs.model &&
    !std::memcmp(lhs.coeffs, rhs.coeffs, sizeof(rhs.coeffs));
}

bool operator ==(const rs2_motion_device_intrinsic& lhs,
                 const rs2_motion_device_intrinsic& rhs)
{
    return !std::memcmp(&lhs, &rhs, sizeof(rs2_motion_device_intrinsic));
}

string get_str_formats(const set<rs2_format>& formats)
{
    stringstream ss;
    for (auto format = formats.begin(); format != formats.end(); ++format)
    {
        ss << *format << ((format != formats.end()) && (next(format) == formats.end())?"":"/");
    }
    return ss.str();
}

ofxRealSenseContext::ofxRealSenseContext() {
  bInited = false;
  realSenseContext = NULL;
}
ofxRealSenseContext::~ofxRealSenseContext() {
  closeAll();
  clear();
}
//
// //---------------------------------------------------------------------------
static bool sortRealSensePairs(ofxRealSenseContext::RealSensePair A, ofxRealSenseContext::RealSensePair B){
	return A.serial < B.serial;
}
//
// //---------------------------------------------------------------------------
bool ofxRealSenseContext::init() {

    rs2_error* e = 0;
    bInited = true;
    ofLogVerbose("ofxRealSense") << "context inited";
    cout << "INIT -ED THIS THING" << endl;
    
    realSenseContext = new rs2::context();

    buildDeviceList();
	listDevices(true);

    return true;
}

//---------------------------------------------------------------------------
void ofxRealSenseContext::clear() {
	if(isInited() && numConnected() < 1) {

//    freenect_shutdown(realSenseContext);
    // no idea what the analogue is perhaps there is a shutdown?
    // freenect_shutdown(kinectContext);
		realSenseContext = NULL;
		bInited = false;
	}
}

bool ofxRealSenseContext::isInited() {
	return bInited;
}

bool ofxRealSenseContext::open(ofxRealSense2& realSense, int id) {

	// rebuild if necessary (aka new real sense plugged in)
	buildDeviceList();

	if(numConnected() >= numTotal()) {
		ofLogWarning("ofxRealSense2") << "no available devices found";
		return false;
	}

	// is the id available?
	if(id < 0) {
		id = nextAvailableId();
	}
	if(isConnected(id)) {
		ofLogWarning("ofxRealSense2") << "device " << id << " already connected";
		return false;
	}

    cout << "IN CONTEXT TRYING TO OPEN DEVICE BY ID: " << id << endl;
    cout << deviceList[id].serial << endl;
    
    realSense.config.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_RGB8, 30);
    realSense.config.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, 30);
    realSense.config.enable_stream(RS2_STREAM_INFRARED, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Y8, 30);
    realSense.config.enable_device(deviceList[id].serial);
    realSense.profile = realSense.pipe.start(realSense.config);
    
    
	realSenses.insert(pair<int,ofxRealSense2*>(id, &realSense));

	// set kinect id & serial from bus id
	realSense.deviceId = id;
	realSense.serial = deviceList[getDeviceIndex(id)].serial;

	return true;
}

bool ofxRealSenseContext::open(ofxRealSense2& realSense, string serial) {

    // rebuild if necessary (aka new kinects plugged in)
    buildDeviceList();

    if(numConnected() >= numTotal()) {
        ofLogWarning("ofxRealSense2") << "no available devices found";
        return false;
    }

    // is the serial available?
    if(isConnected(serial)) {
        ofLogWarning("ofxRealSense2") << "device " << serial << " already connected";
        return false;
    }

    cout << "IN CONTEXT TRYING TO OPEN DEVICE BY SERIAL: " << serial << endl;

    int index = getDeviceIndex(serial);
    realSense.config.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_RGB8, 30);
    realSense.config.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, 30);
    realSense.config.enable_stream(RS2_STREAM_INFRARED, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Y8, 30);
    realSense.config.enable_device(serial);
    realSense.profile = realSense.pipe.start(realSense.config);
    
	realSenses.insert(pair<int,ofxRealSense2*>(deviceList[index].id, &realSense));
	realSense.deviceId = deviceList[index].id;
	realSense.serial = serial;

	return true;
}

void ofxRealSenseContext::close(ofxRealSense2& realSense) {

	// check if it's already closed
	int id = -1;
	std::map<int,ofxRealSense2*>::iterator iter;
	for(iter = realSenses.begin(); iter != realSenses.end(); ++iter) {
		if(iter->second == &realSense) {
			id = iter->first;
			break;
		}
	}
	if(id == -1)
		return;

	// remove connected device and close
	iter = realSenses.find(id);
	if(iter != realSenses.end()) {
		realSenses.erase(iter);
    // will need to find an analogue

		// freenect_close_device(kinect.kinectDevice);
	}
}

void ofxRealSenseContext::closeAll() {
	// make copy of map to avoid invalidating iter when calling close()
	std::map<int,ofxRealSense2*> realSensesCopy(realSenses);
    std::map<int,ofxRealSense2*>::iterator iter;
    for(iter = realSensesCopy.begin(); iter != realSensesCopy.end(); ++iter) {
        iter->second->close();
    }
}

//---------------------------------------------------------------------------
void ofxRealSenseContext::buildDeviceList() {

	deviceList.clear();

	// build the device list from freenect
  // this thing needs to have an analogue below
	// freenect_device_attributes * devAttrib;
  // this also needs an analogue
//  int numDevices = freenect_list_device_attributes(realSenseContext, &devAttrib);
//    // int numDevices = freenect_list_device_attributes(kinectContext, &devAttrib);
//
//    // save bus ids ...
//    for(int i = 0; i < numDevices; i++){
//        RealSensePair rp;
//        rp.id = i;
//        rp.serial = (string) devAttrib->camera_serial;
//        deviceList.push_back(rp);
//        devAttrib = devAttrib->next;
//    }
//  // this also needs an analogue
//    freenect_free_device_attributes(devAttrib);

    // Obtain a list of devices currently present on the system
    
    auto devices = realSenseContext->query_devices();
    size_t device_count = devices.size();
    if (!device_count)
    {
        cout <<"No device detected. Is it plugged in?\n";
        //return EXIT_SUCCESS;
    }
    else{
        cout << "You have " << device_count << " devices" << endl;
    }
    
//    if (compact_view_arg.getValue())
//    {

        cout << left << setw(30) << "Device Name"
        << setw(20) << "Serial Number"
        << setw(20) << "Firmware Version"
        << endl;
    
        int i = 0;
        for (auto&& dev : devices) // Query the list of connected RealSense devices
        {
//        for (int i = 0; i < device_count; i++)
//        {
            
//            auto dev = devices[i];
            
            RealSensePair rp;
            rp.id = i;
            rp.serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
            
            cout << left << setw(30) << dev.get_info(RS2_CAMERA_INFO_NAME)
            << setw(20) << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
            << setw(20) << dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
            << endl;
            
            deviceList.push_back(rp);
            ++i;
        }
        
//        if (show_options.getValue() || show_modes.getValue())
//            cout << "\n\nNote:  \"-s\" option is not compatible with the other flags specified,"
//            << " all the additional options are skipped" << endl;
//
//        return EXIT_SUCCESS;
//    }
//
//    for (auto i = 0; i < device_count; ++i)
//    {
//        auto dev = devices[i];
//
//        // Show which options are supported by this device
//        cout << " Device info: \n";
//        for (auto j = 0; j < RS2_CAMERA_INFO_COUNT; ++j)
//        {
//            auto param = static_cast<rs2_camera_info>(j);
//            if (dev.supports(param))
//                cout << "    " << left << setw(30) << rs2_camera_info_to_string(rs2_camera_info(param))
//                << ": \t" << dev.get_info(param) << endl;
//        }
//
//        cout << endl;
//
//        if (true)
////        if (show_options.getValue())
//        {
//            for (auto&& sensor : dev.query_sensors())
//            {
//                cout << "Options for " << sensor.get_info(RS2_CAMERA_INFO_NAME) << endl;
//
//                cout << setw(55) << " Supported options:" << setw(10) << "min" << setw(10)
//                << " max" << setw(6) << " step" << setw(10) << " default" << endl;
//                for (auto j = 0; j < RS2_OPTION_COUNT; ++j)
//                {
//                    auto opt = static_cast<rs2_option>(j);
//                    if (sensor.supports(opt))
//                    {
////                        auto range = sensor.get_option_range(opt);
////                        cout << "    " << left << setw(50) << opt << " : "
////                        << setw(5) << range.min << "... " << setw(12) << range.max
////                        << setw(6) << range.step << setw(10) << range.def << "\n";
//                    }
//                }
//
//                cout << endl;
//            }
//        }
//
//        if (true)
////        if (show_modes.getValue())
//        {
//            for (auto&& sensor : dev.query_sensors())
//            {
//                cout << "Stream Profiles supported by " << sensor.get_info(RS2_CAMERA_INFO_NAME) << endl;
//
//                cout << setw(55) << " Supported modes:" << setw(10) << "stream" << setw(10)
//                << " resolution" << setw(6) << " fps" << setw(10) << " format" << endl;
//                // Show which streams are supported by this device
////                for (auto&& profile : sensor.get_stream_profiles())
////                {
////                    if (auto video = profile.as<rs2::video_stream_profile>())
////                    {
////                        cout << "    " << profile.stream_name() << "\t  " << video.width() << "x"
////                        << video.height() << "\t@ " << profile.fps() << "Hz\t" << profile.format() << endl;
////                    }
////                    else
////                    {
////                        cout << "    " << profile.stream_name() << "\t@ " << profile.fps() << "Hz\t" << profile.format() << endl;
////                    }
////                }
//
//                cout << endl;
//            }
//        }
//
//        for (auto&& sensor : dev.query_sensors())
//        {
//            cout << "Stream Profiles supported by " << sensor.get_info(RS2_CAMERA_INFO_NAME) << endl;
//
//            cout << setw(55) << " Supported modes:" << setw(10) << "stream" << setw(10)
//            << " resolution" << setw(6) << " fps" << setw(10) << " format" << endl;
//            // Show which streams are supported by this device
////            for (auto&& profile : sensor.get_stream_profiles())
////            {
////                if (auto video = profile.as<rs2::video_stream_profile>())
////                {
////                    cout << "    " << profile.stream_name() << "\t  " << video.width() << "x"
////                    << video.height() << "\t@ " << profile.fps() << "Hz\t" << profile.format() << endl;
////                }
////                else
////                {
////                    cout << "    " << profile.stream_name() << "\t@ " << profile.fps() << "Hz\t" << profile.format() << endl;
////                }
////            }
//
//            cout << endl;
//        }
//
//        // Print Intrinsics
//        if (true)
////        if (show_calibration_data.getValue())
//        {
//            std::map<stream_and_index, rs2::stream_profile> streams;
//            std::map<stream_and_resolution, std::vector<std::pair<std::set<rs2_format>, rs2_intrinsics>>> intrinsics_map;
//            std::map<stream_and_resolution, std::vector<std::pair<std::set<rs2_format>, rs2_motion_device_intrinsic>>> motion_intrinsics_map;
//            for (auto&& sensor : dev.query_sensors())
//            {
//                // Intrinsics
////                for (auto&& profile : sensor.get_stream_profiles())
////                {
////                    if (auto video = profile.as<rs2::video_stream_profile>())
////                    {
////                        if (streams.find(stream_and_index{profile.stream_type(), profile.stream_index()}) == streams.end())
////                        {
////                            streams[stream_and_index{profile.stream_type(), profile.stream_index()}] = profile;
////                        }
////
////                        rs2_intrinsics intrinsics{};
////                        stream_and_resolution stream_res{profile.stream_type(), profile.stream_index(), video.width(), video.height(), profile.stream_name()};
////                        if (safe_get_intrinsics(video, intrinsics))
////                        {
////                            auto it = std::find_if((intrinsics_map[stream_res]).begin(), (intrinsics_map[stream_res]).end(), [&](const std::pair<std::set<rs2_format>, rs2_intrinsics>& kvp) {
////                                return intrinsics == kvp.second;
////                            });
////                            if (it == (intrinsics_map[stream_res]).end())
////                            {
////                                (intrinsics_map[stream_res]).push_back({ {profile.format()}, intrinsics });
////                            }
////                            else
////                            {
////                                it->first.insert(profile.format()); // If the intrinsics are equals, add the profile format to format set
////                            }
////                        }
////                    }
////                    else
////                    {
////                        if (rs2::motion_stream_profile motion = profile.as<rs2::motion_stream_profile>())
////                        {
////                            if (streams.find(stream_and_index{profile.stream_type(), profile.stream_index()}) == streams.end())
////                            {
////                                streams[stream_and_index{profile.stream_type(), profile.stream_index()}] = profile;
////                            }
////
////                            rs2_motion_device_intrinsic motion_intrinsics{};
////                            stream_and_resolution stream_res{profile.stream_type(), profile.stream_index(), motion.stream_type(), motion.stream_index(), profile.stream_name()};
////                            if (safe_get_motion_intrinsics(motion, motion_intrinsics))
////                            {
////                                auto it = std::find_if((motion_intrinsics_map[stream_res]).begin(), (motion_intrinsics_map[stream_res]).end(),
////                                                       [&](const std::pair<std::set<rs2_format>, rs2_motion_device_intrinsic>& kvp)
////                                                       {
////                                                           return motion_intrinsics == kvp.second;
////                                                       });
////                                if (it == (motion_intrinsics_map[stream_res]).end())
////                                {
////                                    (motion_intrinsics_map[stream_res]).push_back({ {profile.format()}, motion_intrinsics });
////                                }
////                                else
////                                {
////                                    it->first.insert(profile.format()); // If the intrinsics are equals, add the profile format to format set
////                                }
////                            }
////                        }
////                    }
////                }
//            }
//
//            cout << "Provided Intrinsic:" << endl;
//            for (auto& kvp : intrinsics_map)
//            {
//                auto stream_res = kvp.first;
//                for (auto& intrinsics : kvp.second)
//                {
//                    auto formats = get_str_formats(intrinsics.first);
//                    cout << "Intrinsic of \"" << stream_res.stream_name << "\"\t  " << stream_res.width << "x"
//                    << stream_res.height << "\t  " << formats << endl;
//                    if (intrinsics.second == rs2_intrinsics{})
//                    {
//                        cout << "Intrinsic NOT available!\n\n";
//                    }
//                    else
//                    {
//                        print(intrinsics.second);
//                    }
//                }
//            }
//
//            cout << "Provided Motion Intrinsic:" << endl;
//            for (auto& kvp : motion_intrinsics_map)
//            {
//                auto stream_res = kvp.first;
//                for (auto& intrinsics : kvp.second)
//                {
//                    auto formats = get_str_formats(intrinsics.first);
//                    cout << "Motion Intrinsic of \"" << stream_res.stream_name << "\"\t  " << formats << endl;
//                    if (intrinsics.second == rs2_motion_device_intrinsic{})
//                    {
//                        cout << "Intrinsic NOT available!\n\n";
//                    }
//                    else
//                    {
//                        print(intrinsics.second);
//                    }
//                }
//            }
//
//            // Print Extrinsics
//            cout << "\nProvided Extrinsic:" << endl;
//            rs2_extrinsics extrinsics{};
//            for (auto kvp1 = streams.begin(); kvp1 != streams.end(); ++kvp1)
//            {
//                for (auto kvp2 = streams.begin(); kvp2 != streams.end(); ++kvp2)
//                {
//                    cout << "Extrinsic from \"" << kvp1->second.stream_name() << "\"\t  " <<
//                    "To" << "\t  \"" << kvp2->second.stream_name() << "\" :\n";
//                    try
//                    {
//                        extrinsics = kvp1->second.get_extrinsics_to(kvp2->second);
//                        print(extrinsics);
//                    }
//                    catch (...)
//                    {
//                        cout << "N/A\n";
//                    }
//                }
//            }
//        }
//    }
//
//    cout << endl;
    
	// sort devices by serial number
	sort(deviceList.begin(), deviceList.end(), sortRealSensePairs);
}

void ofxRealSenseContext::listDevices(bool verbose) {
  if(!isInited())
	init();

	stringstream stream;

	if(numTotal() == 0) {
		stream << "no devices found";
		return;
	}
	else if(numTotal() == 1) {
		stream << 1 << " device found";
	}
	else {
		stream << deviceList.size() << " devices found";
	}

	if(verbose) {
		ofLogVerbose("ofxRealSense2") << stream.str();
	}
	else {
		ofLogNotice("ofxRealSense2") << stream.str();
	}
	stream.str("");

	for(size_t i = 0; i < deviceList.size(); ++i) {
		stream << "    id: " << deviceList[i].id << " serial: " << deviceList[i].serial;
		if(verbose) {
			ofLogVerbose("ofxRealSense2") << stream.str();
		}
		else {
			ofLogNotice("ofxRealSense2") << stream.str();
		}
		stream.str("");
	}
}

int ofxRealSenseContext::numTotal() {
    if(!isInited())
		init();
    // need an analogue
//    return freenect_num_devices(realSenseContext);
    return deviceList.size();
}

int ofxRealSenseContext::numAvailable() {
	if(!isInited())
		init();
    // need an analogue
//    return freenect_num_devices(realSenseContext) - realSenses.size();
    return deviceList.size();
}

int ofxRealSenseContext::numConnected() {
	return realSenses.size();
}

ofxRealSense2* ofxRealSenseContext::getRealSense(rs2::device* dev) {
	std::map<int,ofxRealSense2*>::iterator iter;
	for(iter = realSenses.begin(); iter != realSenses.end(); ++iter) {
		if(iter->second->realSenseDevice == dev)
			return iter->second;
	}
	return NULL;
}

int ofxRealSenseContext::getDeviceIndex(int id) {
	for(size_t i = 0; i < deviceList.size(); ++i) {
		if(deviceList[i].id == id)
			return i;
	}
	return -1;
}

int ofxRealSenseContext::getDeviceIndex(string serial) {
	for(size_t i = 0; i < deviceList.size(); ++i) {
		if(deviceList[i].serial == serial)
			return i;
	}
	return -1;
}


int ofxRealSenseContext::getDeviceId(unsigned int index) {
    if( index >= 0 && index < deviceList.size() ){
        return deviceList[index].id;
    }
	return -1;
}

int ofxRealSenseContext::getDeviceId(string serial){
	for(size_t i = 0; i < deviceList.size(); ++i) {
		if(deviceList[i].serial == serial){
			return deviceList[i].id;
        }
	}
	return -1;
}

bool ofxRealSenseContext::isConnected(int id) {
	std::map<int,ofxRealSense2*>::iterator iter = realSenses.find(id);
	return iter != realSenses.end();
}

bool ofxRealSenseContext::isConnected(string serial) {
	std::map<int,ofxRealSense2*>::iterator iter;
	for(iter = realSenses.begin(); iter != realSenses.end(); ++iter) {
		if(iter->second->getSerial() == serial)
			return true;
	}
	return false;
}

int ofxRealSenseContext::nextAvailableId() {
	if(!isInited())
		init();

	// a brute force free index finder :D
	std::map<int,ofxRealSense2*>::iterator iter;
	for(size_t i = 0; i < deviceList.size(); ++i) {
		iter = realSenses.find(deviceList[i].id);
		if(iter == realSenses.end())
			return deviceList[i].id;
	}
	return -1;
}

string ofxRealSenseContext::nextAvailableSerial() {
	if(!isInited())
		init();

	int id = nextAvailableId();
	if(id == -1) {
		return "";
	}
	return deviceList[getDeviceIndex(id)].serial;
}


