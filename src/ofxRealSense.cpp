/*==============================================================================

    Copyright (c) 2010, 2011 ofxKinect Team

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
#include "ofxRealSense.h"
#include "ofMain.h"
//
//#include "ofxRealSense2.h"
//#include "ofxRealSense2Utils.h"


ofxRealSense2::ofxRealSense2(){
}

ofxRealSense2::~ofxRealSense2(){
}

bool ofxRealSense2::setup(){
    config.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_RGB8, 30);
//    config.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_ANY, 30);
    config.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_INFRARED, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Y8, 30);
    pipe.start(config);
    colorImage.allocate(COLOR_WIDTH, COLOR_HEIGHT, OF_IMAGE_COLOR);
//    depthImage.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);

    depthImage.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);
    infraredImage.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
    
    // allocate
    depthPixelsRaw.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, 1);
    depthPixelsRawBack.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, 1);
    depthPixelsRawIntra.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, 1);
    
    videoPixels.allocate(COLOR_WIDTH, COLOR_HEIGHT, 3);
    videoPixelsBack.allocate(COLOR_WIDTH, COLOR_HEIGHT, 3);
    videoPixelsIntra.allocate(COLOR_WIDTH, COLOR_HEIGHT, 3);
    
    depthPixels.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);
    distancePixels.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, 1);
    
    infraredPixels.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, 1);
    
    // set
    depthPixelsRaw.set(0);
    depthPixelsRawBack.set(0);
    
    videoPixels.set(0);
    videoPixelsBack.set(0);
    
    depthPixels.set(0);
    distancePixels.set(0);
    
    depthTex.allocate(depthPixels);
    videoTex.allocate(videoPixels);
    
    width = DEPTH_WIDTH;
    height = DEPTH_HEIGHT;
    
}

void ofxRealSense2::update(){
    rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
    color_map.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 1.f);
    color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
    rs2::frame depth = color_map(data.get_depth_frame()); // Find and colorize the depth data
//    depth = color_map(data.get_depth_frame()); // Find the depth data

    rs2::frame color = data.get_color_frame();            // Find the color data
    rs2::frame infrared = data.get_infrared_frame();
//    rs2::frame depth = data.get_depth_frame(); // Find and colorize the depth data


    videoPixels.setFromPixels((unsigned char *)color.get_data(), COLOR_WIDTH, COLOR_HEIGHT, OF_IMAGE_COLOR);
    depthPixels.setFromPixels((unsigned char *)depth.get_data(), DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);
    infraredPixels.setFromPixels((unsigned char *)infrared.get_data(), DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
    
    colorImage.setFromPixels((unsigned char *)color.get_data(), COLOR_WIDTH, COLOR_HEIGHT, OF_IMAGE_COLOR);
//    depthImage.setFromPixels((unsigned char *)depth.get_data(), DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_COLOR);

    depthImage.setFromPixels((unsigned char *)depth.get_data(), DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
    infraredImage.setFromPixels((unsigned char *)infrared.get_data(), DEPTH_WIDTH, DEPTH_HEIGHT, OF_IMAGE_GRAYSCALE);
    
    videoTex.loadData(videoPixels);
    depthTex.loadData(depthPixels);
    infraredTex.loadData(infraredPixels);

}


//----------------------------------------------------------
void ofxRealSense2::draw(float _x, float _y, float _w, float _h) const{
//    colorImage.draw(_x, _y, _w, _h);
    videoTex.draw(_x, _y, _w, _h);
}

//----------------------------------------------------------
void ofxRealSense2::draw(float _x, float _y) const{
    draw(_x, _y, (float)COLOR_WIDTH, (float)COLOR_HEIGHT);
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
//    depthImage.draw(_x, _y, _w, _h);
    depthTex.draw(_x, _y, _w, _h);
}

//---------------------------------------------------------------------------
void ofxRealSense2::drawDepth(float _x, float _y) const{
    drawDepth(_x, _y, (float)DEPTH_WIDTH, (float)DEPTH_HEIGHT);
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
//    infraredImage.draw(_x, _y, _w, _h);
    infraredTex.draw(_x, _y, _w, _h);

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

ofPixels & ofxRealSense2::getPixels(){
    return videoPixels;
//    return colorImage.getPixels();
}

ofPixels & ofxRealSense2::getDepthPixels(){
    return depthPixels;
//    return depthImage.getPixels();
}

ofPixels & ofxRealSense2::getIRPixels(){
    return infraredPixels;
//    return infraredImage.getPixels();
}
