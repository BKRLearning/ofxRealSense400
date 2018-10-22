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

    ----------------------------------------------------------------------------

    This project uses libfreenect, copyrighted by the Open Kinect Project using
    the Apache License v2. See the file "APACHE20" in libs/libfreenect.

    See http://www.openkinect.org & https://github.com/OpenKinect/libfreenect
    for documentation

==============================================================================*/
#pragma once

#include "ofMain.h"
#include "rs.h"
#include "rs.hpp"

#define COLOR_WIDTH 1280
#define COLOR_HEIGHT 720
#define DEPTH_WIDTH 1280
#define DEPTH_HEIGHT 720

#if defined(_MSC_VER) || defined(_WIN32) || defined(WIN32) || defined(__MINGW32__)
    // do windows stuff
#else
    // mac and linux need this
    #include <libusb.h>
#endif


#include "ofxBase3DVideo.h"

class ofxRealSenseContext;

/// \class ofxKinect
///
/// wrapper for a freenect kinect device
///
/// references:
/// - http://openkinect.org/wiki/Main_Page
/// - https://github.com/OpenKinect/libfreenect/blob/master/include/libfreenect.h
///
class ofxRealSense2 {
    public:
        ~ofxRealSense2();
        ofxRealSense2();
        bool setup();
        void update();


        ofPixels & getPixels();
        ofPixels & getDepthPixels();
        ofPixels & getIRPixels();

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

//    private:
        rs2::colorizer color_map;
        rs2::config config;
        rs2::pipeline pipe;
    
        rs2::frame depth;

        ofImage colorImage;
        ofImage depthImage;
        ofImage infraredImage;
    
        ofPixels videoPixels;
        ofPixels depthPixels;
        ofPixels infraredPixels;
        ofShortPixels depthPixelsRaw;
        ofFloatPixels distancePixels;
    
        ofShortPixels depthPixelsRawIntra;    ///< depth back
        ofPixels videoPixelsIntra;            ///< rgb back
        ofShortPixels depthPixelsRawBack;    ///< depth back
        ofPixels videoPixelsBack;            ///< rgb back
    
        ofTexture depthTex; ///< the depth texture
        ofTexture videoTex; ///< the RGB texture
        ofTexture infraredTex;
    
        int width;
        int height;
};
