ofxRealSense400
=========

Copyright (c) 2018 Brooklyn Research

MIT License.

For information on usage and redistribution, and for a DISCLAIMER OF ALL
WARRANTIES, see the file, "LICENSE.txt," in this distribution.

See https://github.com/BKRLearning/ofxRealSense400 for documentation as well as the [OF forums](http://forum.openframeworks.cc/index.php).

This project uses [librealsense](https://github.com/IntelRealSense/librealsense), copyrighted by Intel using the Apache License v2. See the file "APACHE20" in libs/librealsense. This addon is modeled after the ofxKinect addon included with OpenFrameworks.

Description
-----------

ofxRealSense400 is an Open Frameworks addon for the Intel RealSense 400 series of its depth sensor that runs on Mac OSX, Linux, and Windows.
OpenFrameworks is a cross platform open source toolkit for creative coding in C++.

[http://www.openframeworks.cc/](http://www.openframeworks.cc/)

Running the Example Project
---------------------------

An example project is provided in the `example-base`. If you've downloaded/cloned OF from Github, use the OpenFrameworks ProjectGenerator in `apps/projectGenerator` to generate the Xcode, VS2012, CodeBlocks projects and/or Makefiles by pointing it to the `d415Example` folder and making sure to include the following addons:

* ofxRealSense400
* ofxOpenCv (for blob tracking in the example, not required by ofxRealSense400 itself.)

### OSX

Open the Xcode project, select the "realSenseExample Debug" scheme, and hit "Run".

### Linux

Install the libusb-1.0 library. On Ubuntu, you can do this with:
<pre>
sudo apt-get install libusb-1.0-0-dev
</pre>

Open the Code::Blocks .cbp and hit F9 to build. Optionally, you can build the example with the Makefile.

To run it, use the terminal:
<pre>
make
cd bin
./example_debug
</pre>


### Windows

Precompiled libusb libs are included for Windows.

Install instructions:


#### Windows (Msys2):

Install libusb using

    pacman --needed -Sy mingw-w64-i686-libusb



How to Create a New ofxRealSense400 Project
-------------------------------------

As of OF 0.8.0, you can also create a new ofxRealSense400 project using the ProjectGenerator, found in `openFrameworks/apps/projectGenerator`.

Adding ofxRealSense400 to an Existing Project
---------------------------------------

Notes
-----

### Using multiple RealSense sensors

ofxRealSense400 supports multiple RealSense sensors, however stability is based on the bandwidth of your usb hardware. If you only require the depth image, it is recommended to disable the video grabbing of the rgb/ir images from your devices:
<pre>
realsense.init(false, false);  // disable video image (faster fps)
</pre>


Developing ofxRealSense400
--------------------

Feel free to log bug reports and issues to the openFrameworks Github page:
https://github.com/BKRLearning/ofxRealSense400

Current Status
--------------------

* Using librealsense v2.16.4
* Must call RealSense2::open() with explicit serial number string argument to use more than one device
* Automatic indexing of devices by ID number does not work when using multiple devices
* Devices tested:
  * D415 -- firmware: 05.10.03.00
  * D435 -- firmware: 05.09.02.00

Changes in Post-Proc Branch
--------------------

* Using colormap options to set higher resolution clipping clipping depths
* Filter class to encapsulate an rs2 processing block
* Separate thread with frame queues for running filters on depth data
* Filters currently enabled:
  * Decimation
  * Spatial
  * Temporal
  * Hole-Filling

Before Merge
--------------------
* Frame queues for rgb and infrared streams
* Filter class methods for changing params
