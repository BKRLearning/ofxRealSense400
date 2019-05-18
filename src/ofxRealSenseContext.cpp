#include "ofxRealSenseContext.h"

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

    // freenect_shutdown(realSenseContext);
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
    realSense.config.enable_device(serial);
    realSense.profile = realSense.pipe.start(realSense.config);

    realSenses.insert(pair<int,ofxRealSense2*>(deviceList[index].id, &realSense));
    realSense.deviceId = deviceList[index].id;
    realSense.serial = serial;

    return true;
}

bool ofxRealSenseContext::openFromFile(ofxRealSense2& realSense, string filename) {

    cout << "IN CONTEXT TRYING TO OPEN DEVICE FROM FILE: " << filename << endl;

    // realSense.config.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, 30);
    realSense.config.enable_device_from_file(filename);
    realSense.profile = realSense.pipe.start(realSense.config); //File will be opened in read mode at this point
    auto device = realSense.pipe.get_active_profile().get_device();
    device.as<rs2::playback>().resume();

    int idNum = nextAvailableId() + 1; //The file 'device' won't be in the hardware list so we take the next-next Id
    realSenses.insert(pair<int,ofxRealSense2*>(idNum, &realSense));
    realSense.deviceId = idNum;
    realSense.serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

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
        return;
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

vector<string> ofxRealSenseContext::getAvailableSerials() {
    vector<string> serials;
    for (size_t i = 0; i < deviceList.size(); ++i) {
        if (!isConnected(deviceList[i].serial)) {
            serials.push_back(deviceList[i].serial);
        }
    }
    return serials;
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

ofxRealSense2* ofxRealSenseContext::getRealSense(string serial) {
    std::map<int,ofxRealSense2*>::iterator iter;
    for (iter = realSenses.begin(); iter != realSenses.end(); ++iter) {
        if(iter->second->getSerial() == serial)
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

map<string, bool> ofxRealSenseContext::getDeviceStatusMap(map<string, string> serialMap) {
    auto devices = realSenseContext->query_devices();
    size_t deviceCount = devices.size();

    map<string, bool> statusMap;
    for (auto&& serial : serialMap) {
        bool bFound = false;
        for (auto device : devices) {
            try {
                string deviceSerial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                ofxRealSense2* rsPtr = getRealSense(serial.second);
                if (rsPtr != NULL && rsPtr->isConnected() && serial.second == device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) {
                    bFound = true;
                }
            } catch (const std::exception& e) {
                ofLogWarning("ofxRealSenseContext") << e.what();
            }
        }
        statusMap.emplace(serial.first, bFound);
    }
    return statusMap;
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
