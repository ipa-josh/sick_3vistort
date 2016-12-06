/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2015 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *   ROS package name: sick_visionary_t_driver
 *
 * \author
 *   Author: Joshua Hampp
 *
 * \date Date of creation: 05/21/2015
 *
 * \brief
 *   Data and parameter parsing of stream
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
 
#include <endian.h>

#if __BYTE_ORDER == __BIG_ENDIAN
uint64_t swapLong(uint64_t x) {
    x = (x & 0x00000000FFFFFFFF) << 32 | (x & 0xFFFFFFFF00000000) >> 32;
    x = (x & 0x0000FFFF0000FFFF) << 16 | (x & 0xFFFF0000FFFF0000) >> 16;
    x = (x & 0x00FF00FF00FF00FF) << 8  | (x & 0xFF00FF00FF00FF00) >> 8;
    return x;
}

#define __Swap2Bytes(val)   ( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) )
#define __Swap2BytesD(val)  (val>7490?0:( (((val) >> 8) & 0x00FF) | (((val) << 8) & 0xFF00) ))
#define __Swap4Bytes(dword) ( ((dword>>24)&0x000000FF) | ((dword>>8)&0x0000FF00) | ((dword<<8)&0x00FF0000) | ((dword<<24)&0xFF000000) )
#define __Swap8Bytes(val) (swapLong(val))
#else
#define __Swap2Bytes(val) (val)
#define __Swap2BytesD(val) (val>7490?0:val)
#define __Swap4Bytes(dword) (dword)
#define __Swap8Bytes(dword) (dword)
#endif


/* This class gathers the main camera parameters. */
struct CameraParameters {
    uint32_t width, height;
    double cam2worldMatrix[4*4];
    double fx, fy, cx, cy, k1, k2, f2rc;
    
    CameraParameters(uint32_t width=176, uint32_t height=144,
                 double *cam2worldMatrix=NULL,
                 double fx=146.5, double fy=146.5,double cx=84.4,double cy=71.2,
                 double k1=0.326442, double k2=0.219623,
                 double f2rc=0.0):
        width(width), height(height),
        fx(fx), fy(fy),
        cx(cx), cy(cy),
        k1(k1), k2(k2),
        f2rc(f2rc)
                  {
        if(cam2worldMatrix)
            memcpy(this->cam2worldMatrix, cam2worldMatrix, sizeof(this->cam2worldMatrix));
    }
};

/* This class contains information for Cartesian case. */
struct PointXYZC {
    float x;
    float y;
    float z;
    float c;
};


/* Gathers methods to handle the raw data. */
class Data {
    //flags
    bool hasDepthData_, hasPolarData_, hasHeightData_;
    
    //depth map
    CameraParameters cameraParams_;
    cv::Mat distance_, intensity_, confidence_;
    int numBytesPerDistanceValue_, numBytesPerIntensityValue_, numBytesPerConfidenceValue_;
    uint32_t framenumber_;
    
    //polar scan
    int numPolarValues_;
    std::vector<float> distances_, confidences_;
    float startingAngle_, stepAngle_;
    
    //height map
    std::vector<PointXYZC> points_;
    
    //consts.
    const float MAX_CONFIDENCE;
public:

    Data() : framenumber_(0), numPolarValues_(0), MAX_CONFIDENCE(65535) {
    }

    //depth map
    bool has_depth_data() const {return hasDepthData_;}
    const cv::Mat &get_distance() const {return distance_;}
    const cv::Mat &get_intensity() const {return intensity_;}
    const cv::Mat &get_confidence() const {return confidence_;}
    uint32_t get_framenumber() const { return framenumber_; }
    
    //polar
    bool has_polar_data() const {return hasPolarData_;}
    const std::vector<float> &get_polar_distances() const {return distances_;}
    const std::vector<float> &get_polar_confidences() const {return confidences_;}
    float get_polar_startingAngle() const {return startingAngle_;}
    float get_polar_stepAngle() const {return stepAngle_;}
    
    //height
    bool has_height_data() const {return hasHeightData_;}
    const std::vector<PointXYZC> &get_height_points() const {return points_;}
    
    const CameraParameters &getCameraParameters() const {return cameraParams_;}

    /* Get size of complete package. */
    static size_t actual_size(const char *data, const size_t size) {
    if(size<8) return 0;
        // first 11 bytes contain some internal definitions   
    const uint32_t pkglength = ntohl( *(uint32_t*)(data+4) );

    return pkglength+9;
    }

    /* Check if there are enough data to be parsed. */
    static bool check_header(const char *data, const size_t size) {
    if(size<11) return false;

    const uint32_t magicword = ntohl( *(uint32_t*)(data+0) );

    //if magic word in invalid we'll read the content anyway to skip data
    return magicword != 0x02020202 || size>=actual_size(data, size);
    }

    /* Extracts necessary data segments and triggers parsing of segments. */
    bool read(const char *data, const size_t size) {
        // first 11 bytes contain some internal definitions   
        const uint32_t magicword = ntohl( *(uint32_t*)(data+0) );
        const uint32_t pkglength = ntohl( *(uint32_t*)(data+4) );
        const uint16_t protocolVersion = ntohs( *(uint16_t*)(data+8) );
        const uint8_t packetType = *(uint8_t*)(data+10);
        
        ROS_ASSERT( magicword == 0x02020202 );
        ROS_ASSERT( protocolVersion == 1 );
        ROS_ASSERT( packetType == 98 );
        
        if( magicword != 0x02020202 || protocolVersion != 1 || packetType != 98 )
            return false;
        
        // next four bytes an id (should equal 1) and
        // the number of segments (should be 3)       
        const uint16_t id = ntohs( *(uint16_t*)(data+11) );
        const uint16_t numSegments = ntohs( *(uint16_t*)(data+13) );
        ROS_ASSERT( id == 1 );
        ROS_ASSERT( numSegments == 3 );
        
        // offset and changedCounter, 4 bytes each per segment
        uint32_t offset[numSegments];
        uint32_t changedCounter[numSegments];
        for(size_t i=0; i<(size_t)numSegments; i++) {
            offset[i] = 11 + ntohl( *(uint32_t*)(data+(i*8+15)) );
            changedCounter[i] = ntohl( *(uint32_t*)(data+(i*8+19)) );
        }
        
        // XML segment describes the data format
        std::string xmlSegment(data+offset[0], offset[1]-offset[0]);
        
        // parsing the XML in order to extract necessary image information
        hasDepthData_ = hasPolarData_ = hasHeightData_ = false;
        parseXML(xmlSegment);
                
        if(hasDepthData_) {
            /*self.cameraParams = \
                    CameraParameters(width=myXMLParser.imageWidth,
                                     height=myXMLParser.imageHeight,
                                     cam2worldMatrix=myXMLParser.cam2worldMatrix,
                                     fx=myXMLParser.fx,fy=myXMLParser.fy,
                                     cx=myXMLParser.cx,cy=myXMLParser.cy,
                                     k1=myXMLParser.k1,k2=myXMLParser.k2,
                                     f2rc=myXMLParser.f2rc)*/

            // extracting data from the binary segment (distance, intensity
            // and confidence).
            ROS_ASSERT(numBytesPerDistanceValue_==2);
            ROS_ASSERT(numBytesPerIntensityValue_==2);
            ROS_ASSERT(numBytesPerConfidenceValue_==2);
            
            distance_   = cv::Mat(cameraParams_.height, cameraParams_.width, CV_16UC1);
            intensity_  = cv::Mat(cameraParams_.height, cameraParams_.width, CV_16UC1);
            confidence_ = cv::Mat(cameraParams_.height, cameraParams_.width, CV_16UC1);
            
            offset[1] += getDataDepth(data+offset[1], size-offset[1]);
        }
        
        if(hasPolarData_) {
            offset[1] += getDataPolar(data+offset[1], size-offset[1]);
        }
        
        if(hasHeightData_) {
            offset[1] += getDataHeight(data+offset[1], size-offset[1]);
        }
        
        return true;
    }
    
private:
    /* Parse method needs the XML segment as string input. */
    bool parseXML(const std::string &xmlString) {
        numBytesPerConfidenceValue_ = numBytesPerIntensityValue_ = numBytesPerConfidenceValue_ = -1;
        
        boost::property_tree::ptree pt;
        std::istringstream ss(xmlString);
        try {
            boost::property_tree::xml_parser::read_xml(ss, pt);
        } catch(...) {
            ROS_ERROR("failed to parse response (XML malformed)\ncontent: %s", xmlString.c_str());
            return false;
        }
        
        if(pt.get_child_optional("SickRecord.DataSets.DataSetDepthMap")) {
            //std::cout<<"status depth "<< pt.get_child("SickRecord.DataSets.DataSetDepthMap.FormatDescriptionDepthMap.DeviceInfo.Status").data() <<std::endl;
            
            boost::property_tree::ptree ds = pt.get_child("SickRecord.DataSets.DataSetDepthMap.FormatDescriptionDepthMap.DataStream");
            cameraParams_.width = ds.get<uint32_t>("Width");
            cameraParams_.height = ds.get<uint32_t>("Height");
            
            int i=0;
            BOOST_FOREACH(const boost::property_tree::ptree::value_type &item, ds.get_child("CameraToWorldTransform")) {
                if(i<16)
                    cameraParams_.cam2worldMatrix[i] = item.second.get_value<double>();
                ++i;
            }
            ROS_ASSERT(i==16);
            
            cameraParams_.fx = ds.get<double>("CameraMatrix.FX");
            cameraParams_.fy = ds.get<double>("CameraMatrix.FY");
            cameraParams_.cx = ds.get<double>("CameraMatrix.CX");
            cameraParams_.cy = ds.get<double>("CameraMatrix.CY");
            
            cameraParams_.k1 = ds.get<double>("CameraDistortionParams.K1");
            cameraParams_.k2 = ds.get<double>("CameraDistortionParams.K2");
            
            cameraParams_.f2rc = ds.get<double>("FocalToRayCross");
        
            // extract data format from XML (although it does not change)
            std::string data_type;
            
            data_type=ds.get<std::string>("Distance", "");
            boost::algorithm::to_lower(data_type);
            if(data_type != "") {
                ROS_ASSERT(data_type == "uint16");
                numBytesPerDistanceValue_ = 2;
            }
            
            data_type=ds.get<std::string>("Intensity", "");
            boost::algorithm::to_lower(data_type);
            if(data_type != "") {
                ROS_ASSERT(data_type == "uint16");
                numBytesPerIntensityValue_ = 2;
            }
            
            data_type=ds.get<std::string>("Confidence", "");
            boost::algorithm::to_lower(data_type);
            if(data_type != "") {
                ROS_ASSERT(data_type == "uint16");
                numBytesPerConfidenceValue_ = 2;
            }
            
            hasDepthData_ = true;
        }
        
        if(pt.get_child_optional("SickRecord.DataSets.DataSetPolar2D")) {
            BOOST_FOREACH(const boost::property_tree::ptree::value_type &item, pt.get_child("SickRecord.DataSets.DataSetPolar2D.FormatDescription")) {
                if(item.first=="DataStream" && item.second.get<std::string>("<xmlattr>.type","")=="distance") {
                    numPolarValues_ = item.second.get<int>("<xmlattr>.datalength");
                    hasPolarData_ = true;
                }
            }
        }
        
        if(pt.get_child_optional("SickRecord.DataSets.DataSetCartesian")) {
            boost::property_tree::ptree ds = pt.get_child("SickRecord.DataSets.DataSetCartesian.FormatDescriptionCartesian.DataStream");
            // extract data format from XML (although it does not change)
            std::string data_type;
            
            hasHeightData_ = true;
            
            data_type=ds.get<std::string>("Length", "");
            boost::algorithm::to_lower(data_type);
            if(data_type != "") {
                ROS_ASSERT(data_type == "uint32");
                hasHeightData_ &= (data_type == "uint32");
            }
            
            data_type=ds.get<std::string>("X", "");
            boost::algorithm::to_lower(data_type);
            if(data_type != "") {
                ROS_ASSERT(data_type == "float32");
                hasHeightData_ &= (data_type == "float32");
            }
            
            data_type=ds.get<std::string>("Y", "");
            boost::algorithm::to_lower(data_type);
            if(data_type != "") {
                ROS_ASSERT(data_type == "float32");
                hasHeightData_ &= (data_type == "float32");
            }
            
            data_type=ds.get<std::string>("Z", "");
            boost::algorithm::to_lower(data_type);
            if(data_type != "") {
                ROS_ASSERT(data_type == "float32");
                hasHeightData_ &= (data_type == "float32");
            }
        }
        
        return true;
    }
    
    uint32_t getDataPolar(const char *data, const size_t size) {
        ROS_ASSERT(size>=46);
        if(size<46) {
            ROS_WARN("malformed data (1): %d<46", (int)size);
            return 0;
        }
        
        size_t offset = 0;
        const uint32_t length = __Swap4Bytes( *(uint32_t*)(data+offset) );
        const uint64_t timeStamp = __Swap8Bytes( *(uint64_t*)(data+offset+4) );
        const uint16_t deviceID = __Swap2Bytes( *(uint16_t*)(data+offset+12) );
        const uint32_t scanCounter = __Swap4Bytes( *(uint32_t*)(data+offset+14) );
        const uint32_t syscountScan = __Swap4Bytes( *(uint32_t*)(data+offset+18) );
        
        const float scanFrequency = ( *(float*)(data+offset+22) );
        const float measFrequency = ( *(float*)(data+offset+26) );
        const float angleFirstScanPoint = ( *(float*)(data+offset+30) );
        const float angularResolution = ( *(float*)(data+offset+34) );
        const float scale = ( *(float*)(data+offset+38) );
        const float offsetF = ( *(float*)(data+offset+42) );
        
        offset += 46;

        if(length>size) {
            ROS_WARN("malformed data (2): %d>%d", (int)length, (int)size);
            return length;
        }
         
        /*ROS_DEBUG_STREAM("length " << length);
        ROS_DEBUG_STREAM("timeStamp " << timeStamp);
        ROS_DEBUG_STREAM("deviceID " << deviceID);
        ROS_DEBUG_STREAM("scanCounter " << scanCounter);
        ROS_DEBUG_STREAM("scanCounter " << scanCounter);
        ROS_DEBUG_STREAM("syscountScan " << syscountScan);
        ROS_DEBUG_STREAM("scanFrequency " << scanFrequency);
        ROS_DEBUG_STREAM("measFrequency " << measFrequency);
        ROS_DEBUG_STREAM("angleFirstScanPoint " << angleFirstScanPoint);
        ROS_DEBUG_STREAM("angularResolution " << angularResolution);
        ROS_DEBUG_STREAM("scale " << scale);
        ROS_DEBUG_STREAM("offset " << offsetF);
        ROS_DEBUG_STREAM("numPolarValues " << numPolarValues_);*/
        
        size_t end = offset + 4*numPolarValues_; // calculating the end index
        ROS_ASSERT(end<=size);

        startingAngle_=angleFirstScanPoint;
        stepAngle_=angularResolution;
        distances_.resize(numPolarValues_);
        for(int i=0; i<numPolarValues_; i++)
            distances_[i] = ( *(float*)(data+offset+4*i) ) / 1000.;
            
        offset += 4*numPolarValues_;
        
        end = offset + 4*(numPolarValues_+4); // calculating the end index
        ROS_ASSERT(end<=size);

        const float rssi_startAngle = ( *(float*)(data+offset+0) );
        const float rssi_angularResolution = ( *(float*)(data+offset+4) );
        const float rssi_scale = ( *(float*)(data+offset+8) );
        const float rssi_offset = ( *(float*)(data+offset+12) );
        offset += 4*4;
        
        //lets assume it matches the angles
        /*ROS_DEBUG_STREAM("rssi_startAngle " << rssi_startAngle);
        ROS_DEBUG_STREAM("rssi_angularResolution " << rssi_angularResolution);
        ROS_DEBUG_STREAM("rssi_scale " << rssi_scale);
        ROS_DEBUG_STREAM("rssi_offset " << rssi_offset);*/
        
        confidences_.resize(numPolarValues_);
        for(int i=0; i<numPolarValues_; i++)
            confidences_[i] = ( *(float*)(data+offset+4*i) ) / MAX_CONFIDENCE * 100.;
            
        offset += 4*numPolarValues_;

        if(offset!=length) {    //otherwise another binary block
            // data end with a 32byte (unused) CRC field and a copy of the length byte
            const uint32_t unusedCrc = __Swap4Bytes( *(uint32_t*)(data+offset) );
            const uint32_t lengthCopy = __Swap4Bytes( *(uint32_t*)(data+offset + 4) );

            if(length != lengthCopy)
                ROS_WARN("check failed %d!=%d", (int)length, (int)lengthCopy);
            ROS_ASSERT(length == lengthCopy);
        }
            
        return length;
    }
    
    uint32_t getDataHeight(const char *data, const size_t size) {
        ROS_ASSERT(size>=18);
        if(size<18) {
            ROS_WARN("malformed data (1): %d<18", (int)size);
            return 0;
        }

        // the binary part starts with entries for length, a timestamp
        // and a version identifier
        size_t offset = 0;
        const uint32_t length = __Swap4Bytes( *(uint32_t*)(data+offset) );
        const uint64_t timeStamp = __Swap8Bytes( *(uint64_t*)(data+offset+4) );
        const uint16_t version = __Swap2Bytes( *(uint16_t*)(data+offset+12) );
        const uint32_t numPoints = __Swap4Bytes( *(uint32_t*)(data+offset+14) );
        
        offset += 18;

        if(length>size) {
            ROS_WARN("malformed data (2): %d>%d", (int)length, (int)size);
            return length;
        }
         
        /*ROS_DEBUG_STREAM("length " << length);
        ROS_DEBUG_STREAM("timeStamp " << timeStamp);
        ROS_DEBUG_STREAM("version " << version);
        ROS_DEBUG_STREAM("numPoints " << numPoints);*/
        
        size_t end = offset + 16*numPoints; // calculating the end index
        ROS_ASSERT(end<=size);
        
        ROS_DEBUG_STREAM("end " << end);

        points_.resize(numPoints);
        for(int i=0; i<numPoints; i++) {
            points_[i].x = ( *(float*)(data+offset+16*i +  0) ) / 1000.;
            points_[i].y = ( *(float*)(data+offset+16*i +  4) ) / 1000.;
            points_[i].z = ( *(float*)(data+offset+16*i +  8) ) / 1000.;
            points_[i].c = ( *(float*)(data+offset+16*i + 12) ) / MAX_CONFIDENCE * 100.;
        }
        
        offset += 16*numPoints;
        
        // data end with a 32byte (unused) CRC field and a copy of the length byte
        const uint32_t unusedCrc = __Swap4Bytes( *(uint32_t*)(data+offset) );
        const uint32_t lengthCopy = __Swap4Bytes( *(uint32_t*)(data+offset + 4) );

        if(length != lengthCopy)
            ROS_WARN("check failed %d!=%d", (int)length, (int)lengthCopy);
        ROS_ASSERT(length == lengthCopy);
            
        return length;
    }
    
    uint32_t getDataDepth(const char *data, const size_t size) {
        ROS_ASSERT(size>=14);
        if(size<14) {
            ROS_WARN("malformed data (1): %d<14", (int)size);
            return 0;
        }
        const size_t numBytesDistance   = (numBytesPerDistanceValue_ > 0) ? distance_.total()*numBytesPerDistanceValue_ : 0;
        const size_t numBytesIntensity  = (numBytesPerIntensityValue_ > 0) ? intensity_.total()*numBytesPerIntensityValue_ : 0;
        const size_t numBytesConfidence = (numBytesPerConfidenceValue_ > 0) ? confidence_.total()*numBytesPerConfidenceValue_ : 0;

        // the binary part starts with entries for length, a timestamp
        // and a version identifier
        size_t offset = 0;
        const uint32_t length = __Swap4Bytes( *(uint32_t*)(data+offset) );
        const uint64_t timeStamp = __Swap8Bytes( *(uint64_t*)(data+offset+4) );
        const uint16_t version = __Swap2Bytes( *(uint16_t*)(data+offset+12) );

        offset += 14; //calcsize('>IQH')

        if(length>size) {
            ROS_WARN("malformed data (2): %d>%d", (int)length, (int)size);
            return length;
        }

        if (version > 1) {
            // more frame information follows in this case: frame number, data quality, device status ('<IBB')
            framenumber_ = ntohl( *(uint32_t*)(data+offset) );
            const uint8_t dataQuality = *(uint8_t*)(data+offset+4);
            const uint8_t deviceStatus = *(uint8_t*)(data+offset+5);
            offset += 6;
        } else {
            ++framenumber_;
        }

        size_t end = offset + numBytesDistance + numBytesIntensity + numBytesConfidence; // calculating the end index
        //wholeBinary = binarySegment[start:end] // whole data block
        //distance = wholeBinary[0:numBytesDistance] // only the distance
                                                   // data (as string)

        ROS_ASSERT(end<=size);

        if (numBytesDistance > 0) {
            for(size_t i=0; i<distance_.total(); i++)
                distance_.at<uint16_t>(i) = __Swap2BytesD( *(uint16_t*)(data+offset+i*2) );
            offset += numBytesDistance;
        }
        else {
            distance_.setTo(0);
        }
        if (numBytesIntensity > 0) {
            for(size_t i=0; i<intensity_.total(); i++)
                intensity_.at<uint16_t>(i) = __Swap2Bytes( *(uint16_t*)(data+offset+i*2) );
            offset += numBytesIntensity;
        }
        else {
            intensity_.setTo(std::numeric_limits<uint16_t>::max()/2);
        }
        if (numBytesConfidence > 0) {
            for(size_t i=0; i<confidence_.total(); i++)
                confidence_.at<uint16_t>(i) = __Swap2Bytes( *(uint16_t*)(data+offset+i*2) );
            offset += numBytesConfidence;
        }
        else {
            confidence_.setTo(std::numeric_limits<uint16_t>::max());
        }

        if(offset!=length) {    //otherwise another binary block
            // data end with a 32byte (unused) CRC field and a copy of the length byte
            const uint32_t unusedCrc = __Swap4Bytes( *(uint32_t*)(data+offset) );
            const uint32_t lengthCopy = __Swap4Bytes( *(uint32_t*)(data+offset + 4) );

            if(length != lengthCopy)
                ROS_WARN("check failed %d!=%d", (int)length, (int)lengthCopy);
            ROS_ASSERT(length == lengthCopy);
        }
        
        return length;
    }

};

#undef __Swap2Bytes
