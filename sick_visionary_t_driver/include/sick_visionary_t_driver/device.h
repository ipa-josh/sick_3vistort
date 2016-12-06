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
 *   Device control and stream init.
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
 
#pragma once
#include "cola.h"

#define SUPPORTED_DEVICE_VERSION "2.9.4.8923R"

//read parameters from "somewhere" -> ros functions outside
bool read_param(const std::string &name, std::string &var);

struct ControlVariables {
    //blob information
    std::string blob_transport_protocol;
    std::string blob_device_ip_addr;
    std::string blob_multicast_ip_addr;
    uint16_t blob_tcp_port;
    uint16_t blob_udp_peer_port;
    uint16_t blob_udp_local_port;
    bool blob_active;
    uint16_t blob_fragment_size;
    uint16_t blob_tcp_port_api;
    
    //IOs
    int8_t IOValue[6];
    
    //parameters
    uint16_t framePeriod;
    uint8_t confAlgo;
    uint8_t integrationTime;
    uint8_t nareThreshold;
    uint8_t modFreq;
    
    //health
    uint8_t TmpLvl;
    uint32_t doutPinError;
    uint8_t doutOverload;
    uint8_t DataQualityLevel;
    
    //control parameters
    uint8_t PowerMode;
    bool enableDepthMapAPI;
    bool enablePolarScanAPI;
    bool enableHeightMapAPI;
    bool enablePolarScan;
    bool enableCartMap;
    
    //device information
    std::string device_name, device_version;
    
    //internal states
    bool applyingParams;
};

typedef std::map<std::string, std::string> DiagnoseInfo;

/* all methods that use the control channel (sopas) */
class Control : public TCP_Session {
    Any_Session::SIG_ON_DATA on_data_;      ///< signal handler for incoming data
    const std::string remote_device_ip_;
    bool stream_started_;
    CoLaFrame frame_recv_;
    ControlVariables control_variables_;
    boost::signals2::signal< void(const ControlVariables &, const DiagnoseInfo &) > on_diag_changed_;
    boost::mutex lock_;
    
    typedef boost::signals2::signal<bool (const std::string &) > SIG_ON_METHOD;
    
    struct SData {
        enum EType {METHOD, VARIABLE};
        
        EType type_;
        StructParser parser_;
        SIG_ON_METHOD callback_;
        boost::timed_mutex lock_;
        bool success_;
        bool param_;
    };
    typedef std::map<std::string, boost::shared_ptr<SData> > DataParserMap;
    DataParserMap data_parser_;
    
    inline static bool cmp_cmds(std::string &response, const std::string &cmd) {
        if(response.compare(0, cmd.size(), cmd)==0) {
            size_t offset=cmd.size();
            if(response.size()>cmd.size() && response[cmd.size()]==' ')
                offset+=1;
            response = std::string(response.begin()+offset, response.end());
            return true;
        }
    }
    
    DiagnoseInfo generateDiagnoseInfo() const {
        DiagnoseInfo info;
        for(DataParserMap::const_iterator it=data_parser_.begin(); it!=data_parser_.end(); it++) {
            it->second->parser_ >> info;
        }
        return info;
    }
    
    static void parse_response(const std::string &response, std::string &name, std::string &data)
    {
        size_t pos = response.find(' ', 4);
        if(pos==std::string::npos) {
            pos = response.size();
            name = response;
        }
        else {
            name = std::string(response.begin(), response.begin()+pos);
            pos+=1;
        }
        ROS_DEBUG("response from %s (%d)", name.c_str(), (int)(response.size()-pos));
        
        data = std::string(response.begin()+pos, response.end());
    }
    
    void on_data(const char *data, const size_t size, Any_Session *writer)
    {
        if(!data || size<0) {
            ROS_DEBUG("Socket not connected, terminating.");
            return;
        }
        
        boost::mutex::scoped_lock lock(lock_);
        
        ROS_DEBUG("got data for control");
        
        frame_recv_.add_data(data, size);
        
        while(frame_recv_.pop_until_valid()) {
            std::string response = frame_recv_.get_data();
            
            std::string name, data;
            if(cmp_cmds(response, "sAN ")) {
                parse_response(response, name, data);
                DataParserMap::iterator it = data_parser_.find(name);
                if(it!=data_parser_.end()) {
                    if(it->second->type_!=SData::METHOD)
                        ROS_WARN("expected a variable, but got method");
                    it->second->lock_.try_lock();
                    it->second->success_  = it->second->parser_.parse(data.c_str(), data.size()) && it->second->callback_(data);
                    it->second->lock_.unlock();
                }
                else ROS_DEBUG("no handler for method: '%s'", name.c_str());
            }
            else if(cmp_cmds(response, "sRA ")) {
                parse_response(response, name, data);
                DataParserMap::iterator it = data_parser_.find(name);
                if(it!=data_parser_.end()) {
                    if(it->second->type_!=SData::VARIABLE)
                        ROS_WARN("expected a variable, but got method");
                    it->second->lock_.try_lock();
                    it->second->success_  = it->second->parser_.parse(data.c_str(), data.size()) && it->second->callback_(data);
                    it->second->lock_.unlock();
                }
                else ROS_DEBUG("no handler for variable: '%s'", name.c_str());
            }
            else if(cmp_cmds(response, "sWA ")) {
                parse_response(response, name, data);
                DataParserMap::iterator it = data_parser_.find(name);
                if(it!=data_parser_.end())
                    request("sRN "+name);
            }
            else if(cmp_cmds(response, "sFA")) {
                ROS_ERROR("error notification by SOPAS");
            }
            else
                ROS_ERROR("unknown command '%s'", response.c_str());
            
            frame_recv_.pop();
        }
        
    }
    
    void request(const std::string &cmd) {
        CoLaFrame fr(cmd);
        ROS_DEBUG("Sending on %s", cmd.c_str());
        write(fr.finish());
    }
    
    bool call_method(const std::string &method_name, const std::string &data=std::string()) {
        DataParserMap::iterator it = data_parser_.find(method_name);
        if(it==data_parser_.end() || it->second->type_!=SData::METHOD) {
            ROS_ERROR("unknown function: %s", method_name.c_str());
            return false;
        }
        
        it->second->lock_.lock();
        request("sMN "+method_name+" "+data);
        
        bool ret = it->second->lock_.timed_lock(boost::posix_time::seconds(1)) && it->second->success_;
        ROS_DEBUG("call '%s': %s", method_name.c_str(), ret?"success":"failure");
        
        it->second->lock_.try_lock();
        it->second->lock_.unlock();
        return ret;
    }
    
    bool read_variable(const std::string &variable_name) {
        DataParserMap::iterator it = data_parser_.find(variable_name);
        if(it==data_parser_.end() || it->second->type_!=SData::VARIABLE) {
            ROS_ERROR("unknown variable: %s", variable_name.c_str());
            return false;
        }
        
        it->second->lock_.lock();
        request("sRN "+variable_name);
        
        bool ret = it->second->lock_.timed_lock(boost::posix_time::seconds(1)) && it->second->success_;
        if(!ret)
            ROS_WARN("read '%s': %s", variable_name.c_str(), ret?"success":"failure");
        else
            ROS_DEBUG("read '%s': %s", variable_name.c_str(), ret?"success":"failure");
        
        it->second->lock_.try_lock();
        it->second->lock_.unlock();
        return ret;
    }
    
    bool write_variable(const std::string &variable_name, const std::string &data=std::string(), const bool blocking=false) {
        DataParserMap::iterator it = data_parser_.find(variable_name);
        if(it==data_parser_.end() || it->second->type_!=SData::VARIABLE) {
            ROS_ERROR("unknown variable: %s", variable_name.c_str());
            return false;
        }
        
        if(blocking) it->second->lock_.lock();
        request("sWN "+variable_name+" "+(data.size()==0?it->second->parser_.generate():data));
        
        if(!blocking) return true;
        
        bool ret = it->second->lock_.timed_lock(boost::posix_time::seconds(1)) && it->second->success_;
        
        it->second->lock_.try_lock();
        it->second->lock_.unlock();
        return ret;
    }
    
    
    //callbacks of methods
    bool meth_GetBlobClientConfig(const std::string &data) {
        on_diag_changed_(control_variables_, generateDiagnoseInfo());
        return true;
    }
    
    bool meth_Dummy(const std::string &data) {
        return true;
    }
    
    bool meth_PLAYSTART(const std::string &data) {
        stream_started_ = true;
        return true;
    }
    
    bool meth_PLAYSTOP(const std::string &data) {
        stream_started_ = false;
        return true;
    }
    
    bool meth_SetAccessMode(const std::string &data) {
        assert(data.size()==1);
        
        if(data.size()>=1)
            return data[0]==1;
        return false;
    }
    
    static bool var_Dummy(const std::string &data) {
        return true;
    }
    
    bool var_DiagVar(const std::string &data) {
        on_diag_changed_(control_variables_, generateDiagnoseInfo());
        return true;
    }
    
    bool var_DeviceInfo(const std::string &data) {
        if(control_variables_.device_version!=SUPPORTED_DEVICE_VERSION)
            ROS_WARN("unsupported device version: '%s' (Supported: %s)",
                control_variables_.device_version.c_str(),
                SUPPORTED_DEVICE_VERSION);
            
        return var_DiagVar(data);
    }
    
    //helpers to setup
    StructParser &createMethod(const std::string &methode_name, const boost::function<bool(const std::string &)> &callback) {
        boost::shared_ptr<SData> data(new SData);
        data->type_ = SData::METHOD;
        data->param_ = false;
        data->callback_.connect(callback);
        data_parser_.insert( std::pair<std::string, boost::shared_ptr<SData> >(methode_name, data) );
        
        return data->parser_;
    }
    
    StructParser &_createVariable(const std::string &var_name, DatatypeParserAbstract *type, const bool param=false, const boost::function<bool(const std::string &)> &callback = boost::bind(&Control::var_Dummy, _1)) {
        boost::shared_ptr<SData> data(new SData);
        data->type_ = SData::VARIABLE;
        data->param_ = false;
        if(param) {
            std::string init_value;
            data->param_ = read_param(var_name, init_value);
            if(data->param_) {
                ROS_DEBUG("setting variable '%s' with initial value='%s'", var_name.c_str(), init_value.c_str());
                type->set(init_value);
            }
            else ROS_DEBUG("not setting variable '%s'", var_name.c_str());
        }
        data->parser_(type);
        data->callback_.connect(callback);
        data_parser_.insert( std::pair<std::string, boost::shared_ptr<SData> >(var_name, data) );
        
        return data->parser_;
    }
    
    template<class T>
    StructParser &createVariable(const std::string &var_name, T *variable, const bool param=false, const boost::function<bool(const std::string &)> &callback = boost::bind(&Control::var_Dummy, _1), const std::string &alternative_name=std::string()) {
        return _createVariable(var_name, new DatatypeParser<T>(variable, alternative_name.size()>0?alternative_name:var_name), param, callback);
    }
    
    template<class T>
    DatatypeParserEnum<T> &createEnumVariable(const std::string &var_name, T *variable, const bool param=false, const boost::function<bool(const std::string &)> &callback = boost::bind(&Control::var_Dummy, _1), const std::string &alternative_name=std::string()) {
        DatatypeParserEnum<T> *e = new DatatypeParserEnum<T>(variable, alternative_name.size()>0?alternative_name:var_name);
        _createVariable(var_name, e, param, callback);
        return *e;
    }
    
    bool waitForParamsApplied() {
        control_variables_.applyingParams = true;
        for(int i=0; i<30; i++) {
            ros::Duration(0.2).sleep();
            read_variable("applyingParams");
            if(!control_variables_.applyingParams) return true;
        }
        
        ROS_WARN("waitForParamsApplied failed");
        
        return false;
    }
    
public:
    enum EUserLevel {RUN_LEVEL=0, OPERATOR=1, MAINTENANCE=2, AUTHORIZEDCLIENT=3, SERVICE=4};
    enum EIlluminationCode {
        ILLUMINATION_MODE_1=0, ILLUMINATION_MODE_2=1, ILLUMINATION_MODE_3=2, ILLUMINATION_MODE_4=3,
        ILLUMINATION_MODE_5=4, ILLUMINATION_MODE_6=5, ILLUMINATION_MODE_7=6, ILLUMINATION_MODE_8=7,
        ILLUMINATION_MODE_AUTOMATIC=8
    };
    enum EIntegrationTime {MS_0500=0, MS_1000=1, MS_1500=2, MS_2000=3, MS_2500=4};
    enum EPowerMode {ACTIVE=0, STREAMING_STANDBY=1};
    enum EThreeLevel {INVALID=0, ERROR=1, WARNING=2, GOOD=3};
    
    Control(boost::asio::io_service& io_service, const std::string &remote_device_ip,
        const boost::function<void(const ControlVariables &, const DiagnoseInfo &)> &diag_callback) :
        TCP_Session(io_service, on_data_),
        remote_device_ip_(remote_device_ip),
        stream_started_(false)
    {
        on_diag_changed_.connect(diag_callback);
        
        //setup the methods and variables
        DatatypeParserEnum<uint8_t> *power_mode = new DatatypeParserEnum<uint8_t>(&control_variables_.PowerMode, "_Power Mode");
        (*power_mode)
            COLA_ENUM(ACTIVE)
            COLA_ENUM(STREAMING_STANDBY);
        
        createMethod("SetAccessMode", boost::bind(&Control::meth_SetAccessMode, this, _1));
        createMethod("PLAYSTART", boost::bind(&Control::meth_PLAYSTART, this, _1));
        createMethod("PLAYSTOP", boost::bind(&Control::meth_PLAYSTOP, this, _1));
        createMethod("PLAYNEXT", boost::bind(&Control::meth_PLAYSTOP, this, _1));
        createMethod("DeviceReInit", boost::bind(&Control::meth_Dummy, this, _1));
        createMethod("SetPwrMod", boost::bind(&Control::meth_Dummy, this, _1))((DatatypeParserAbstract*)power_mode);
        createMethod("GetBlobClientConfig", boost::bind(&Control::meth_GetBlobClientConfig, this, _1))
            (&control_variables_.blob_transport_protocol, "Blob Transport Protocol")
            (&control_variables_.blob_device_ip_addr, "Blob Device IP Address")
            (&control_variables_.blob_multicast_ip_addr, "Blob Multicast IP Address")
            (&control_variables_.blob_tcp_port, "Blob TCP Port")
            (&control_variables_.blob_udp_peer_port, "Blob UDP Peer Port")
            (&control_variables_.blob_udp_local_port, "Blob UDP Local Port")
            (&control_variables_.blob_active, "Blob Active")
            (&control_variables_.blob_fragment_size, "Blob Fragment Size");
            
        createVariable("applyingParams", &control_variables_.applyingParams, false, boost::bind(&Control::var_Dummy, _1), "_applyingParams");
        
        createEnumVariable("modFreq", &control_variables_.modFreq, true, boost::bind(&Control::var_DiagVar, this, _1), "Modulation frequency")
            COLA_ENUM(ILLUMINATION_MODE_1)
            COLA_ENUM(ILLUMINATION_MODE_2)
            COLA_ENUM(ILLUMINATION_MODE_3)
            COLA_ENUM(ILLUMINATION_MODE_4)
            COLA_ENUM(ILLUMINATION_MODE_5)
            COLA_ENUM(ILLUMINATION_MODE_6)
            COLA_ENUM(ILLUMINATION_MODE_7)
            COLA_ENUM(ILLUMINATION_MODE_8)
            COLA_ENUM(ILLUMINATION_MODE_AUTOMATIC);
            
        //system health
        createEnumVariable("TmpLvl", &control_variables_.TmpLvl, false, boost::bind(&Control::var_DiagVar, this, _1), "Temperature Level")
            COLA_ENUM(INVALID)
            COLA_ENUM(ERROR)
            COLA_ENUM(WARNING)
            COLA_ENUM(GOOD);
        createVariable("DoPinErr", &control_variables_.doutPinError, false, boost::bind(&Control::var_DiagVar, this, _1), "Digital output health");
        createVariable("DoOvrld", &control_variables_.doutOverload, false, boost::bind(&Control::var_DiagVar, this, _1), "Digital output overheated");
        createEnumVariable("DatQLvl", &control_variables_.DataQualityLevel, false, boost::bind(&Control::var_DiagVar, this, _1), "Data quality level")
            COLA_ENUM(INVALID)
            COLA_ENUM(ERROR)
            COLA_ENUM(WARNING)
            COLA_ENUM(GOOD);
            
        createVariable("BlobTcpPortAPI", &control_variables_.blob_tcp_port_api, false, boost::bind(&Control::var_DiagVar, this, _1), "Blob TCP Port API");
        createVariable("DeviceIdent", &control_variables_.device_name, false, boost::bind(&Control::var_DeviceInfo, this, _1), "Device Name")(&control_variables_.device_version, "Device Version");
        createVariable("enDepthAPI", &control_variables_.enableDepthMapAPI, false, boost::bind(&Control::var_DiagVar, this, _1), "Depth Map API enabled");
        createVariable("enPolarAPI", &control_variables_.enablePolarScanAPI, false, boost::bind(&Control::var_DiagVar, this, _1), "Polar Scan API enabled");
        createVariable("enHeightAPI", &control_variables_.enableHeightMapAPI, false, boost::bind(&Control::var_DiagVar, this, _1), "Height Map API enabled");
        
        createVariable("enPolar", &control_variables_.enablePolarScan, false, boost::bind(&Control::var_DiagVar, this, _1), "Polar Scan enabled");
        createVariable("enCart", &control_variables_.enableCartMap, false, boost::bind(&Control::var_DiagVar, this, _1), "Cartesian Map enabled");
        
        createEnumVariable("confAlgo", &control_variables_.confAlgo, true, boost::bind(&Control::var_DiagVar, this, _1), "Confidence algorithm");
        createEnumVariable("integrationTime", &control_variables_.integrationTime, true, boost::bind(&Control::var_DiagVar, this, _1), "Integration time")
            COLA_ENUM(MS_0500)
            COLA_ENUM(MS_1000)
            COLA_ENUM(MS_1500)
            COLA_ENUM(MS_2000)
            COLA_ENUM(MS_2500);
        createEnumVariable("nareThreshold", &control_variables_.nareThreshold, true, boost::bind(&Control::var_DiagVar, this, _1), "Amplitude threshold");
        createVariable("framePeriod", &control_variables_.framePeriod, true, boost::bind(&Control::var_DiagVar, this, _1), "Frame Period");
        createVariable("IOValue", &control_variables_.IOValue[0], false,  boost::bind(&Control::var_Dummy, _1), "_IO Value 0")
            (&control_variables_.IOValue[1], "_IO Value 1")
            (&control_variables_.IOValue[2], "_IO Value 2")
            (&control_variables_.IOValue[3], "_IO Value 3")
            (&control_variables_.IOValue[4], "_IO Value 4")
            (&control_variables_.IOValue[5], "_IO Value 5");
        
        on_data_.connect( boost::bind(&Control::on_data, this, _1, _2, _3) );
    }
    
    bool connect_IO_callback(const boost::function<bool(const std::string &)> &io_callback) {
        const std::string variable_name = "IOValue";
        DataParserMap::iterator it = data_parser_.find(variable_name);
        if(it==data_parser_.end() || it->second->type_!=SData::VARIABLE) {
            ROS_ERROR("unknown variable: %s", variable_name.c_str());
            return false;
        }
        
        it->second->callback_.connect(io_callback);
        return true;
    }
    
    bool isAGDevice() const {
        return control_variables_.device_name.find("AG")!=std::string::npos;
    }
    
    void triggerIO() {
        request("sRN IOValue");
    }
    
    /* establish the control channel to the device */
    bool open() {
        ROS_DEBUG("Connecting to device...");
        
        if(!connect(remote_device_ip_, "2112")) {
            ROS_ERROR("Error on connecting to %s", remote_device_ip_.c_str());
            return false;
        }
        
        ROS_DEBUG("done.");
        
        return true;
    }
    
    bool setFramePeriod(uint16_t hz) {
        if(!write_variable("framePeriod", StructParser()(&hz).generate(), true))
            return false;
        return call_method("DeviceReInit");
    }
    
    bool enableDepthMap() {
        bool var;
        
        if(control_variables_.enableCartMap || control_variables_.enablePolarScan) {
            var = false;
            if(!write_variable("enCart", StructParser()(&var).generate(), true))
                return false;
            var = false;
            if(!write_variable("enPolar", StructParser()(&var).generate(), true))
                return false;
                
            waitForParamsApplied();
        }
        
        var = false;
        if(!write_variable("enPolarAPI", StructParser()(&var).generate(), true))
            return false;
        var = false;
        if(!write_variable("enHeightAPI", StructParser()(&var).generate(), true))
            return false;
        var = true;
        if(!write_variable("enDepthAPI", StructParser()(&var).generate(), true))
            return false;
            
        return call_method("DeviceReInit");
    }
    
    bool enablePolarScan() {
        bool var;
        
        if(control_variables_.enableCartMap) { //just one of these can run
            var = false;
            if(!write_variable("enCart", StructParser()(&var).generate(), true))
                return false;
            var = false;
            if(!write_variable("enHeightAPI", StructParser()(&var).generate(), true))
                return false;
        }
        
        var = true;
        if(!write_variable("enPolar", StructParser()(&var).generate(), true))
            return false;
        waitForParamsApplied();
        
        var = false;
        if(!write_variable("enDepthAPI", StructParser()(&var).generate(), true))
            return false;
        var = true;
        if(!write_variable("enPolarAPI", StructParser()(&var).generate(), true))
            return false;
            
        return call_method("DeviceReInit");
    }
    
    bool enableHeightMap() {
        bool var;
        
        if(control_variables_.enablePolarScan) { //just one of these can run
            var = false;
            if(!write_variable("enPolar", StructParser()(&var).generate(), true))
                return false;
            var = false;
            if(!write_variable("enPolarAPI", StructParser()(&var).generate(), true))
                return false;
        }
        
        var = true;
        if(!write_variable("enCart", StructParser()(&var).generate(), true))
            return false;
        waitForParamsApplied();
        
        var = false;
        if(!write_variable("enDepthAPI", StructParser()(&var).generate(), true))
            return false;
        var = true;
        if(!write_variable("enHeightAPI", StructParser()(&var).generate(), true))
            return false;
            
        return call_method("DeviceReInit");
    }
        
    bool read_parameters() {
        //read all parameters
        for(DataParserMap::const_iterator it=data_parser_.begin(); it!=data_parser_.end(); it++)
            if(it->second->type_==SData::VARIABLE) {
                if(it->second->param_)
                    write_variable(it->first);
                else
                    read_variable(it->first);
            }
        
        return true;
    }
    
    void setModFreq(const EIlluminationCode illumination_code) {
        ROS_DEBUG("Setting modulation frequency to %d", (int)illumination_code);
        
        uint8_t code = (uint8_t)illumination_code;
        write_variable("modFreq", StructParser()(&code).generate());
    }

    /* Tells the device that there is a streaming channel by invoking a
     * method named GetBlobClientConfig.
     */
    bool initStream() {
        ROS_DEBUG("Initializing streaming...");
        return call_method("GetBlobClientConfig");
    }

    /* Start streaming the data by calling the "PLAYSTART" method on the
     * device and sending a "Blob request" afterwards.
     */
    bool startStream() {   
        if(stream_started_) return true;
             
        return call_method("PLAYSTART");
    }
        
    /* Stops the data stream. */
    bool stopStream() { 
        if(!stream_started_) return true;
             
        return call_method("PLAYSTOP");
    }
        
    /* Requests one frame. */
    bool singleStep() { 
        if(stream_started_) return true;
             
        return call_method("PLAYNEXT");
    }
    
    /* Call: SetAccessMode to change operation mode */
    bool setAccessMode(uint8_t mode, uint32_t hash) {       
        return call_method("SetAccessMode", StructParser()(&mode)(&hash).generate());
    }
    
    bool setAccessMode(const EUserLevel user_level, const uint32_t hash=0) {
        switch(user_level) {
            case MAINTENANCE:
                return setAccessMode((uint8_t)user_level, hash!=0?hash:0x557700E6);
                break;
            case AUTHORIZEDCLIENT:
                return setAccessMode((uint8_t)user_level, hash!=0?hash:0xFB356CDE);
                break;
            case SERVICE:
                return setAccessMode((uint8_t)user_level, hash!=0?hash:0XED784BAA);
                break;
            default:
                ROS_WARN("user level is not supported");
                assert(0);
                break;
        }
        
        return false;
    }
    
    const ControlVariables &getControlVariables() const {
        return control_variables_;
    }
};


    
/* All methods that use the streaming channel. */
class Streaming : public TCP_Session {
public:

    struct SFrame {
        std::vector<char> buffer;
        
        typedef boost::shared_ptr<SFrame> Ptr;
    };
    
    typedef boost::signals2::signal<void (const boost::shared_ptr<Data> &)> SIG_ON_FRAME;
    
private:
    const std::string remote_device_ip_;
    const std::string remote_device_port_;
    Any_Session::SIG_ON_DATA on_data_;      ///< signal handler for incoming data
    SIG_ON_FRAME on_frame_;
    bool debugOutput_;
    SFrame::Ptr cur_frame_;
    
public:

    Streaming(boost::asio::io_service& io_service, const std::string &remote_device_ip, const std::string &remote_device_port="2113") :
        TCP_Session(io_service, on_data_),
        remote_device_ip_(remote_device_ip),
        remote_device_port_(remote_device_port),
        debugOutput_(false)
    {
        on_data_.connect( boost::bind(&Streaming::on_data, this, _1, _2, _3) );
    }
    
    SIG_ON_FRAME &getSignal() {return on_frame_;}
    
    bool &debugFlag()       {return debugOutput_;}
    bool debugFlag() const {return debugOutput_;}
    
    /* Opens the streaming channel. */
    bool openStream() {
        ROS_DEBUG("Opening streaming socket...");
        if(!connect(remote_device_ip_, remote_device_port_)) {
            ROS_DEBUG("Error on connecting to %s", remote_device_ip_.c_str());
            return false;
        }
        ROS_DEBUG("done.");
        
        // saying hello
        const char HEARTBEAT_MSG[] = "BlbReq";
        
        ROS_DEBUG("Sending BlbReq: %s", HEARTBEAT_MSG);
        write(std::string(HEARTBEAT_MSG, sizeof(HEARTBEAT_MSG)-1));
        
        return true;
    }
        
    /* Closes the streaming channel. */
    bool closeStream() {
        ROS_DEBUG("Closing streaming connection...");
        close();
        cur_frame_.reset();
        ROS_DEBUG("done.");
        
        return true;
    }

    /*
     *  Receives the raw data frame from the device via the streamingbchannel.
     */
    void on_data(const char *data, const size_t size, Any_Session *writer)
    {
        if(debugOutput_)
            ROS_DEBUG("Reading image from stream...");
        
        if(!data || size<0) {
            ROS_DEBUG("Socket not connected, terminating.");
            return;
        }
            
        if(cur_frame_) {
            cur_frame_->buffer.insert(cur_frame_->buffer.end(), data, data+size );

            bool goon=true;
            while(goon) {
                goon=false;

                if(Data::check_header(&cur_frame_->buffer[0], cur_frame_->buffer.size())) {             
                    boost::shared_ptr<Data> parser(new Data);
                    const size_t actual_size = parser->actual_size(&cur_frame_->buffer[0], cur_frame_->buffer.size());
                    if(parser->read(&cur_frame_->buffer[0], actual_size)) {
                        on_frame_(parser);
                        goon=true;
                    }
                    else {
                        cur_frame_.reset();
                        ROS_ERROR("failed to parse frame");
                        break;
                    }

                    cur_frame_->buffer.erase(cur_frame_->buffer.begin(), cur_frame_->buffer.begin()+actual_size);
                }
            }
        }
        else {
            cur_frame_.reset(new SFrame());
            on_data(data, size, writer);
        }
               
        if(debugOutput_)
            ROS_DEBUG("done.");
    }
};
