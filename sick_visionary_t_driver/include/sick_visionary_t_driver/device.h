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

template<class T>
bool read_param(const std::string &name, T &var) {
	return ros::param::get("~"+name, var);
}

template<>
bool read_param<uint8_t>(const std::string &name, uint8_t &var);


struct ControlVariables {
	std::string blob_transport_protocol;
	
	std::string device_name, device_version;
	uint8_t modFreq;
};

/* all methods that use the control channel (sopas) */
class Control : public TCP_Session {
	Any_Session::SIG_ON_DATA on_data_;		///< signal handler for incoming data
	const std::string remote_device_ip_;
	bool stream_started_;
	CoLaFrame frame_recv_;
	ControlVariables control_variables_;
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
			ROS_DEBUG("response: %s (%d)", response.c_str(), (int)response.size());
			
			size_t pos = response.find(' ', 4);
			std::string name;
			if(pos==std::string::npos) {
				pos = response.size();
				name = response;
			}
			else {
				pos+=1;
				name = std::string(response.begin(), response.begin()+pos);
			}
			ROS_DEBUG("response from %s (%d)", name.c_str(), (int)(response.size()-pos));
			
			std::string data = std::string(response.begin()+pos, response.end());
			
			if(cmp_cmds(response, "sAN ")) {				
				DataParserMap::iterator it = data_parser_.find(name);
				if(it!=data_parser_.end()) {
					if(it->second->type_!=SData::METHOD)
						ROS_WARN("expected a variable, but got method");
					it->second->lock_.try_lock();
					it->second->success_  = it->second->parser_.parse(data.c_str(), data.size()) && it->second->callback_(data);
					it->second->lock_.unlock();
				}
				
			}
			else if(cmp_cmds(response, "sRA ")) {
				DataParserMap::iterator it = data_parser_.find(name);
				if(it!=data_parser_.end()) {
					if(it->second->type_!=SData::VARIABLE)
						ROS_WARN("expected a variable, but got method");
					it->second->lock_.try_lock();
					it->second->success_  = it->second->parser_.parse(data.c_str(), data.size()) && it->second->callback_(data);
					it->second->lock_.unlock();
				}
						
				assert(response.size()==1);
				if(response.size()>=1)
					ROS_INFO("modFreq=%d", (int)response[0]);
			}
			else if(cmp_cmds(response, "sWA ")) {
				DataParserMap::iterator it = data_parser_.find(name);
				if(it!=data_parser_.end())
					request("sRN "+name);
			}
			
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
		ROS_DEBUG("read '%s': %s", variable_name.c_str(), ret?"success":"failure");
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
		
		return it->second->lock_.timed_lock(boost::posix_time::seconds(1)) && it->second->success_;
	}
	
	
	//callbacks of methods
	bool meth_GetBlobClientConfig(const std::string &data) {
		size_t offset=0;
		uint16_t len;
		const char *ptr = data.c_str();
		
		if(data.size()-offset<2) return false;
		len = ntohs( *((uint16_t*)(ptr+offset)) );
		offset+=2;
		if(data.size()-offset<len) return false;
		std::string transport_protocol(ptr+offset, ptr+(offset+len));
		offset+=len;
		
		ROS_INFO("BlobClientConfig %s", transport_protocol.c_str());
		
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
	
	
	//helpers to setup
	StructParser &createMethod(const std::string &methode_name, const boost::function<bool(const std::string &)> &callback) {
		boost::shared_ptr<SData> data(new SData);
		data->type_ = SData::METHOD;
		data->param_ = false;
		data->callback_.connect(callback);
		data_parser_.insert( std::pair<std::string, boost::shared_ptr<SData> >(methode_name, data) );
		
		return data->parser_;
	}
	
	template<class T>
	StructParser &createVariable(const std::string &var_name, T *variable, const bool param=false, const boost::function<bool(const std::string &)> &callback = boost::bind(&Control::var_Dummy, _1)) {
		boost::shared_ptr<SData> data(new SData);
		data->type_ = SData::VARIABLE;
		data->param_ = param;
		if(param)
			read_param(var_name, *variable);
		data->parser_(variable);
		data->callback_.connect(callback);
		data_parser_.insert( std::pair<std::string, boost::shared_ptr<SData> >(var_name, data) );
		
		return data->parser_;
	}
	
public:
	enum EUserLevel {RUN_LEVEL=0, OPERATOR=1, MAINTENANCE=2, AUTHORIZEDCLIENT=3, SERVICE=4};
	enum EIlluminationCode {
		ILLUMINATION_MODE_1=0, ILLUMINATION_MODE_2=1, ILLUMINATION_MODE_3=2, ILLUMINATION_MODE_4=3,
		ILLUMINATION_MODE_5=4, ILLUMINATION_MODE_6=5, ILLUMINATION_MODE_7=6, ILLUMINATION_MODE_8=7,
		ILLUMINATION_MODE_AUTOMATIC=8
	};
	
	Control(boost::asio::io_service& io_service, const std::string &remote_device_ip) :
		TCP_Session(io_service, on_data_),
		remote_device_ip_(remote_device_ip),
		stream_started_(false)
	{
		//setup the methods and variables
		createMethod("SetAccessMode", boost::bind(&Control::meth_SetAccessMode, this, _1));
        createMethod("PLAYSTART", boost::bind(&Control::meth_PLAYSTART, this, _1));
        createMethod("PLAYSTOP", boost::bind(&Control::meth_PLAYSTOP, this, _1));
        createMethod("GetBlobClientConfig", boost::bind(&Control::meth_GetBlobClientConfig, this, _1))
			(&control_variables_.blob_transport_protocol);
			
		createVariable("modFreq", &control_variables_.modFreq);
		createVariable("DeviceIdent", &control_variables_.device_name)(&control_variables_.device_version);
        
		on_data_.connect( boost::bind(&Control::on_data, this, _1, _2, _3) );
	}
	
    /* establish the control channel to the device */
    bool open() {
        ROS_DEBUG("Connecting to device...");
        
        if(!connect(remote_device_ip_, "2112")) {
            ROS_ERROR("Error on connecting to %s", remote_device_ip_.c_str());
            return false;
		}
		
        //read all parameters
        for(DataParserMap::const_iterator it=data_parser_.begin(); it!=data_parser_.end(); it++)
			if(it->second->type_==SData::VARIABLE) {
				if(it->second->param_)
					write_variable(it->first);
				else
					read_variable(it->first);
			}
        
        ROS_DEBUG("done.");
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
	
	/* Call: SetAccessMode to change operation mode */
	bool setAccessMode(uint8_t mode, uint32_t hash) {		
        return call_method("SetAccessMode", StructParser()(&mode)(&hash).generate());
	}
	
	bool setAccessMode(const EUserLevel user_level, const uint32_t hash=0) {
		switch(user_level) {
			case MAINTENANCE:
				return setAccessMode(user_level, hash!=0?hash:0x557700E6);
				break;
			case AUTHORIZEDCLIENT:
				return setAccessMode(user_level, hash!=0?hash:0xFB356CDE);
				break;
			case SERVICE:
				return setAccessMode(user_level, hash!=0?hash:0XED784BAA);
				break;
			default:
				ROS_WARN("user level is not supported");
				assert(0);
				break;
		}
		
		return false;
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
	Any_Session::SIG_ON_DATA on_data_;		///< signal handler for incoming data
	SIG_ON_FRAME on_frame_;
	bool debugOutput_;
	SFrame::Ptr cur_frame_;
	
public:

	Streaming(boost::asio::io_service& io_service, const std::string &remote_device_ip) :
		TCP_Session(io_service, on_data_),
		remote_device_ip_(remote_device_ip),
		debugOutput_(false)
	{
		on_data_.connect( boost::bind(&Streaming::on_data, this, _1, _2, _3) );
	}
	
	SIG_ON_FRAME &getSignal() {return on_frame_;}
	
	bool &debugFlag() 		{return debugOutput_;}
	bool debugFlag() const {return debugOutput_;}
	
    /* Opens the streaming channel. */
    bool openStream() {
        ROS_DEBUG("Opening streaming socket...");
        if(!connect(remote_device_ip_, "2113")) {
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
