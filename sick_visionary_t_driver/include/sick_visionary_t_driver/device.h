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
 
class CoLaFrame {
	std::vector<char> buffer_;
	
	uint8_t checksum(const size_t offset=0) const {
		uint8_t c=0;
		for(size_t i=8; i<buffer_.size()-offset; i++)
			c^=buffer_[i];
		return c;
	}
	
public:
	CoLaFrame() : buffer_(8, (char)0) {
	}
	
	CoLaFrame(const std::string str) : buffer_(8+str.size(), (char)0) {
		for(size_t i=0; i<str.size(); i++)
			buffer_[i+8] = str[i];
	}
	
	std::vector<char> finish() {
		char *data = &buffer_[0];
		*(uint32_t*)(data+0) = htonl(0x02020202);
		*(uint32_t*)(data+4) = htonl(buffer_.size()-8);
		
		buffer_.push_back(checksum());
		
		return buffer_;
	}
	
	void add_data(const char *data, const size_t size) {
		buffer_.insert(buffer_.end(), data, data+size);
	}
	
	bool pop_until_valid() {
		//we need at least 4B header + 4B length + 1B checksum
		for(;buffer_.size()>=9; buffer_.erase(buffer_.begin())) {
			if(valid())
				return true;
		}
		
		return false;
	}
	
	bool valid() const {
		const char *data = &buffer_[0];
		//we need at least 4B header + 4B length + 1B checksum
		if(
			buffer_.size()>=9 &&
			*(uint32_t*)(data+0) == htonl(0x02020202) &&
			size()+9<=buffer_.size() &&
			checksum(1)==buffer_[size()+8]
		)
			return true;
		
		return false;
	}
	
	uint32_t size() const {
		assert(buffer_.size()>=8);
		
		const char *data = &buffer_[0];
		return ntohl( *(uint32_t*)(data+4) );
	}
	
	void pop()
	{
		assert(valid());
		
		//size + header + checksum
		buffer_.erase(buffer_.begin(), buffer_.begin()+(size()+9));
	}
	
	std::string get_data() const {
		assert(valid());
		
		const char *data = &buffer_[0];
		return std::string(data+8, size());
	}
	
};


/* all methods that use the control channel (sopas) */
class Control : public TCP_Session {
	Any_Session::SIG_ON_DATA on_data_;		///< signal handler for incoming data
	const std::string remote_device_ip_;
	bool stream_started_;
	CoLaFrame frame_recv_;
	
	typedef boost::signals2::signal<bool (const std::string &) > SIG_ON_METHOD;
	
	struct SMethod {
		SIG_ON_METHOD callback_;
		boost::timed_mutex lock_;
		bool success_;
	};
	std::map<std::string, boost::shared_ptr<SMethod> > pending_methods_;
	
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
		
		ROS_DEBUG("got data for control");
		
		frame_recv_.add_data(data, size);
		
		while(frame_recv_.pop_until_valid()) {
			std::string response = frame_recv_.get_data();
			ROS_DEBUG("response: %s", response.c_str());
			
			if(cmp_cmds(response, "sAN ")) {
				size_t pos = response.find(' ');
				if(pos==std::string::npos) pos = response.size();
				const std::string method_name(response.begin(), response.begin()+pos);
				ROS_DEBUG("response from method %s", method_name.c_str());
				
				std::map<std::string, boost::shared_ptr<SMethod> >::iterator it = pending_methods_.find(method_name);
				if(it!=pending_methods_.end()) {
					it->second->success_ = it->second->callback_(std::string(response.begin()+pos, response.end()));
					it->second->lock_.unlock();
					pending_methods_.erase(it);
				}
				
			}
			else if(cmp_cmds(response, "sRA modFreq")) {
				assert(response.size()==1);
				if(response.size()>=1)
					ROS_INFO("modFreq=%d", (int)response[0]);
			}
			else if(cmp_cmds(response, "sWA modFreq")) {
				request("sRN modFreq");
			}
			else if(cmp_cmds(response, "sRA DeviceIdent")) {
				uint16_t len_name=0, len_version=0;
				if(
					response.size()>=4 &&
					(len_name=ntohs( *(uint16_t*)(&response[0]) ))+4<=response.size() &&
					(len_version=ntohs( *(uint16_t*)(&response[len_name+2]) ))+len_name+4<=response.size()
				) {
					ROS_INFO("DeviceIdent %s / %s",
						std::string(response.begin()+2, response.begin()+(2+len_name)).c_str(),
						std::string(response.begin()+(4+len_name), response.begin()+(4+len_version)).c_str()
						);
				}
				else
					ROS_ERROR("error while parsing DeviceIdent response: %s", response.c_str());
			}
			
			frame_recv_.pop();
		}
		
	}
	
	void request(const std::string &cmd) {
        CoLaFrame fr(cmd);
        ROS_DEBUG("Sending on %s", cmd.c_str());
        write(fr.finish());
	}
	
	template<class FCT>
	bool call_method(const std::string &methode_name, const FCT &callback, const std::string &data=std::string()) {
		boost::shared_ptr<SMethod> method(new SMethod);
		method->callback_.connect(callback);
		pending_methods_.insert( std::pair<std::string, boost::shared_ptr<SMethod> >(methode_name, method) );
		
		method->lock_.lock();
		request("sMN "+methode_name+" "+data);
		
		return method->lock_.timed_lock(boost::posix_time::seconds(1)) && method->success_;
	}
	
	
	//callbacks of methods
	bool meth_GetBlobClientConfig(const std::string &data) {
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
		on_data_.connect( boost::bind(&Control::on_data, this, _1, _2, _3) );
	}
	
    /* establish the control channel to the device */
    bool open() {
        ROS_DEBUG("Connecting to device...");
        
        if(!connect(remote_device_ip_, "2112")) {
            ROS_ERROR("Error on connecting to %s", remote_device_ip_.c_str());
            return false;
		}
		
		//read device identification (to verify version)
        request("sRN DeviceIdent");
        //read parameters
        request("sRN modFreq");
        
        ROS_DEBUG("done.");
        return true;
    }
    
    void setModFreq(const EIlluminationCode illumination_code) {
        ROS_DEBUG("Setting modulation frequency to %d", (int)illumination_code);
        
		request("sWN modFreq "+std::string(1, (char)illumination_code));
	}

    /* Tells the device that there is a streaming channel by invoking a
     * method named GetBlobClientConfig.
     */
    bool initStream() {
        ROS_DEBUG("Initializing streaming...");
        return call_method("GetBlobClientConfig", boost::bind(&Control::meth_GetBlobClientConfig, this, _1));
	}

    /* Start streaming the data by calling the "PLAYSTART" method on the
     * device and sending a "Blob request" afterwards.
     */
    bool startStream() {   
		if(stream_started_) return true;
		     
        return call_method("PLAYSTART", boost::bind(&Control::meth_PLAYSTART, this, _1));
	}
            
        
    /* Stops the data stream. */
    bool stopStream() { 
        if(!stream_started_) return true;
		     
        return call_method("PLAYSTOP", boost::bind(&Control::meth_PLAYSTOP, this, _1));
	}
	
	/* Call: SetAccessMode to change operation mode */
	bool setAccessMode(const uint8_t mode, const uint32_t hash) {
		std::string data(5, (char)0);
		*((uint8_t*) (data.c_str()+0)) = mode;
		*((uint32_t*)(data.c_str()+1)) = htonl(hash);
		
        return call_method("SetAccessMode", boost::bind(&Control::meth_SetAccessMode, this, _1), data);
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
