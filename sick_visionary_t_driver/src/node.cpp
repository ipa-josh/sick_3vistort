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
 *   ROS node for SICK 3DCS driver
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
 
#include <sick_visionary_t_driver/driver.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ByteMultiArray.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_srvs/Trigger.h>

/// Flag whether invalid points are to be replaced by NaN
const bool SUPPRESS_INVALID_POINTS = true;

/// Distance code for data outside the TOF range
const uint16_t NARE_DISTANCE_VALUE = 0xffffU;


bool Driver_3DCS::read_param(const std::string &name, std::string &var) {
	return ros::param::get("~"+name, var);
}

class DriverNode {
	
	//member variables
	/// If true: prevents skipping of frames and publish everything, otherwise use newest data to publish to ROS world
	bool publish_all_;
	std::string frame_id_;
	
	ros::NodeHandle nh_;
	Driver_3DCS::Control *control_;
	image_transport::ImageTransport it_;
	image_transport::Publisher pub_depth_, pub_confidence_, pub_intensity_;
	ros::Publisher pub_camera_info_, pub_points_, pub_ios_, pub_scan_, pub_cart_;
	ros::ServiceServer srv_enable_depth_map_;
	ros::ServiceServer srv_enable_height_map_;
	ros::ServiceServer srv_enable_polar_scan_;
	ros::Timer timer_io_;
	bool enabled_depth_, enabled_polar_, enabled_cart_;
	
	double io_polling_interval_;
	
	boost::mutex mtx_data_;
	boost::shared_ptr<Driver_3DCS::Data> data_;
	
	
	//methods

	void thr_publish_frame() {
		mtx_data_.lock();
		publish_frame(*data_);
		mtx_data_.unlock();
	}

	void publish_frame(const Driver_3DCS::Data &data) {
		bool published_anything = false;
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = frame_id_;
		
		if(data.has_polar_data()) {
			
			if(pub_scan_.getNumSubscribers()>0) {
				published_anything = true;
    
				sensor_msgs::LaserScan msg;
				msg.range_min = 0.1;
				msg.range_max = 6.0;
				msg.angle_min = data.get_polar_startingAngle()*M_PI/180;
				msg.angle_increment = data.get_polar_stepAngle()*M_PI/180;
				msg.ranges = data.get_polar_distances();
				msg.intensities = data.get_polar_confidences();
				msg.angle_max = msg.angle_min + msg.angle_increment*msg.ranges.size();
				msg.header = header;
				
				pub_scan_.publish(msg);
			}
		}
		
		if(data.has_height_data()) {
			
			if(pub_cart_.getNumSubscribers()>0) {
				published_anything = true;
				
				typedef sensor_msgs::PointCloud2 PointCloud;
				
				// Allocate new point cloud message
				PointCloud::Ptr cloud_msg (new PointCloud);
				cloud_msg->header = header; // Use depth image time stamp
				cloud_msg->height = data.get_height_points().size();
				cloud_msg->width  = 1;
				cloud_msg->is_dense = true;
				cloud_msg->is_bigendian = false;

				cloud_msg->fields.resize (4);
				cloud_msg->fields[0].name = "x"; cloud_msg->fields[1].name = "y"; cloud_msg->fields[2].name = "z";
				cloud_msg->fields[3].name = "confidence";
				int offset = 0;
				// All offsets are *4, as all field data types are float32
				for (size_t d = 0; d < cloud_msg->fields.size (); ++d, offset += 4)
				{
					cloud_msg->fields[d].offset = offset;
					cloud_msg->fields[d].datatype = int(sensor_msgs::PointField::FLOAT32);
					cloud_msg->fields[d].count  = 1;
				}
				cloud_msg->point_step = offset;
				cloud_msg->row_step   = cloud_msg->point_step * cloud_msg->width;
				cloud_msg->data.resize (cloud_msg->height * cloud_msg->width * cloud_msg->point_step);

				for(size_t i = 0; i < data.get_height_points().size(); ++i)
				{
					memcpy (&cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[0].offset], &data.get_height_points()[i].x, sizeof (float));
					memcpy (&cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[1].offset], &data.get_height_points()[i].y, sizeof (float));
					memcpy (&cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[2].offset], &data.get_height_points()[i].z, sizeof (float));
					memcpy (&cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[3].offset], &data.get_height_points()[i].c, sizeof (float));
				}
				
				pub_cart_.publish(cloud_msg);
			}
		}
		
		if(data.has_depth_data()) {
			sensor_msgs::ImagePtr msg;
			
			if(pub_camera_info_.getNumSubscribers()>0) {
				published_anything = true;
				
				sensor_msgs::CameraInfo ci;
				ci.header = header;
				
				ci.height = data.getCameraParameters().height;
				ci.width  = data.getCameraParameters().width;
				
				ci.D.clear();
				ci.D.resize(5,0);
				ci.D[0] = data.getCameraParameters().k1;
				ci.D[1] = data.getCameraParameters().k2;
				
				for(int i=0; i<9; i++) ci.K[i]=0;
				ci.K[0] = data.getCameraParameters().fx;
				ci.K[4] = data.getCameraParameters().fy;
				ci.K[2] = data.getCameraParameters().cx;
				ci.K[5] = data.getCameraParameters().cy;
				
				for(int i=0; i<12; i++)
					ci.P[i] = 0;//data.getCameraParameters().cam2worldMatrix[i];
				//TODO:....
				ci.P[0] = data.getCameraParameters().fx;
				ci.P[5] = data.getCameraParameters().fy;
				ci.P[10] = 1;
				ci.P[2] = data.getCameraParameters().cx;
				ci.P[6] = data.getCameraParameters().cy;
					
				pub_camera_info_.publish(ci);
			}
			
			if(pub_depth_.getNumSubscribers()>0) {
				published_anything = true;
				
				msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, data.get_distance()).toImageMsg();
				msg->header = header;
				pub_depth_.publish(msg);
			}
			if(pub_confidence_.getNumSubscribers()>0) {
				published_anything = true;
				
				msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, data.get_confidence()).toImageMsg();
				msg->header = header;
				pub_confidence_.publish(msg);
			}
			if(pub_intensity_.getNumSubscribers()>0) {
				published_anything = true;
				
				msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1, data.get_intensity()).toImageMsg();
				msg->header = header;
				pub_intensity_.publish(msg);
			}
			if(pub_points_.getNumSubscribers()>0) {
				published_anything = true;
				
				typedef sensor_msgs::PointCloud2 PointCloud;
				
				// Allocate new point cloud message
				PointCloud::Ptr cloud_msg (new PointCloud);
				cloud_msg->header = header; // Use depth image time stamp
				cloud_msg->height = data.get_distance().rows;
				cloud_msg->width  = data.get_distance().cols;
				cloud_msg->is_dense = false;
				cloud_msg->is_bigendian = false;

				cloud_msg->fields.resize (5);
				cloud_msg->fields[0].name = "x"; cloud_msg->fields[1].name = "y"; cloud_msg->fields[2].name = "z";
				cloud_msg->fields[3].name = "confidence"; cloud_msg->fields[4].name = "intensity";
				int offset = 0;
				// All offsets are *4, as all field data types are float32
				for (size_t d = 0; d < cloud_msg->fields.size (); ++d, offset += 4)
				{
					cloud_msg->fields[d].offset = offset;
					cloud_msg->fields[d].datatype = (d<3) ? int(sensor_msgs::PointField::FLOAT32) : int(sensor_msgs::PointField::UINT16);
					cloud_msg->fields[d].count  = 1;
				}
				cloud_msg->point_step = offset;
				cloud_msg->row_step   = cloud_msg->point_step * cloud_msg->width;
				cloud_msg->data.resize (cloud_msg->height * cloud_msg->width * cloud_msg->point_step);

				const float f2rc = data.getCameraParameters().f2rc / 1000.f; // since f2rc is given in [mm], but the pcl message wants [m]
				
				uint16_t *pDepth, *pConf, *pInt;
				int cp=0;
				for(int i = 0; i < data.get_distance().rows; ++i)
				{
					pDepth = (uint16_t*)(data.get_distance()  .data + (data.get_distance()  .step*i) );
					pConf  = (uint16_t*)(data.get_confidence().data + (data.get_confidence().step*i) );
					pInt   = (uint16_t*)(data.get_intensity() .data + (data.get_intensity() .step*i) );
					
					for (int j = 0; j < data.get_distance().cols; ++j, ++cp)
					{
						if(pDepth[j]==0 || pDepth[j]==NARE_DISTANCE_VALUE) {
							const float bad_point = std::numeric_limits<float>::quiet_NaN();
							memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[0].offset], &bad_point, sizeof (float));
							memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[1].offset], &bad_point, sizeof (float));
							memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[2].offset], &bad_point, sizeof (float));
						}
						else {
							float xp = (data.getCameraParameters().cx - j) / data.getCameraParameters().fx;
							float yp = (data.getCameraParameters().cy - i) / data.getCameraParameters().fy;

							// Correction of the lens distortion
							float r2 = (xp * xp + yp * yp);
							float r4 = r2 * r2;
							float k = 1 + data.getCameraParameters().k1 * r2 + data.getCameraParameters().k2 * r4;
							xp = xp * k;
							yp = yp * k;

							// Calculation of the distances
							float s0 = std::sqrt(xp*xp + yp*yp + 1.f) * 1000.f;

							// Calculation of the Cartesian coordinates
							const float ox = xp * pDepth[j] / s0;
							const float oy = yp * pDepth[j] / s0;
							const float oz = pDepth[j] / s0 - f2rc;
							
							memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[0].offset], &ox, sizeof (float));
							memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[1].offset], &oy, sizeof (float));
							memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[2].offset], &oz, sizeof (float));
						}
						
						memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[3].offset], &pConf[j], sizeof (uint16_t));
						memcpy (&cloud_msg->data[cp * cloud_msg->point_step + cloud_msg->fields[4].offset], &pInt [j], sizeof (uint16_t));
					}
				}
				
				pub_points_.publish(cloud_msg);
			}
		}
		
		if(!published_anything) {
			if(control_) control_->stopStream();
		}
	}

	void _on_new_subscriber() {        
			ROS_DEBUG("Got new subscriber");

		if(!control_) return;

		bool r=control_->startStream(); // tell the device to start streaming data
		ROS_ASSERT(r);
	}

	void on_new_subscriber_ros(const ros::SingleSubscriberPublisher& pub) {        
			_on_new_subscriber();
	}

	void on_new_subscriber_it(const image_transport::SingleSubscriberPublisher& pub) {        
			_on_new_subscriber();
	}
	
	void on_enableDepth(const bool enabled) {
		if(enabled_depth_ == enabled) return;
		
		if(enabled) {
			pub_depth_       = it_.advertise("depth", 1,
				boost::bind(&DriverNode::on_new_subscriber_it, this, _1));
			pub_points_      = nh_.advertise<sensor_msgs::PointCloud2>("points", 2,
				boost::bind(&DriverNode::on_new_subscriber_ros, this, _1));
			pub_confidence_  = it_.advertise("confidence", 1,
				boost::bind(&DriverNode::on_new_subscriber_it, this, _1));
			pub_intensity_   = it_.advertise("intensity", 1,
				boost::bind(&DriverNode::on_new_subscriber_it, this, _1));
			pub_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1,
				boost::bind(&DriverNode::on_new_subscriber_ros, this, _1));
		}
		else {
			pub_depth_.shutdown();
			pub_points_.shutdown();
			pub_confidence_.shutdown();
			pub_intensity_.shutdown();
			pub_camera_info_.shutdown();
		}
		
		enabled_depth_ = enabled;
	}
	
	void on_enablePolar(const bool enabled) {
		if(enabled_polar_ == enabled) return;
		
		if(enabled)
			pub_scan_      = nh_.advertise<sensor_msgs::LaserScan>("scan", 2,
				boost::bind(&DriverNode::on_new_subscriber_ros, this, _1));
		else
			pub_scan_.shutdown();
		
		enabled_polar_ = enabled;
	}
	
	void on_enableCartesian(const bool enabled) {
		if(enabled_cart_ == enabled) return;
		
		if(enabled)
			pub_cart_      = nh_.advertise<sensor_msgs::PointCloud2>("cartesian", 2,
				boost::bind(&DriverNode::on_new_subscriber_ros, this, _1));
		else
			pub_cart_.shutdown();
		
		enabled_cart_ = enabled;
	}
	
	void setIOPollingInterval(const double interval) {
		io_polling_interval_ = interval;
		timer_io_.stop();
		pub_ios_.shutdown();
		
		if(io_polling_interval_>0) {
			timer_io_ = nh_.createTimer(ros::Duration(io_polling_interval_), boost::bind(&Driver_3DCS::Control::triggerIO, control_));
			pub_ios_ = nh_.advertise<std_msgs::ByteMultiArray>("ios", 1);
		}
	}
	
public:
	DriverNode(Driver_3DCS::Control *control, ros::NodeHandle &nh) :
		publish_all_(false), frame_id_("camera"), nh_(nh), control_(control),  it_(nh),
		enabled_depth_(false), enabled_polar_(false), enabled_cart_(false),
		io_polling_interval_(0.1)
	{
		bool r = control_->connect_IO_callback(boost::bind(&DriverNode::on_IO, this, _1));
		ROS_ASSERT(r);
	
		std::string channel = "NO_CHANGE";
		ros::param::get("~prevent_frame_skipping", publish_all_);
		ros::param::get("~frame_id", frame_id_);
		ros::param::get("~io_polling_interval", io_polling_interval_);
		ros::param::get("~channel", channel);
		
		setIOPollingInterval(io_polling_interval_);
		
		if(control_->isAGDevice()) {
			if(channel=="NO_CHANGE")
				;
			else if(channel=="DEPTHMAP")
				control_->enableDepthMap();
			else if(channel=="POLAR2D")
				control_->enablePolarScan();
			else if(channel=="CARTESIAN")
				control_->enableHeightMap();
			else
				ROS_ERROR("channel was set to '%s', allowed are NO_CHANGE, DEPTHMAP, POLAR2D, CARTESIAN", channel.c_str());
				
			srv_enable_depth_map_ = nh.advertiseService("enableDepthMap", &DriverNode::enableDepthMap, this);
			srv_enable_height_map_ = nh.advertiseService("enableHeightMap", &DriverNode::enableHeightMap, this);
			srv_enable_polar_scan_ = nh.advertiseService("enablePolarScan", &DriverNode::enablePolarScan, this);
		}
	}
	
	~DriverNode() {
		pub_depth_.shutdown();
		pub_confidence_.shutdown();
		pub_intensity_.shutdown();
		pub_camera_info_.shutdown();
		pub_points_.shutdown();
		pub_ios_.shutdown();
	}

	void on_frame(const boost::shared_ptr<Driver_3DCS::Data> &data) {
		//make ros interfaces public in dependence of information source
		on_enablePolar(data->has_polar_data());
		on_enableCartesian(data->has_height_data());
		on_enableDepth(data->has_depth_data());
		
		//update data in queue and
		//detach publishing data from network thread
		
		if(publish_all_ || mtx_data_.try_lock()) {
			if(publish_all_)
				mtx_data_.lock();
			data_ = data;
			mtx_data_.unlock();
			
			boost::thread thr(boost::bind(&DriverNode::thr_publish_frame, this));
		}
		else
			ROS_WARN("skipping frame");
	}
	
	bool enableDepthMap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
		if(!control_) return false;
		res.success = true;
		
		res.success &= control_->enableDepthMap();
		ROS_ASSERT(res.success);
		
		//request now one frame to check which data we have
		res.success &= control_->singleStep();
		ROS_ASSERT(res.success);
		
		return true;
	}
	
	bool enableHeightMap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
		if(!control_) return false;
		res.success = true;
		
		res.success &= control_->enableHeightMap();
		ROS_ASSERT(res.success);
		
		//request now one frame to check which data we have
		res.success &= control_->singleStep();
		ROS_ASSERT(res.success);
		
		return true;
	}
	
	bool enablePolarScan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
		if(!control_) return false;
		res.success = true;
		
		res.success &= control_->enablePolarScan();
		ROS_ASSERT(res.success);
		
		//request now one frame to check which data we have
		res.success &= control_->singleStep();
		ROS_ASSERT(res.success);
		
		return true;
	}
	
	bool on_IO(const std::string &data) {
		std_msgs::ByteMultiArray msg;
		std_msgs::MultiArrayDimension lay;
		
		lay.label="IO Values";
		lay.size = 6;
		lay.stride=1;
		msg.layout.dim.push_back(lay);
		
		msg.data.resize(6);
		for(int i=0; i<6; i++)
			msg.data[i] = control_->getControlVariables().IOValue[i];
			
		pub_ios_.publish(msg);
		return true;
	}
	
};
			
void on_diagnostic_status_changed(const Driver_3DCS::ControlVariables &var, const Driver_3DCS::DiagnoseInfo &info, ros::Publisher &pub) {
	diagnostic_updater::DiagnosticStatusWrapper diag;
	
	for(Driver_3DCS::DiagnoseInfo::const_iterator it=info.begin(); it!=info.end(); it++)
		if(it->first.size()>0 && it->first[0]!='_')	//check for non-private variables
			diag.add(it->first, it->second); 
	
	//health
	Driver_3DCS::Control::EThreeLevel lvl = (Driver_3DCS::Control::EThreeLevel)var.DataQualityLevel;
	if(var.TmpLvl<lvl) lvl = (Driver_3DCS::Control::EThreeLevel)var.TmpLvl;
	if(var.doutPinError!=0 || var.doutOverload)
		lvl = Driver_3DCS::Control::ERROR;
	
	if(var.doutPinError!=0) {
		for(int i=0; i<32; i++)
			if(var.doutPinError&(1<<i))
				diag.add("Out Pin "+boost::lexical_cast<std::string>(i), "short circuit");
	}
	
	switch(lvl) {
		case Driver_3DCS::Control::GOOD:
			diag.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
			break;
		case Driver_3DCS::Control::WARNING:
			diag.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Warning");
			break;
		case Driver_3DCS::Control::ERROR:
			diag.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Error");
			break;
		default:
			diag.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Invalid");
			break;
	}
	
	pub.publish( (diagnostic_msgs::DiagnosticStatus)diag );
}

int main(int argc, char **argv) {
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	ros::init(argc, argv, "driver_3DCS");
	ros::NodeHandle nh("~");
	
	bool r;
	boost::asio::io_service io_service;
	
	//default parameters
	std::string remote_device_ip="192.168.1.10";
	
	ros::param::get("~remote_device_ip", remote_device_ip);
	
	ros::Publisher diag_pub = nh.advertise<diagnostic_msgs::DiagnosticStatus>("/diagnostics", 1, true);
	
	boost::asio::io_service::work work(io_service); //prevent io service running out of work
	boost::thread thr(boost::bind(&boost::asio::io_service::run, &io_service));

	Driver_3DCS::Control control(io_service, remote_device_ip, boost::bind(&on_diagnostic_status_changed, _1, _2, diag_pub));
	r=control.open();
	ROS_ASSERT(r);
	r=control.read_parameters();
	ROS_ASSERT(r);
	
	r=control.initStream();
	ROS_ASSERT(r);
	
	DriverNode driver(&control, nh);
	
	Driver_3DCS::Streaming device(io_service, control.getControlVariables().blob_device_ip_addr, boost::lexical_cast<std::string>(control.getControlVariables().blob_tcp_port_api));
	device.getSignal().connect( boost::bind(&DriverNode::on_frame, &driver, _1) );
	r=device.openStream();
	ROS_ASSERT(r);
	
	//request now one frame to check which data we have
	r=control.singleStep();
	ROS_ASSERT(r);
	
	//wait til end of exec.
	ros::spin();

	io_service.stop();
	device.closeStream();
	control.close();

	return 0;
}
