/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2016 \n
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
 * \date Date of creation: 12/02/2016
 *
 * \brief
 *   Parser for CoLa frames and content
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

class DatatypeParserAbstract {
public:
    virtual size_t parse(const char *data_ptr, const size_t remaining_size)=0;
    virtual std::string generate()=0;
    virtual void set(const std::string &)=0;
    
    //accessors for diagnose info.
    virtual std::string name() const=0;
    virtual std::string str() const=0;
};

namespace DatatypeParserTemplate {
    
    template<class C>
    inline C ntoh(const C &data_in)
    {
        return data_in;
    }
    
    template<>
    inline uint16_t ntoh(const uint16_t &data_in)
    {
        return ntohs(data_in);
    }
    
    template<>
    inline int16_t ntoh(const int16_t &data_in)
    {
        return ntohs(data_in);
    }
    
    template<>
    inline uint32_t ntoh(const uint32_t &data_in)
    {
        return ntohl(data_in);
    }
    
    template<>
    inline int32_t ntoh(const int32_t &data_in)
    {
        return ntohl(data_in);
    }
    
    
    
    template<class C>
    inline C hton(const C &data_in)
    {
        return data_in;
    }
    
    template<>
    inline uint16_t hton(const uint16_t &data_in)
    {
        return htons(data_in);
    }
    
    template<>
    inline int16_t hton(const int16_t &data_in)
    {
        return htons(data_in);
    }
    
    template<>
    inline uint32_t hton(const uint32_t &data_in)
    {
        return htonl(data_in);
    }
    
    template<>
    inline int32_t hton(const int32_t &data_in)
    {
        return htonl(data_in);
    }
    
}

template<class T>
class DatatypeParser : public DatatypeParserAbstract {
protected:
    T *variable_;
    std::string name_;
    
public:
    DatatypeParser(T *variable, const std::string &name="") : variable_(variable), name_(name)
    {
        assert(variable_);
    }
    
    virtual size_t parse(const char *data_ptr, const size_t remaining_size) {
        if(remaining_size<sizeof(T)) {
            ROS_WARN("not able to parse variable");
            return 0; //we could not read variable
        }
        
        *variable_ = DatatypeParserTemplate::ntoh( *((T*)data_ptr) );
        
        return sizeof(T);
    }
    
    virtual std::string generate() {
        std::string r(sizeof(T), (char)0);
        
        *((T*)r.c_str()) = DatatypeParserTemplate::hton(*variable_);
        
        return r;
    }
    
    virtual void set(const std::string &data) {
        assert(variable_);
        *variable_ = boost::lexical_cast<T>(data);
    }
    
    virtual std::string str() const {
        assert(variable_);
        return boost::lexical_cast<std::string>(*variable_);
    }
    
    virtual std::string name() const {
        return name_;
    }
};

template<>
class DatatypeParser<std::string> : public DatatypeParserAbstract {
protected:
    std::string *variable_;
    std::string name_;
    
public:
    DatatypeParser(std::string *variable, const std::string &name="") : variable_(variable), name_(name)
    {
        assert(variable_);
    }
    
    virtual size_t parse(const char *data_ptr, const size_t remaining_size) {
        uint16_t len=0;
        size_t offset = DatatypeParser<uint16_t>(&len).parse(data_ptr, remaining_size);
        
        if(offset==0 || remaining_size-offset<len) {
            ROS_WARN("not able to parse flex string");
            return 0; //we could not read variable
        }
        
        variable_->resize(len);
        memcpy(&(*variable_)[0], data_ptr+offset, len);
        
        return offset+len;
    }
    
    virtual std::string generate() {
        assert(variable_->size()<(1<<16));
        uint16_t len=variable_->size();
        
        return DatatypeParser<uint16_t>(&len).generate() + *variable_;
    }
    
    virtual void set(const std::string &data) {
        assert(variable_);
        *variable_ = data;
    }
    
    virtual std::string str() const {
        assert(variable_);
        return *variable_;
    }
    
    virtual std::string name() const {
        return name_;
    }
};

template<class T>
class DatatypeParserEnum : public DatatypeParser<T> {
    std::map<std::string, T> map_named_;
    std::map<T, std::string> map_valued_;
public:
    DatatypeParserEnum(T *variable, const std::string &name="") : DatatypeParser<T>(variable, name)
    {
    }
    
    virtual void set(const std::string &data) {
        assert(this->variable_);
        typename std::map<std::string, T>::const_iterator it = map_named_.find(data);
        if(it!=map_named_.end())
            *this->variable_ = it->second;
        else
            *this->variable_ = boost::lexical_cast<T>(data);
    }
    
    DatatypeParserEnum &operator()(const std::string &key, const T &value)
    {
        map_named_[key] = value;
        map_valued_[value] = key;
        return *this;
    }
    
    virtual std::string str() const {
        assert(this->variable_);
        typename std::map<T, std::string>::const_iterator it = map_valued_.find(*this->variable_);
        if(it!=map_valued_.end())
            return it->second;
        return boost::lexical_cast<std::string>( (int) *this->variable_);
    }
};

#define COLA_ENUM(x) (#x, x)

class StructParser {
    std::vector<DatatypeParserAbstract*> elements_;
public:
    ~StructParser() {
        for(size_t i=0; i<elements_.size(); i++)
            delete elements_[i];
    }
    
    template<class T>
    StructParser &operator()(T *variable, const std::string &name="") {
        elements_.push_back(new DatatypeParser<T>(variable, name));
        return *this;
    }
    
    StructParser &operator()(DatatypeParserAbstract *variable) {
        elements_.push_back(variable);
        return *this;
    }
    
    bool parse(const char *data_ptr, const size_t remaining_size) {
        size_t offset=0;
        
        for(size_t i=0; i<elements_.size(); i++) {
            size_t shift = elements_[i]->parse(data_ptr+offset, remaining_size-offset);
            if(shift==0) { //error
                ROS_ERROR("failed to parse struct");
                return false;
            }
            offset+=shift;
        }
        
        return true;
    }
    
    std::string generate() {
        std::string r;
        for(size_t i=0; i<elements_.size(); i++)
            r += elements_[i]->generate();
        return r;
    }
    
    StructParser &operator>>(std::map<std::string, std::string> &info) {
        for(size_t i=0; i<elements_.size(); i++) {
            if(elements_[i]->name().size()>0)
                info[elements_[i]->name()] = elements_[i]->str();
        }
        return *this;
    }
};
