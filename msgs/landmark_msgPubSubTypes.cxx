// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file landmark_msgPubSubTypes.cpp
 * This header file contains the implementation of the serialization functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

#include "landmark_msgPubSubTypes.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

landmark_msgPubSubType::landmark_msgPubSubType() {
    setName("landmark_msg");
    m_typeSize = (uint32_t)landmark_msg::getMaxCdrSerializedSize() + 4 /*encapsulation*/;
    m_isGetKeyDefined = landmark_msg::isKeyDefined();
    m_keyBuffer = (unsigned char*)malloc(landmark_msg::getKeyMaxCdrSerializedSize()>16 ? landmark_msg::getKeyMaxCdrSerializedSize() : 16);
}

landmark_msgPubSubType::~landmark_msgPubSubType() {
    if(m_keyBuffer!=nullptr)
        free(m_keyBuffer);
}

bool landmark_msgPubSubType::serialize(void *data, SerializedPayload_t *payload) {
    landmark_msg *p_type = (landmark_msg*) data;
    eprosima::fastcdr::FastBuffer fastbuffer((char*) payload->data, payload->max_size); // Object that manages the raw buffer.
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
            eprosima::fastcdr::Cdr::DDS_CDR); // Object that serializes the data.
    payload->encapsulation = ser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    // Serialize encapsulation
    ser.serialize_encapsulation();

    try
    {
        p_type->serialize(ser); // Serialize the object:
    }
    catch(eprosima::fastcdr::exception::NotEnoughMemoryException& /*exception*/)
    {
        return false;
    }

    payload->length = (uint32_t)ser.getSerializedDataLength(); //Get the serialized length
    return true;
}

bool landmark_msgPubSubType::deserialize(SerializedPayload_t* payload, void* data) {
    landmark_msg* p_type = (landmark_msg*) data; 	//Convert DATA to pointer of your type
    eprosima::fastcdr::FastBuffer fastbuffer((char*)payload->data, payload->length); // Object that manages the raw buffer.
    eprosima::fastcdr::Cdr deser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
            eprosima::fastcdr::Cdr::DDS_CDR); // Object that deserializes the data.
    // Deserialize encapsulation.
    deser.read_encapsulation();
    payload->encapsulation = deser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;

    try
    {
        p_type->deserialize(deser); //Deserialize the object:
    }
    catch(eprosima::fastcdr::exception::NotEnoughMemoryException& /*exception*/)
    {
        return false;
    }

    return true;
}

std::function<uint32_t()> landmark_msgPubSubType::getSerializedSizeProvider(void* data) {
    return [data]() -> uint32_t
    {
        return (uint32_t)type::getCdrSerializedSize(*static_cast<landmark_msg*>(data)) + 4 /*encapsulation*/;
    };
}

void* landmark_msgPubSubType::createData() {
    return (void*)new landmark_msg();
}

void landmark_msgPubSubType::deleteData(void* data) {
    delete((landmark_msg*)data);
}

bool landmark_msgPubSubType::getKey(void *data, InstanceHandle_t* handle) {
    if(!m_isGetKeyDefined)
        return false;
    landmark_msg* p_type = (landmark_msg*) data;
    eprosima::fastcdr::FastBuffer fastbuffer((char*)m_keyBuffer,landmark_msg::getKeyMaxCdrSerializedSize()); 	// Object that manages the raw buffer.
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS); 	// Object that serializes the data.
    p_type->serializeKey(ser);
    if(landmark_msg::getKeyMaxCdrSerializedSize()>16)	{
        m_md5.init();
        m_md5.update(m_keyBuffer,(unsigned int)ser.getSerializedDataLength());
        m_md5.finalize();
        for(uint8_t i = 0;i<16;++i)    	{
            handle->value[i] = m_md5.digest[i];
        }
    }
    else    {
        for(uint8_t i = 0;i<16;++i)    	{
            handle->value[i] = m_keyBuffer[i];
        }
    }
    return true;
}

