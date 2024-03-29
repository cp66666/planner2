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
 * @file MsgModulePubSubTypes.cpp
 * This header file contains the implementation of the serialization functions.
 *
 * This file was generated by the tool fastcdrgen.
 */


#include <fastcdr/FastBuffer.h>
#include <fastcdr/Cdr.h>

#include "MsgModulePubSubTypes.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

namespace MsgNameSpace
{
    Msg_APubSubType::Msg_APubSubType()
    {
        setName("MsgNameSpace::Msg_A");
        m_typeSize = static_cast<uint32_t>(Msg_A::getMaxCdrSerializedSize()) + 4 /*encapsulation*/;
        m_isGetKeyDefined = Msg_A::isKeyDefined();
        size_t keyLength = Msg_A::getKeyMaxCdrSerializedSize()>16 ? Msg_A::getKeyMaxCdrSerializedSize() : 16;
        m_keyBuffer = reinterpret_cast<unsigned char*>(malloc(keyLength));
        memset(m_keyBuffer, 0, keyLength);
    }

    Msg_APubSubType::~Msg_APubSubType()
    {
        if(m_keyBuffer!=nullptr)
            free(m_keyBuffer);
    }

    bool Msg_APubSubType::serialize(void *data, SerializedPayload_t *payload)
    {
        Msg_A *p_type = static_cast<Msg_A*>(data);
        eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->max_size); // Object that manages the raw buffer.
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

        payload->length = static_cast<uint32_t>(ser.getSerializedDataLength()); //Get the serialized length
        return true;
    }

    bool Msg_APubSubType::deserialize(SerializedPayload_t* payload, void* data)
    {
        Msg_A* p_type = static_cast<Msg_A*>(data); //Convert DATA to pointer of your type
        eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->length); // Object that manages the raw buffer.
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

    std::function<uint32_t()> Msg_APubSubType::getSerializedSizeProvider(void* data)
    {
        return [data]() -> uint32_t
        {
            return static_cast<uint32_t>(type::getCdrSerializedSize(*static_cast<Msg_A*>(data))) + 4 /*encapsulation*/;
        };
    }

    void* Msg_APubSubType::createData()
    {
        return reinterpret_cast<void*>(new Msg_A());
    }

    void Msg_APubSubType::deleteData(void* data)
    {
        delete(reinterpret_cast<Msg_A*>(data));
    }

    bool Msg_APubSubType::getKey(void *data, InstanceHandle_t* handle, bool force_md5)
    {
        if(!m_isGetKeyDefined)
            return false;
        Msg_A* p_type = static_cast<Msg_A*>(data);
        eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(m_keyBuffer),Msg_A::getKeyMaxCdrSerializedSize());     // Object that manages the raw buffer.
        eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS);     // Object that serializes the data.
        p_type->serializeKey(ser);
        if(force_md5 || Msg_A::getKeyMaxCdrSerializedSize()>16)    {
            m_md5.init();
            m_md5.update(m_keyBuffer, static_cast<unsigned int>(ser.getSerializedDataLength()));
            m_md5.finalize();
            for(uint8_t i = 0;i<16;++i)        {
                handle->value[i] = m_md5.digest[i];
            }
        }
        else    {
            for(uint8_t i = 0;i<16;++i)        {
                handle->value[i] = m_keyBuffer[i];
            }
        }
        return true;
    }

    Msg_BPubSubType::Msg_BPubSubType()
    {
        setName("MsgNameSpace::Msg_B");
        m_typeSize = static_cast<uint32_t>(Msg_B::getMaxCdrSerializedSize()) + 4 /*encapsulation*/;
        m_isGetKeyDefined = Msg_B::isKeyDefined();
        size_t keyLength = Msg_B::getKeyMaxCdrSerializedSize()>16 ? Msg_B::getKeyMaxCdrSerializedSize() : 16;
        m_keyBuffer = reinterpret_cast<unsigned char*>(malloc(keyLength));
        memset(m_keyBuffer, 0, keyLength);
    }

    Msg_BPubSubType::~Msg_BPubSubType()
    {
        if(m_keyBuffer!=nullptr)
            free(m_keyBuffer);
    }

    bool Msg_BPubSubType::serialize(void *data, SerializedPayload_t *payload)
    {
        Msg_B *p_type = static_cast<Msg_B*>(data);
        eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->max_size); // Object that manages the raw buffer.
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

        payload->length = static_cast<uint32_t>(ser.getSerializedDataLength()); //Get the serialized length
        return true;
    }

    bool Msg_BPubSubType::deserialize(SerializedPayload_t* payload, void* data)
    {
        Msg_B* p_type = static_cast<Msg_B*>(data); //Convert DATA to pointer of your type
        eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->length); // Object that manages the raw buffer.
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

    std::function<uint32_t()> Msg_BPubSubType::getSerializedSizeProvider(void* data)
    {
        return [data]() -> uint32_t
        {
            return static_cast<uint32_t>(type::getCdrSerializedSize(*static_cast<Msg_B*>(data))) + 4 /*encapsulation*/;
        };
    }

    void* Msg_BPubSubType::createData()
    {
        return reinterpret_cast<void*>(new Msg_B());
    }

    void Msg_BPubSubType::deleteData(void* data)
    {
        delete(reinterpret_cast<Msg_B*>(data));
    }

    bool Msg_BPubSubType::getKey(void *data, InstanceHandle_t* handle, bool force_md5)
    {
        if(!m_isGetKeyDefined)
            return false;
        Msg_B* p_type = static_cast<Msg_B*>(data);
        eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(m_keyBuffer),Msg_B::getKeyMaxCdrSerializedSize());     // Object that manages the raw buffer.
        eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS);     // Object that serializes the data.
        p_type->serializeKey(ser);
        if(force_md5 || Msg_B::getKeyMaxCdrSerializedSize()>16)    {
            m_md5.init();
            m_md5.update(m_keyBuffer, static_cast<unsigned int>(ser.getSerializedDataLength()));
            m_md5.finalize();
            for(uint8_t i = 0;i<16;++i)        {
                handle->value[i] = m_md5.digest[i];
            }
        }
        else    {
            for(uint8_t i = 0;i<16;++i)        {
                handle->value[i] = m_keyBuffer[i];
            }
        }
        return true;
    }


} //End of namespace MsgNameSpace
