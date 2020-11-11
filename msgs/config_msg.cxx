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
 * @file config_msg.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "config_msg.h"

#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

commander_msg::ConfigMsg::ConfigMsg()
{
}

commander_msg::ConfigMsg::~ConfigMsg()
{
}

commander_msg::ConfigMsg::ConfigMsg(const ConfigMsg &x)
{
    m_config_msg = x.m_config_msg;
}

commander_msg::ConfigMsg::ConfigMsg(ConfigMsg &&x)
{
    m_config_msg = std::move(x.m_config_msg);
}

commander_msg::ConfigMsg& commander_msg::ConfigMsg::operator=(const ConfigMsg &x)
{
    m_config_msg = x.m_config_msg;
    
    return *this;
}

commander_msg::ConfigMsg& commander_msg::ConfigMsg::operator=(ConfigMsg &&x)
{
    m_config_msg = std::move(x.m_config_msg);
    
    return *this;
}

size_t commander_msg::ConfigMsg::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + 10000 + 1;

    return current_alignment - initial_alignment;
}

size_t commander_msg::ConfigMsg::getCdrSerializedSize(const commander_msg::ConfigMsg& data, size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + data.config_msg().size() + 1;

    return current_alignment - initial_alignment;
}

void commander_msg::ConfigMsg::serialize(eprosima::fastcdr::Cdr &scdr) const
{
    scdr << m_config_msg;
}

void commander_msg::ConfigMsg::deserialize(eprosima::fastcdr::Cdr &dcdr)
{
    dcdr >> m_config_msg;
}

size_t commander_msg::ConfigMsg::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
	size_t current_align = current_alignment;
            

    return current_align;
}

bool commander_msg::ConfigMsg::isKeyDefined()
{
    return false;
}

void commander_msg::ConfigMsg::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
	(void) scdr;
	 
}
