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
 * @file downgoing_msg.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "downgoing_msg.h"

#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

commander_msg::DowngoingMsg::DowngoingMsg()
{
}

commander_msg::DowngoingMsg::~DowngoingMsg()
{
}

commander_msg::DowngoingMsg::DowngoingMsg(const DowngoingMsg &x)
{
    m_downgoing_msg = x.m_downgoing_msg;
}

commander_msg::DowngoingMsg::DowngoingMsg(DowngoingMsg &&x)
{
    m_downgoing_msg = std::move(x.m_downgoing_msg);
}

commander_msg::DowngoingMsg& commander_msg::DowngoingMsg::operator=(const DowngoingMsg &x)
{
    m_downgoing_msg = x.m_downgoing_msg;
    
    return *this;
}

commander_msg::DowngoingMsg& commander_msg::DowngoingMsg::operator=(DowngoingMsg &&x)
{
    m_downgoing_msg = std::move(x.m_downgoing_msg);
    
    return *this;
}

size_t commander_msg::DowngoingMsg::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + 10000 + 1;

    return current_alignment - initial_alignment;
}

size_t commander_msg::DowngoingMsg::getCdrSerializedSize(const commander_msg::DowngoingMsg& data, size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4) + data.downgoing_msg().size() + 1;

    return current_alignment - initial_alignment;
}

void commander_msg::DowngoingMsg::serialize(eprosima::fastcdr::Cdr &scdr) const
{
    scdr << m_downgoing_msg;
}

void commander_msg::DowngoingMsg::deserialize(eprosima::fastcdr::Cdr &dcdr)
{
    dcdr >> m_downgoing_msg;
}

size_t commander_msg::DowngoingMsg::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
	size_t current_align = current_alignment;
            

    return current_align;
}

bool commander_msg::DowngoingMsg::isKeyDefined()
{
    return false;
}

void commander_msg::DowngoingMsg::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
	(void) scdr;
	 
}
