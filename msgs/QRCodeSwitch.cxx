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
 * @file QRCodeSwitch.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "QRCodeSwitch.h"

#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

qrcode_switch_msg::qrcode_switch_msg()
{
    m_detect_qrcode = 0;
}

qrcode_switch_msg::~qrcode_switch_msg()
{
}

qrcode_switch_msg::qrcode_switch_msg(const qrcode_switch_msg &x)
{
    m_detect_qrcode = x.m_detect_qrcode;
}

qrcode_switch_msg::qrcode_switch_msg(qrcode_switch_msg &&x)
{
    m_detect_qrcode = x.m_detect_qrcode;
}

qrcode_switch_msg& qrcode_switch_msg::operator=(const qrcode_switch_msg &x)
{
    m_detect_qrcode = x.m_detect_qrcode;
    
    return *this;
}

qrcode_switch_msg& qrcode_switch_msg::operator=(qrcode_switch_msg &&x)
{
    m_detect_qrcode = x.m_detect_qrcode;
    
    return *this;
}

size_t qrcode_switch_msg::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    return current_alignment - initial_alignment;
}

size_t qrcode_switch_msg::getCdrSerializedSize(const qrcode_switch_msg& data, size_t current_alignment)
{
    size_t initial_alignment = current_alignment;
            
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    return current_alignment - initial_alignment;
}

void qrcode_switch_msg::serialize(eprosima::fastcdr::Cdr &scdr) const
{
    scdr << m_detect_qrcode;
}

void qrcode_switch_msg::deserialize(eprosima::fastcdr::Cdr &dcdr)
{
    dcdr >> m_detect_qrcode;
}

size_t qrcode_switch_msg::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
	size_t current_align = current_alignment;
            

    return current_align;
}

bool qrcode_switch_msg::isKeyDefined()
{
    return false;
}

void qrcode_switch_msg::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
	(void) scdr;
	 
}