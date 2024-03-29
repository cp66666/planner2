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
 * @file pic.cpp
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifdef _WIN32
// Remove linker warning LNK4221 on Visual Studio
namespace { char dummy; }
#endif

#include "pic.h"
#include <fastcdr/Cdr.h>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

#include <utility>

pic::pic()
{
    memset(&m_img, 0, (1920 * 1080) * 1);
    m_tv_sec = 0;

    m_tv_usec = 0;

    m_index = 0;


}

pic::~pic()
{
}

pic::pic(const pic &x)
{
    m_img = x.m_img;
    m_tv_sec = x.m_tv_sec;
    m_tv_usec = x.m_tv_usec;
    m_index = x.m_index;
}

pic::pic(pic &&x)
{
    m_img = std::move(x.m_img);
    m_tv_sec = x.m_tv_sec;
    m_tv_usec = x.m_tv_usec;
    m_index = x.m_index;
}

pic& pic::operator=(const pic &x)
{
    m_img = x.m_img;
    m_tv_sec = x.m_tv_sec;
    m_tv_usec = x.m_tv_usec;
    m_index = x.m_index;

    return *this;
}

pic& pic::operator=(pic &&x)
{
    m_img = std::move(x.m_img);
    m_tv_sec = x.m_tv_sec;
    m_tv_usec = x.m_tv_usec;
    m_index = x.m_index;

    return *this;
}

size_t pic::getMaxCdrSerializedSize(size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += ((1920 * 1080) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

size_t pic::getCdrSerializedSize(const pic& data, size_t current_alignment)
{
    (void)data;
    size_t initial_alignment = current_alignment;

    current_alignment += ((1920 * 1080) * 1) + eprosima::fastcdr::Cdr::alignment(current_alignment, 1);

    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);


    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);



    return current_alignment - initial_alignment;
}

void pic::serialize(eprosima::fastcdr::Cdr &scdr) const
{
    scdr << m_img;
    scdr << m_tv_sec;
    scdr << m_tv_usec;
    scdr << m_index;
}

void pic::deserialize(eprosima::fastcdr::Cdr &dcdr)
{
    dcdr >> m_img;
    dcdr >> m_tv_sec;
    dcdr >> m_tv_usec;
    dcdr >> m_index;
}

size_t pic::getKeyMaxCdrSerializedSize(size_t current_alignment)
{
	size_t current_align = current_alignment;
            





    return current_align;
}

bool pic::isKeyDefined()
{
    return false;
}

void pic::serializeKey(eprosima::fastcdr::Cdr &scdr) const
{
	(void) scdr;
	 
	 
	 
	 
}