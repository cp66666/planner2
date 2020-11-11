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
 * @file process_heartbeat.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _PROCESS_HEARTBEAT_H_
#define _PROCESS_HEARTBEAT_H_

// TODO Poner en el contexto.

#include <stdint.h>
#include <array>
#include <string>
#include <vector>
#include <map>
#include <bitset>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(process_heartbeat_SOURCE)
#define process_heartbeat_DllAPI __declspec( dllexport )
#else
#define process_heartbeat_DllAPI __declspec( dllimport )
#endif // process_heartbeat_SOURCE
#else
#define process_heartbeat_DllAPI
#endif
#else
#define process_heartbeat_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}

/*!
 * @brief This class represents the structure Msg_Heartbeat defined by the user in the IDL file.
 * @ingroup PROCESS_HEARTBEAT
 */
class Msg_Heartbeat
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport Msg_Heartbeat();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~Msg_Heartbeat();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object Msg_Heartbeat that will be copied.
     */
    eProsima_user_DllExport Msg_Heartbeat(const Msg_Heartbeat &x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object Msg_Heartbeat that will be copied.
     */
    eProsima_user_DllExport Msg_Heartbeat(Msg_Heartbeat &&x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object Msg_Heartbeat that will be copied.
     */
    eProsima_user_DllExport Msg_Heartbeat& operator=(const Msg_Heartbeat &x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object Msg_Heartbeat that will be copied.
     */
    eProsima_user_DllExport Msg_Heartbeat& operator=(Msg_Heartbeat &&x);

    /*!
     * @brief This function sets a value in member pid
     * @param _pid New value for member pid
     */
    inline eProsima_user_DllExport void pid(int32_t _pid)
    {
        m_pid = _pid;
    }

    /*!
     * @brief This function returns the value of member pid
     * @return Value of member pid
     */
    inline eProsima_user_DllExport int32_t pid() const
    {
        return m_pid;
    }

    /*!
     * @brief This function returns a reference to member pid
     * @return Reference to member pid
     */
    inline eProsima_user_DllExport int32_t& pid()
    {
        return m_pid;
    }
    /*!
     * @brief This function copies the value in member moduleName
     * @param _moduleName New value to be copied in member moduleName
     */
    inline eProsima_user_DllExport void moduleName(const std::string &_moduleName)
    {
        m_moduleName = _moduleName;
    }

    /*!
     * @brief This function moves the value in member moduleName
     * @param _moduleName New value to be moved in member moduleName
     */
    inline eProsima_user_DllExport void moduleName(std::string &&_moduleName)
    {
        m_moduleName = std::move(_moduleName);
    }

    /*!
     * @brief This function returns a constant reference to member moduleName
     * @return Constant reference to member moduleName
     */
    inline eProsima_user_DllExport const std::string& moduleName() const
    {
        return m_moduleName;
    }

    /*!
     * @brief This function returns a reference to member moduleName
     * @return Reference to member moduleName
     */
    inline eProsima_user_DllExport std::string& moduleName()
    {
        return m_moduleName;
    }

    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(const Msg_Heartbeat& data, size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr &cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr &cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr &cdr) const;

private:
    int32_t m_pid;
    std::string m_moduleName;
};

#endif // _PROCESS_HEARTBEAT_H_