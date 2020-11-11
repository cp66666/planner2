#ifndef DDS_MONITOR_H_
#define DDS_MONITOR_H_

#include <string>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/Domain.h>
#include <fastrtps/rtps/reader/ReaderDiscoveryInfo.h>
#include <fastrtps/rtps/writer/WriterDiscoveryInfo.h>
#include <fastrtps/utils/eClock.h>
//#include <fastrtps/xmlparser/XMLProfileManager.h>

using namespace std;
namespace cmdrDDS
{

class CustomParticipantListener : public eprosima::fastrtps::ParticipantListener
{
    struct Msg_t
    {
        string topictype;
        vector<string> topics;
        int cnt;
    };
    /* Custom Listener onSubscriberDiscovery */
    void onSubscriberDiscovery(
        eprosima::fastrtps::Participant *participant,
        eprosima::fastrtps::rtps::ReaderDiscoveryInfo &&info) override
    {
        (void)participant;
        std::string name;
        int index = 0;
        int index2 = 0;
        switch (info.status)
        {
        case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::DISCOVERED_READER:
            /* Process the case when a new subscriber was found in the domain */
            name = info.info.typeName().to_string();
            index = containstypereader(name);
            if (index != -1)
            {
                index2 = containstopic(msgtypes_reader[index].topics, info.info.topicName().to_string());
                if (index2 == -1)
                {
                    msgtypes_reader[index].topics.push_back(info.info.topicName().to_string());
                }
            }
            else
            {
                Msg_t t;
                t.topictype = name;
                t.topics.push_back(info.info.topicName().to_string());
                t.cnt = 1;
                msgtypes_reader.push_back(t);
            }
            break;
        case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::CHANGED_QOS_READER:
            /* Process the case when a subscriber changed its QOS */
            break;
        case eprosima::fastrtps::rtps::ReaderDiscoveryInfo::REMOVED_READER:
            name = info.info.typeName().to_string();
            index = containstypereader(name);
            if (index != -1)
            {
                index2 = containstopic(msgtypes_reader[index].topics, info.info.topicName().to_string());
                if (index2 != -1)
                {
                    msgtypes_reader[index].topics.erase(msgtypes_reader[index].topics.begin() + index2);
                    if (msgtypes_reader[index].topics.empty())
                    {
                        msgtypes_reader.erase(msgtypes_reader.begin() + index);
                    }
                }
            }
            else
            {
                ;
            }
            break;
        }
    }

    /* Custom Listener onPublisherDiscovery */
    void onPublisherDiscovery(
        eprosima::fastrtps::Participant *participant,
        eprosima::fastrtps::rtps::WriterDiscoveryInfo &&info) override
    {
        (void)participant;
        std::string name;
        int index = 0;
        int index2 = 0;
        switch (info.status)
        {
        case eprosima::fastrtps::rtps::WriterDiscoveryInfo ::DISCOVERED_WRITER:
            name = info.info.typeName().to_string();
            index = containstypewriter(name);
            if (index != -1)
            {
                index2 = containstopic(msgtypes_writer[index].topics, info.info.topicName().to_string());
                if (index2 == -1)
                {
                    msgtypes_writer[index].topics.push_back(info.info.topicName().to_string());
                }
            }
            else
            {
                Msg_t t;
                t.topictype = name;
                t.topics.push_back(info.info.topicName().to_string());
                msgtypes_writer.push_back(t);
            }
            break;
        case eprosima::fastrtps::rtps::WriterDiscoveryInfo ::CHANGED_QOS_WRITER:
            /* Process the case when a publisher changed its QOS */
            break;
        case eprosima::fastrtps::rtps::WriterDiscoveryInfo ::REMOVED_WRITER:
            name = info.info.typeName().to_string();
            index = containstypewriter(name);
            if (index != -1)
            {
                index2 = containstopic(msgtypes_writer[index].topics, info.info.topicName().to_string());
                if (index2 != -1)
                {
                    msgtypes_writer[index].topics.erase(msgtypes_writer[index].topics.begin() + index2);
                    if (msgtypes_writer[index].topics.empty())
                    {
                        msgtypes_writer.erase(msgtypes_writer.begin() + index);
                    }
                }
            }
            else
            {
                ;
            }
            break;
        }
    }

public:
    void printmsgs()
    {
        static long cnt = 0;
        cnt++;
        cout << "\033[J\n";
        linecnt = 1;
        // printf("cnt=%ld\n", cnt);
        if (!msgtypes_writer.empty())
        {
            cout << "Publisher\n";
            linecnt++;
        }
        for (int i = 0; i < msgtypes_writer.size(); i++)
        {
            cout << "      " << msgtypes_writer[i].topictype << ": \n";
            linecnt++;
            printvec(msgtypes_writer[i].topics);
            cout << "\n";
            linecnt++;
        }
        if (!msgtypes_reader.empty())
        {
            cout << "Subscriber\n";
            linecnt++;
        }

        for (int i = 0; i < msgtypes_reader.size(); i++)
        {
            cout << "      " << msgtypes_reader[i].topictype << ": \n";
            linecnt++;
            printvec(msgtypes_reader[i].topics);
            cout << "\n";
            linecnt++;
        }
        for (; linecnt > 0; linecnt--)
        {
            cout << "\033[K";
            cout << "\033[1A";
        }
    }
    void printvec(vector<string> vec)
    {
        for (int i = 0; i < vec.size(); i++)
        {
            cout << "         └──";
            cout << vec[i] << endl;
            linecnt++;
        }
    }
    int containstypereader(string str)
    {
        if (msgtypes_reader.empty())
            return -1;

        for (int i = 0; i < msgtypes_reader.size(); i++)
        {
            if (!msgtypes_reader[i].topictype.compare(str))
            {
                return i;
            }
        }
        return -1;
    }
    int containstypewriter(string str)
    {
        if (msgtypes_writer.empty())
            return -1;

        for (int i = 0; i < msgtypes_writer.size(); i++)
        {
            if (!msgtypes_writer[i].topictype.compare(str))
            {
                return i;
            }
        }
        return -1;
    }
    int containstopic(vector<string> vec, string str)
    {
        if (vec.empty())
            return -1;

        for (int i = 0; i < vec.size(); i++)
        {
            if (vec[i] == str)
            {
                return i;
            }
        }
        return -1;
    }

private:
    vector<Msg_t> msgtypes_reader;
    vector<Msg_t> msgtypes_writer;
    int index = 0;
    int linecnt = 0;
    int DomainID = 0;
};

}; // namespace cmdrDDS

#endif /* end of DDS_MONITOR_H_*/
