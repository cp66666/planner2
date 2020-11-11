#include "dds_participant.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "dds_msg_monitor.h"
#include <unistd.h>
#include <thread>

void help()
{
    cout << "Usage help: ";
    cout << "cmdrmonitor <id>\n";
    cout << "[optional]--id: The domain id of monitor <0,232>. Default is 100.\n";
}
bool isNum(string str)
{
    for (int i = 0; i < str.size(); i++)
    {
        int tmp = (int)str[i];
        if (tmp >= 48 && tmp <= 57)
        {
            continue;
        }
        else
        {
            return false;
        }
    }
    return true;
}
int main(int argc, char **argv)
{
    std::string id;
    int domainID = 102;
    if (argc > 1)
    {
        if (strcmp(argv[1], "--h") == 0 || strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0)
        {
            help();
            return 0;
        }
        else
        {
            id = string(argv[1]);
            if (isNum(id))
                domainID = std::atoi(id.c_str());
            else
            {
                cout << "Wrong arguments. Please enter a right domain id.\n";
                help();
                return 0;
            }
        }
    }

    eprosima::fastrtps::ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = domainID;
    PParam.rtps.setName("Monitor");
    eprosima::fastrtps::Participant *mp = eprosima::fastrtps::Domain::createParticipant(PParam);

    cmdrDDS::CustomParticipantListener *listener = new cmdrDDS::CustomParticipantListener();
    // Pass the listener on participant creation.
    eprosima::fastrtps::Participant *participant = eprosima::fastrtps::Domain::createParticipant(mp->getAttributes(), listener);
    cout << "In Domain [" << domainID << "]" << endl;
    while (true)
    {
        listener->printmsgs();
        usleep(100000);
    }
}
