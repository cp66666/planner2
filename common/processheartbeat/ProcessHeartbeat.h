#include <stdio.h>
#include <string>
#include <unistd.h>

#include "dds_participant.h"
#include "dds_publisher.h"
#include "process_heartbeat.h"
#include "process_heartbeatPubSubTypes.h"
#include <thread>
#include <boost/bind.hpp>

class ProcessHeartbeat
{
public:
    ProcessHeartbeat(){};
    ProcessHeartbeat(
        const std::string &name,
        cmdrDDS::DdsParticipant &participant_ref);

    virtual ~ProcessHeartbeat();

public:
    int GetPID(void);
    void run();
    void stop();

private:
    std::string moduleName_;
    cmdrDDS::DdsPublisher<Msg_Heartbeat, Msg_HeartbeatPubSubType> publisher_;
    int domainID_;
    int pid_;
    bool need_stop_;
    bool need_pause_;
    std::thread backgroundThread_;
    Msg_Heartbeat msg_;
    uint32_t counter;
    void PublishHeartbeat();
};
