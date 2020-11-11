#include "dds_participant.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"

#include "MsgModule.h"
#include "MsgModulePubSubTypes.h"
#include "sys/time.h"
#include <unistd.h>

using namespace std;
using namespace cmdrDDS;

MsgNameSpace::Msg_B pubmsg;
DdsPublisher<MsgNameSpace::Msg_B, MsgNameSpace::Msg_BPubSubType> mPublisher;
DdsSubscriber<MsgNameSpace::Msg_B, MsgNameSpace::Msg_BPubSubType> mSubscriber;

bool canbeOver = false;
/**
 * void onMessageReceived(MsgNameSpace::Msg_B *message)
 * @brief Callback function for subscriber.
 **/
void onMessageReceived(MsgNameSpace::Msg_B *message)
{
    static unsigned long cnt = 0;
    cout << "--Received Message: " << cnt << "\n"
         << "   msgA.x = " << message->msgA().x() << "\n"
         << "   index  = " << message->index() << "\n";
    ++cnt;
    if (cnt >= 10000)
    {
        canbeOver = true;
    }
}
void publishRun()
{
    uint32_t cnt = 0;
    while (1)
    {
        pubmsg.index(cnt);
        pubmsg.msgA().x(cnt % 100);
        // Publish message.
        mPublisher.PublishDds(&pubmsg);
        // Judging if over.
        if (canbeOver)
            break;
        cnt++;
        usleep(500);
    }
}

int main(int argc, char **argv)
{
    // Declare participant for publisher.
    DdsParticipant mParticipant;
    // Declare participant for subscriber.
    DdsParticipant mParticipant2;
    // Register topic type in participant.
    mParticipant.Create("idlmodule", 100);
    mParticipant2.Create("idlmodule", 100);
    // Create publisher and subscriber.
    mPublisher.Create(mParticipant, "Msg_B_message", 1);
    mSubscriber.Create(mParticipant2, "Msg_B_message", 1, onMessageReceived);

    // Run thread publishRun().
    std::thread t(&publishRun);
    t.join();
}
