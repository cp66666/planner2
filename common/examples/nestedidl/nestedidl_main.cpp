#include "dds_participant.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"

#include "msg_inherit.h"
#include "msg_inheritPubSubTypes.h"
#include "sys/time.h"
#include <unistd.h>

using namespace std;
using namespace cmdrDDS;

msg_inherit pubmsg;
DdsPublisher<msg_inherit, msg_inheritPubSubType> mPublisher;
DdsSubscriber<msg_inherit, msg_inheritPubSubType> mSubscriber;

bool canbeOver = false;
/**
 * void onMessageReceived(msg_inherit *message)
 * @brief Callback function for subscriber.
 **/
void onMessageReceived(msg_inherit *message)
{
    static unsigned long cnt = 0;
    ++cnt;
    cout << "--Received Message: " << cnt << "\n"
         << "IMU:\n"
         << "   x= " << message->imu().x() << "\n"
         << "   y= " << message->imu().y() << "\n"
         << "   theta= " << message->imu().theta() << "\n";
    cout << "tv_sec: " << message->tv_sec() << "s\n";
    cout << "tv_usec: " << (message->tv_usec() * 0.000001) << "s\n";
    if (cnt >= 10000)
    {
        canbeOver = true;
        cout << "\033[?25h";
    }
}
void publishRun()
{
    uint32_t cnt = 0;
    while (1)
    {
        cnt++;
        // Get system time.
        struct timeval mTime;
        struct timezone mZone;
        gettimeofday(&mTime, &mZone);
        // Set message time value.
        pubmsg.tv_sec(mTime.tv_sec);
        pubmsg.tv_usec(mTime.tv_usec);

        pubmsg.imu().x(0.5 + cnt * 0.0001);
        pubmsg.imu().y(-0.5 - cnt * 0.0001);
        pubmsg.imu().theta(0.01 * cnt);
        // Publish message.
        mPublisher.PublishDds(&pubmsg);
        // Judging if over.
        if (canbeOver)
            break;
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
    mParticipant.Create("nestedidl", 100);
    mParticipant2.Create("nestedidl", 100);
    // Create publisher and subscriber.
    mPublisher.Create(mParticipant, "msg_inherit_message", 1);
    mSubscriber.Create(mParticipant2, "msg_inherit_message", 1, onMessageReceived);

    // Run thread publishRun().
    std::thread t(&publishRun);
    t.join();
}
