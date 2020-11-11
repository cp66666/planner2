#include "dds_participant.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "pic.h"
#include "picPubSubTypes.h"
#include "sys/time.h"
#include <unistd.h>

using namespace std;
using namespace cmdrDDS;

#define WIDTH 1920
#define HEIGHT 1080

pic pubmsg;

DdsPublisher<pic, picPubSubType> mPublisher;
DdsSubscriber<pic, picPubSubType> mSubscriber;

bool canbeOver = false;
double averagetime = 0.0;
double meantime = 0.0;
double maxtime = 0.0;
double mintime = 0.0;
vector<double> timings;

/**
 * void onMessageReceived(pic *message)
 * @brief Callback function for subscriber.
 **/
void onMessageReceived(pic *message)
{
    static double totaltime;
    struct timeval mTimePre2;
    struct timeval mTime2;
    struct timezone mZone2;

    mTimePre2.tv_sec = message->tv_sec();
    mTimePre2.tv_usec = message->tv_usec();
    gettimeofday(&mTime2, &mZone2);

    unsigned long stmp = mTime2.tv_sec - mTimePre2.tv_sec;
    unsigned long ustmp;
    double tt;

    if (stmp > 0)
    {
        ustmp = 1000000 - mTimePre2.tv_usec + mTime2.tv_usec;
        stmp = 0;
    }
    else
    {
        ustmp = mTime2.tv_usec - mTimePre2.tv_usec;
    }
    // Get receiving period time.
    tt = stmp + ustmp * 0.000001;
    // Add current period to vector timings.
    timings.push_back(tt);
    // Total time.
    totaltime += tt;

    cout << "\033[?25l";
    cout << "Receiving Index " << message->index() << " Time period: " << tt << "s\r";
    // If index == 10000, 500us * 10000
    if (message->index() >= 10000)
    {
        cout << "\033[?25h";
        cout << endl;
        sort(timings.begin(), timings.end());
        averagetime = totaltime / 10000.0;
        meantime = timings.at(5000);
        maxtime = timings.at(timings.size() - 1);
        mintime = timings.at(0);
        canbeOver = true;
    }
}
/**
 * Thread publishRun().
 * @brief Publish topic message every 500us until canbeover equals true.
 **/
void publishRun()
{
    uint32_t cnt = 0;

    while (1)
    {
        // Get system time.
        struct timeval mTime;
        struct timezone mZone;
        gettimeofday(&mTime, &mZone);
        // Set message time value.
        pubmsg.tv_sec(mTime.tv_sec);
        pubmsg.tv_usec(mTime.tv_usec);
        pubmsg.index(++(pubmsg.index()));
        // Publish message.
        mPublisher.PublishDds(&pubmsg);
        // Judging if over.
        if (canbeOver == true)
            break;
        usleep(500);
    }
}
int main(int argc, char **argv)
{
    // Declare topic raw data.
    array<array<char, HEIGHT>, WIDTH> raw;
    for (int i = 0; i < HEIGHT; i++)
        for (int j = 0; j < WIDTH; j++)
            raw[i][j] = i % 127;

    // Set publish message data.
    pubmsg.img(raw);
    // Declare participant for publisher.
    DdsParticipant mParticipant;
    // Declare participant for subscriber.
    DdsParticipant mParticipant2;
    // Register topic type in participant.
    mParticipant.Create("Large_Pic", 100);
    mParticipant2.Create("Large_Pic", 100);
    // Create publisher and subscriber.
    mPublisher.Create(mParticipant, "TopicCameraImg", 1, cmdrDDS::ASYNC_PUBLISH_MODE);
    mSubscriber.Create(mParticipant2, "TopicCameraImg", 1, onMessageReceived);

    cout << "Testing...\n";
    cout << "Message pic \n"
         << "\tstruct{\n"
         << "\tchar[" << WIDTH << "][" << HEIGHT << "]];\n"
         << "\tunsigned long tv_sec;\n"
         << "\tunsigned long tv_usec;\n"
         << "\t}"
         << endl;
    // Run thread publishRun().
    std::thread t(&publishRun);
    t.join();
    cout << "Test Over.\n"
         << "Average Time\tMean Time\tMax Time\tMin Time\n"
         << averagetime << "s\t"
         << meantime << "s\t"
         << maxtime << "s\t"
         << mintime << "s\t\n";
}
