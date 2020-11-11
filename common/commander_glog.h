#ifndef _COMMANDER_GLOG_H
#define _COMMANDER_GLOG_H

#include <glog/logging.h>
#include <glog/raw_logging.h>
#include <stdlib.h>

//配置输出日志的目录，以下配置将log文件保存到可执行文件上一层的log目录下
#define LOGDIR "../log"
#define MKDIR "mkdir -p " LOGDIR

class CommanderGlog
{
public:
    //GLOG配置：
    CommanderGlog(char* argv)
    {
        int ret = system(MKDIR);
	ret = ret;
        google::InitGoogleLogging(argv);
        //设置级别等于或高于 google::ERROR 的日志同时输出到屏幕
        google::SetStderrLogging(google::ERROR);
        //设置输出到屏幕的日志显示相应颜色
        FLAGS_colorlogtostderr=true;
        //设置VLOG自定义log打印级别，只有当小于等于下值的VLOG才会被打印
        FLAGS_v = 10;

        //以下两种方式二选一：
        //1.简单设置log文件的存储路径
        FLAGS_log_dir = LOGDIR;
        //2.分别单独设置文件存储路径及文件名前缀 
        //单独设置 google::INFO 级别的日志存储路径和文件名前缀 
        // google::SetLogDestination(google::INFO,LOGDIR"/INFO_");
        // //单独设置 google::WARNING 级别的日志存储路径和文件名前缀
        // google::SetLogDestination(google::ERROR,LOGDIR"/ERROR_");
        // //单独设置 google::ERROR 级别的日志存储路径和文件名前缀
        // google::SetLogDestination(google::WARNING,LOGDIR"/WARNING_");

        //缓冲日志输出，默认为30秒，此处改为立即输出
        FLAGS_logbufsecs = 0;
        //最大日志大小为 100MB,当超过100MByte时会重新输出至一个新的log文件
        FLAGS_max_log_size = 100;
        //当磁盘被写满时，停止日志输出
        FLAGS_stop_logging_if_full_disk = true;
        //设置文件名扩展，如平台名或其它需要区分的信息  
        //google::SetLogFilenameExtension("x86_");
        //捕捉 core dumped
        google::InstallFailureSignalHandler();
        //默认捕捉 SIGSEGV 信号信息输出会输出到 stderr，可以通过下面的方法自定义输出方式    
        google::InstallFailureWriter(&(CommanderGlog::SignalHandle));
    };

    //当捕捉到 SIGSEGV 信号，将信息输出到单独的文件和 LOG(ERROR)
    static void SignalHandle(const char* data, int size)
    {
        std::string str = std::string(data,size);
        LOG(ERROR)<<str;
    }

    //GLOG内存清理：
    ~CommanderGlog()
    {
        google::ShutdownGoogleLogging();
    };
};

#endif