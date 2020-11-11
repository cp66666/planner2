#ifndef _COMMON_HTTP_CONNECT_H_
#define _COMMON_HTTP_CONNECT_H_

#include <iostream>

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "base64coder.h"

using namespace std;
class HttpConnect
{
public:
    HttpConnect()
    {

    }
    virtual ~HttpConnect()
    {

    }
    void socketHttp(std::string host, int port, std::string request)
    {
        int sockfd;
        struct sockaddr_in address;
        struct hostent *server;

        sockfd = socket(AF_INET,SOCK_STREAM,0);
        address.sin_family = AF_INET;
        address.sin_port = htons(port);
        server = gethostbyname(host.c_str());

        memcpy((char *)&address.sin_addr.s_addr,(char*)server->h_addr, server->h_length);

        if(-1 == connect(sockfd,(struct sockaddr *)&address,sizeof(address)))
        {
            cout <<"connection error!"<<std::endl;
            return;
        }

        cout << request << std::endl;

        write(sockfd,request.c_str(),request.size());
        char buf[1024*1024] = {0};


        int offset = 0;
        int rc;

        while(rc = read(sockfd, buf+offset, 1024))
        {
            offset += rc;
        }

        close(sockfd);
        buf[offset] = 0;
        cout << buf << std::endl;

    }

    void postImage(std::string host, int port, std::string path, unsigned char* pbyImage, int w, int h)
    {
        int size = w*h;
        char content_[size*2];
        for(int i = 0; i < size; i++)
        {
            sprintf(content_ + i * 2, "%02x", pbyImage[i]);
        }

        Base64Encrypt base64encoder;

        base64encoder.Update(content_, size*2);

        std::string w_ = std::to_string(w) + 'w';
        std::string h_ = std::to_string(h) + 'h';

        cout << w_;

        std::string content = base64encoder.GetString();
        std::stringstream stream;
        stream << "POST " << path;
        stream << " HTTP/1.0\r\n";
        stream << "Host: "<< host << "\r\n";
        stream << "User-Agent: Mozilla/5.0 (Windows; U; Windows NT 5.1; zh-CN; rv:1.9.2.3) Gecko/20100401 Firefox/3.6.3\r\n";
        stream << "Content-Type:application/x-www-form-urlencoded\r\n";
        stream << "Content-Length:" << content.length() + w_.length() + h_.length() <<"\r\n";
        stream << "Connection:close\r\n\r\n";
        stream << w_ << h_ << content.c_str();

        socketHttp(host, port, stream.str());
    }
    void postData(std::string host, int port, std::string path, std::string post_content)
    {
        //POST请求方式
        std::stringstream stream;
        stream << "POST " << path;
        stream << " HTTP/1.0\r\n";
        stream << "Host: "<< host << "\r\n";
        stream << "User-Agent: Mozilla/5.0 (Windows; U; Windows NT 5.1; zh-CN; rv:1.9.2.3) Gecko/20100401 Firefox/3.6.3\r\n";
        stream << "Content-Type:application/x-www-form-urlencoded\r\n";
        stream << "Content-Length:" << post_content.length()<<"\r\n";
        stream << "Connection:close\r\n\r\n";
        stream << post_content.c_str();

        socketHttp(host, port, stream.str());
    }

    void getData(std::string host, int port, std::string path, std::string get_content)
    {
        //GET请求方式
        std::stringstream stream;
        stream << "GET " << path << "?" << get_content;
        stream << " HTTP/1.0\r\n";
        stream << "Host: " << host << "\r\n";
        stream <<"User-Agent: Mozilla/5.0 (Windows; U; Windows NT 5.1; zh-CN; rv:1.9.2.3) Gecko/20100401 Firefox/3.6.3\r\n";
        stream <<"Connection:close\r\n\r\n";

        socketHttp(host, port, stream.str());
    }
};

#endif // _COMMON_HTTP_CONNECT_H_