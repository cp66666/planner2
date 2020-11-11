#include "MessageParser.h"

MessageParser::MessageParser()
{

}

MessageParser::~MessageParser()
{

}

std::string MessageParser::GetRobotcommMsgStr(std::string optype, float v, float omega, std::string status_str, std::string extra_str)
{
    
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    writer.StartObject();
    
    writer.Key("operation_type");
    writer.String(optype.c_str());
    writer.Key("v");
    writer.Double(v);
    writer.Key("w");
    writer.Double(omega);
    writer.Key("status");
    writer.String(status_str.c_str());
    writer.Key("extra");
    writer.String(extra_str.c_str());
    
    writer.EndObject();
    return buffer.GetString();

}
