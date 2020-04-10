#pragma once
#include <fstream>
#include <google/protobuf/message.h>
#include "software/multithreading/subject.h"

template <typename ProtobufMessage>
class ProtoReplayer : public Subject<ProtobufMessage>
{
    static_assert(std::is_base_of_v<google::protobuf::Message, ProtobufMessage>,
                  "Template parameter must be a derived class of google::protobuf::Message!");
public:
    ProtoReplayer() = delete;
    explicit ProtoReplayer(const std::string& filename);
    void start() {is_running = true;}
    void stop() {is_running = false;}

private:
    std::ifstream file_stream;
    bool is_running;
};

template<typename ProtobufMessage>
ProtoReplayer<ProtobufMessage>::ProtoReplayer(const std::string &filename) {

}


