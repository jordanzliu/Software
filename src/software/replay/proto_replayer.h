#pragma oncel
#include <fstream>
#include <google/protobuf/message.h>
#include "software/multithreading/subject.h"
#include "software/logger/init.h"

template <typename ProtobufMessage>
class ProtoReplayer : public Subject<ProtobufMessage>
{
    static_assert(std::is_base_of_v<google::protobuf::Message, ProtobufMessage>,
                  "Template parameter must be a derived class of google::protobuf::Message!");
public:
    ProtoReplayer() = delete;
    explicit ProtoReplayer(const std::string& filename);
    void runOnce();

private:
    std::ifstream file_ifstream;
};

template<typename ProtobufMessage>
ProtoReplayer<ProtobufMessage>::ProtoReplayer(const std::string &filename)
: file_ifstream(filename) {}


template <typename ProtobufMessage>
void ProtoReplayer<ProtobufMessage>::runOnce()
{
    ProtobufMessage message;
    if (!message.ParseFromIstream(&file_ifstream))
    {
        LOG(INFO) << "FUCK";
        throw std::string("Failed to parse message from istream!");
    }
    else
    {
        this->sendValueToObservers(message);
    }
}
