#pragma once
#include <fstream>
#include <google/protobuf/message.h>
#include "software/logger/init.h"
#include "software/multithreading/threaded_observer.h"

template <class ProtobufMessage>
class ProtoLogger : public ThreadedObserver<ProtobufMessage> {
    static_assert(std::is_base_of_v<google::protobuf::Message, ProtobufMessage>,
            "ProtoLogger only works for protobuf messages!");
public:
    ProtoLogger() = delete;

    explicit ProtoLogger(const std::string &filename): output_file_stream(filename) {
        LOG(INFO) << "Logging " << typeid(ProtobufMessage).name() << " to " << filename;
    }

private:
    void onValueReceived(ProtobufMessage val) override;

    std::ofstream output_file_stream;
};

template<class ProtobufMessage>
void ProtoLogger<ProtobufMessage>::onValueReceived(ProtobufMessage val) {
    if (!val.SerializeToOstream(&output_file_stream))
    {
        LOG(INFO) << "FUCK";
        throw std::string("Failed to log protobuf to file");
    } else {
        LOG(INFO) << "Logged a protobuf message";
    }
}
