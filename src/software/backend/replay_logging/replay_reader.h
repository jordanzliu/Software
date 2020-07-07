#pragma once
#include <experimental/filesystem>

#include "software/proto/replay_msg.pb.h"
#include "software/proto/sensor_msg.pb.h"

class ReplayReader
{
   public:
    explicit ReplayReader(const std::string& _replay_dir);
    std::optional<SensorMsg> getNextMsg();

   private:
    bool hasNextMsg() const;
    void nextChunk();
    size_t max_chunk_idx;
    size_t cur_chunk_idx;
    int cur_msg_idx;
    ReplayMsg cur_chunk;
    std::experimental::filesystem::path replay_dir;
};