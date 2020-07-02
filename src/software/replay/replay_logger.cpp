//
// Created by jordan on 5/2/20.
//

#include "software/replay/replay_logger.h"

#include <fstream>
#include <google/protobuf/util/delimited_message_util.h>

#include "software/logger/logger.h"

namespace fs = std::experimental::filesystem;

ReplayLogger::ReplayLogger(const std::string& out_dir_path, int _frames_per_chunk) :
OrderedThreadedObserver<SensorMsg>(2000), output_dir_path(out_dir_path), current_chunk(),
current_chunk_idx(0), frames_per_chunk(_frames_per_chunk)
{
    // check if directory exists, if not make a directory
    if (fs::exists(output_dir_path))
    {
        if (!fs::is_directory(output_dir_path))
        {
            throw std::invalid_argument(out_dir_path + " exists and is not a directory!");
        }
        if (!fs::is_empty(output_dir_path))
        {
            // this is better behavior than either adding more chunks to the same directory
            // (and having one directory end up with multiple replays)
            // or silently overwriting and destroying a previous replay
            throw std::invalid_argument(out_dir_path + " is not empty! Find another directory!");
        }
    }


    if (!fs::exists(output_dir_path))
    {
        fs::create_directory(output_dir_path);
        LOG(INFO) << "Created directory " << output_dir_path;
    }

    LOG(INFO) << "Logging to " << output_dir_path.string();
}


ReplayLogger::~ReplayLogger() {
    saveCurrentChunk();
}


void ReplayLogger::onValueReceived(SensorMsg frame)
{
    if (frame.has_ssl_vision_msg()) {
        LOG(INFO) << "Logging vision frame with t_sent=" << frame.ssl_vision_msg().detection().t_sent();
    }
    current_chunk.mutable_replay_frames()->Add();
    (current_chunk.mutable_replay_frames()->end() - 1)->CopyFrom(frame);
    if (current_chunk.replay_frames_size() >= frames_per_chunk) {
        saveCurrentChunk();
        nextChunk();
    }
}


void ReplayLogger::nextChunk() {
    current_chunk_idx++;
    current_chunk.Clear();
}
void ReplayLogger::saveCurrentChunk() {
    fs::path chunk_path = output_dir_path / std::to_string(current_chunk_idx);
    std::ofstream chunk_ofstream(chunk_path);
    auto result = google::protobuf::util::SerializeDelimitedToOstream(current_chunk, &chunk_ofstream);
    if (!result) {
        LOG(WARNING) << "Failed to log chunk!";
    }
}
