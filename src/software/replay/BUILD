package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_logger",
    hdrs = ["proto_logger.h"],
    deps = [
        "//software/logger",
        "//software/multithreading:threaded_observer",
        "@com_google_protobuf//:protobuf_headers",
    ],
)

cc_library(
    name = "proto_replayer",
    hdrs = ["proto_replayer.h"],
    deps = [
        "//software/logger",
        "//software/multithreading:subject",
        "@com_google_protobuf//:protobuf_headers",
    ],
)
