package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_logger",
    hdrs = ["proto_logger.h"],
    deps = [
        "//software/logger",
        "//software/multithreading:threaded_observer",
        "@com_google_protobuf//:protobuf_headers",
        "@g3log",
    ],
)

cc_library(
    name = "proto_replayer",
    hdrs = ["proto_replayer.h"],
    deps = [
        "//software/logger",
        "//software/multithreading:subject",
        "@com_google_protobuf//:protobuf_headers",
        "@g3log",
    ],
)

cc_test(
    name = "proto_logger_replayer_test",
    srcs = ["proto_logger_replayer_test.cpp"],
    deps = [
        "proto_logger",
        "proto_replayer",
        "//software/multithreading:subject",
        "//software/multithreading:threaded_observer",
        "//software/proto:ssl_cc_proto",
        "@gtest//:gtest_main",
    ],
)
