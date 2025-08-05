package(default_visibility = ["//visibility:public"])

load("@simulated_tests_deps//:requirements.bzl", "requirement")

py_test(
    name = "utils_test",
    srcs = ["utils_test.py"],
    deps = [
        "//software/ml:utils",
        requirement("numpy"),
        requirement("numpy-quaternion"),
    ],
)
