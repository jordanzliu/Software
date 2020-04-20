#include <gtest/gtest.h>
#include "software/multithreading/threaded_observer.h"
#include "software/proto/messages_robocup_ssl_detection.pb.h"
#include "software/replay/proto_logger.h"
#include "software/replay/proto_replayer.h"

class TestObserver : public ThreadedObserver<SSL_DetectionBall>
{
   public:
    TestObserver() = default;
    SSL_DetectionBall lastValue() const;
   private:
    void onValueReceived(SSL_DetectionBall val) override;
    SSL_DetectionBall last_value;
};

SSL_DetectionBall TestObserver::lastValue() const
{
    return SSL_DetectionBall();
}

void TestObserver::onValueReceived(SSL_DetectionBall val) {
    last_value = val;
}

class TestSubject : public Subject<SSL_DetectionBall>
{
   public:
    TestSubject() = default;
    void sendValue(SSL_DetectionBall val);
};

void TestSubject::sendValue(SSL_DetectionBall val) {
    sendValueToObservers(val);
}

TEST(ProtoLoggerReplayerTest, test_log_and_replay_one_message)
{
    SSL_DetectionBall test_message;
    test_message.set_x(1.0);
    test_message.set_y(2.0);
    test_message.set_z(3.0);
    test_message.set_pixel_x(4.0);
    test_message.set_pixel_y(5.0);
    test_message.set_confidence(6.0);
    test_message.set_area(7.0);

    // create a scope here so that the logger is destroyed at the end
    // and closes the file handle
    {
        TestSubject test_subject;
        auto logger = std::make_shared<ProtoLogger<SSL_DetectionBall>>("test.protolog");
        test_subject.registerObserver(logger);
        test_subject.sendValue(test_message);
        test_subject.sendValue(test_message);
    }

    ProtoReplayer<SSL_DetectionBall> replayer("test.protolog");
    auto test_observer = std::make_shared<TestObserver>();
    replayer.registerObserver(test_observer);
    replayer.runOnce();
    SSL_DetectionBall result_message = test_observer->lastValue();
    EXPECT_EQ(test_message.x(), result_message.x());
    EXPECT_EQ(test_message.y(), result_message.y());
    EXPECT_EQ(test_message.z(), result_message.z());
    EXPECT_EQ(test_message.pixel_x(), result_message.pixel_x());
    EXPECT_EQ(test_message.pixel_y(), result_message.pixel_y());
    EXPECT_EQ(test_message.confidence(), result_message.confidence());
    EXPECT_EQ(test_message.area(), result_message.area());
}