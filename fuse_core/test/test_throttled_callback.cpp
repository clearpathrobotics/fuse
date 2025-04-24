/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Clearpath Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <fuse_core/throttled_callback.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>

#include <gtest/gtest.h>


/**
 * @brief A helper class to publish a given number geometry_msgs::PointStamped messages at a given frequency.
 *
 * The messages published are geometry_msgs::PointStamped because it is simple. The 'x' field is set to the number of messages
 * published so far, starting at 0.
 */
class PointPublisher
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] frequency The publishing frequency in Hz
   */
  explicit PointPublisher(const double frequency)
    : frequency_(frequency)
  {
    publisher_ = node_handle_.advertise<geometry_msgs::PointStamped>("point", 1);
  }

  /**
   * @brief Publish the given number of messages
   *
   * @param[in] num_messages The number of messages to publish
   */
  void publish(const size_t num_messages)
  {
    // Wait for the subscribers to be ready before sending them data:
    const ros::WallTime subscriber_timeout = ros::WallTime::now() + ros::WallDuration(1.0);
    while (publisher_.getNumSubscribers() < 1u && ros::WallTime::now() < subscriber_timeout)
    {
      ros::WallDuration(0.01).sleep();
    }

    ASSERT_GE(publisher_.getNumSubscribers(), 1u);

    // Send data:
    ros::Rate rate(frequency_);
    for (size_t i = 0; i < num_messages; ++i)
    {
      geometry_msgs::PointStamped point_message;
      point_message.point.x = i;

      publisher_.publish(point_message);

      rate.sleep();
    }
  }

  void publishWithHeaderTimestamp(const ros::Time& timestamp)
  {
    const ros::WallTime subscriber_timeout = ros::WallTime::now() + ros::WallDuration(1.0);
    while (publisher_.getNumSubscribers() < 1u && ros::WallTime::now() < subscriber_timeout)
    {
      ros::WallDuration(0.01).sleep();
    }

    ASSERT_GE(publisher_.getNumSubscribers(), 1u);

    geometry_msgs::PointStamped point_message;
    point_message.header.stamp = timestamp;

    publisher_.publish(point_message);
  }

private:
  ros::NodeHandle node_handle_;  //!< The node handle
  ros::Publisher publisher_;     //!< The publisher
  double frequency_{ 10.0 };     //!< The publish rate frequency
};

/**
 * @brief A dummy point sensor model that uses a fuse_core::ThrottledMessageCallback<geometry_msgs::PointStamped> with a keep
 * and drop callback.
 *
 * The callbacks simply count the number of times they are called, for testing purposes. The keep callback also caches
 * the last message received, also for testing purposes.
 */
class PointSensorModel
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] throttle_period The throttle period duration in seconds
   */
  explicit PointSensorModel(const ros::Duration& throttle_period, bool use_header_timestamp = false)
    : throttled_callback_(std::bind(&PointSensorModel::keepCallback, this, std::placeholders::_1),
                          std::bind(&PointSensorModel::dropCallback, this, std::placeholders::_1),
                          throttle_period, false, use_header_timestamp)
  {
    subscriber_ = node_handle_.subscribe<geometry_msgs::PointStamped>(
        "point", 10, &PointThrottledCallback::callback, &throttled_callback_);
  }

  /**
   * @brief The kept messages counter getter
   *
   * @return The number of messages kept
   */
  size_t getKeptMessages() const
  {
    return kept_messages_;
  }

  /**
   * @brief The dropped messages counter getter
   *
   * @return The number of messages dropped
   */
  size_t getDroppedMessages() const
  {
    return dropped_messages_;
  }

  /**
   * @brief The last message kept getter
   *
   * @return The last message kept. It would be nullptr if no message has been kept so far
   */
  const geometry_msgs::PointStamped::ConstPtr getLastKeptMessage() const
  {
    return last_kept_message_;
  }

  /**
   * @brief Reset the number of kept and dropped messages to zero
   */
  void reset()
  {
    kept_messages_ = 0;
    dropped_messages_ = 0;
  }

private:
  /**
   * @brief Keep callback, that counts the number of times it has been called and caches the last message received
   *
   * @param[in] msg A geometry_msgs::PointStamped message
   */
  void keepCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    ++kept_messages_;
    last_kept_message_ = msg;
  }

  /**
   * @brief Drop callback, that counts the number of times it has been called
   *
   * @param[in] msg A geometry_msgs::PointStamped message (not used)
   */
  void dropCallback(const geometry_msgs::PointStamped::ConstPtr& /*msg*/)
  {
    ++dropped_messages_;
  }

  ros::NodeHandle node_handle_;  //!< The node handle
  ros::Subscriber subscriber_;   //!< The subscriber

  using PointThrottledCallback = fuse_core::ThrottledMessageCallback<geometry_msgs::PointStamped>;
  PointThrottledCallback throttled_callback_;  //!< The throttled callback

  size_t kept_messages_{ 0 };                         //!< Messages kept
  size_t dropped_messages_{ 0 };                      //!< Messages dropped
  geometry_msgs::PointStamped::ConstPtr last_kept_message_;  //!< The last message kept
};


TEST(ThrottledCallback, NoDroppedMessagesIfThrottlePeriodIsZero)
{
  // Time should be valid after ros::init() returns in main(). But it doesn't hurt to verify.
  ASSERT_TRUE(ros::Time::waitForValid(ros::WallDuration(1.0)));

  // Start sensor model to listen to messages:
  const ros::Duration throttled_period(0.0);
  PointSensorModel sensor_model(throttled_period);

  // Publish some messages:
  const size_t num_messages = 10;
  const double frequency = 10.0;

  PointPublisher publisher(frequency);
  publisher.publish(num_messages);

  // Check all messages are kept and none are dropped, because when the throttle period is zero, throttling is disabled:
  EXPECT_EQ(num_messages, sensor_model.getKeptMessages());
  EXPECT_EQ(0u, sensor_model.getDroppedMessages());
}

TEST(ThrottledCallback, DropMessagesIfThrottlePeriodIsGreaterThanPublishPeriod)
{
  // Time should be valid after ros::init() returns in main(). But it doesn't hurt to verify.
  ASSERT_TRUE(ros::Time::waitForValid(ros::WallDuration(1.0)));

  // Start sensor model to listen to messages:
  const ros::Duration throttled_period(0.2);
  PointSensorModel sensor_model(throttled_period);

  // Publish some messages at half the throttled period:
  const size_t num_messages = 10;
  const double period_factor = 0.25;
  const double period = period_factor * throttled_period.toSec();
  const double frequency = 1.0 / period;

  PointPublisher publisher(frequency);
  publisher.publish(num_messages);

  // Check the number of kept and dropped callbacks:
  const auto expected_kept_messages = period_factor * num_messages;
  const auto expected_dropped_messages = num_messages - expected_kept_messages;

  EXPECT_NEAR(expected_kept_messages, sensor_model.getKeptMessages(), 1.0);
  EXPECT_NEAR(expected_dropped_messages, sensor_model.getDroppedMessages(), 1.0);
}

TEST(ThrottledCallback, TestMessageHeaderThrottling)
{
  // Time should be valid after ros::init() returns in main(). But it doesn't hurt to verify.
  ASSERT_TRUE(ros::Time::waitForValid(ros::WallDuration(1.0)));

  constexpr double kThrottleFrequency = 5.0;
  constexpr double kThrottlePeriodS = 1 / kThrottleFrequency;

  const ros::Duration throttled_period(kThrottlePeriodS);
  PointSensorModel sensor_model(throttled_period, true);

  constexpr unsigned kPublishToThrottleRatio = 5.0;
  constexpr double kPublishFrequency = kThrottleFrequency * kPublishToThrottleRatio;
  constexpr double kPublishPeriodS = 1 / kPublishFrequency;

  PointPublisher publisher(kPublishFrequency);

  constexpr double kBaseTimeS = 10.0;
  constexpr double kEpsilonS = 1e-3;
  ros::Time timestamp;

  // Initialize the throttled calback with a time just after kBaseTimeS so that kBaseTimeS is the
  // start timestamp for throttling and the first accepted message will be kThrottlePeriodS after kbaseTimeS
  timestamp.fromSec(kBaseTimeS + kEpsilonS);
  publisher.publishWithHeaderTimestamp(timestamp);
  ros::Duration(0.1).sleep();

  // First message is dropped, just helps find the next expected message period
  EXPECT_EQ(sensor_model.getKeptMessages(), 0);
  EXPECT_EQ(sensor_model.getDroppedMessages(), 1);

  // Make sure that messages before the next start of period are dropped
  // Publish for a full period starting from just before the start of the previous period
  // No messages should be kept because we haven't crossed into the period after the initial message yet
  sensor_model.reset();
  timestamp.fromSec(kBaseTimeS - kEpsilonS);
  unsigned i = 0;
  for (; i < kPublishToThrottleRatio; ++i)
  {
    timestamp.fromSec(timestamp.toSec() + kPublishPeriodS);
    publisher.publishWithHeaderTimestamp(timestamp);
  }

  ros::Duration(0.1).sleep();
  EXPECT_EQ(sensor_model.getKeptMessages(), 0);
  EXPECT_EQ(sensor_model.getDroppedMessages(), kPublishToThrottleRatio);

  // Publish for another period, phase aligned to just before the period division
  // Should keep one message
  sensor_model.reset();

  for (; i < 2 * kPublishToThrottleRatio; ++i)
  {
    timestamp.fromSec(timestamp.toSec() + kPublishPeriodS);
    publisher.publishWithHeaderTimestamp(timestamp);
  }
  ros::Duration(0.1).sleep();

  EXPECT_EQ(sensor_model.getKeptMessages(), 1);
  EXPECT_EQ(sensor_model.getDroppedMessages(), kPublishToThrottleRatio - 1);

  // Publish one more message which puts us over the next period division, should be kept
  sensor_model.reset();
  timestamp.fromSec(timestamp.toSec() + kPublishPeriodS);
  publisher.publishWithHeaderTimestamp(timestamp);
  ros::Duration(0.1).sleep();

  EXPECT_EQ(sensor_model.getKeptMessages(), 1);
  EXPECT_EQ(sensor_model.getDroppedMessages(), 0);
}

// Test that the call operator is behaving correctly
TEST(ThrottledCallback, TestCallOperator)
{
  constexpr double kThrottlePeriod = 0.1;

  unsigned base_kept = 0;
  auto base_keep_callback = [&base_kept](const geometry_msgs::PointStamped::ConstPtr & msg){ ++base_kept; };

  unsigned header_timestamp_kept = 0;
  auto msg_keep_callback =
    [&header_timestamp_kept](const geometry_msgs::PointStamped::ConstPtr & msg){ ++header_timestamp_kept; };

  fuse_core::ThrottledCallback<std::function<void(const typename geometry_msgs::PointStamped::ConstPtr&)>>
    base_callback(base_keep_callback, nullptr, ros::Duration(kThrottlePeriod));

  fuse_core::ThrottledMessageCallback<geometry_msgs::PointStamped> message_header_calback(
    msg_keep_callback, nullptr, ros::Duration(kThrottlePeriod), false, true);

  geometry_msgs::PointStamped point_msg;
  point_msg.header.stamp.fromSec(0.0);

  geometry_msgs::PointStampedConstPtr msg_const = boost::make_shared<const geometry_msgs::PointStamped>(point_msg);

  // Initialize both callbacks
  base_callback(msg_const);
  message_header_calback(msg_const);

  // Neither keeps the first message
  EXPECT_EQ(base_kept, 0);
  EXPECT_EQ(header_timestamp_kept, 0);

  ros::Duration(1.0).sleep();
  // Call with the same message again
  base_callback(msg_const);
  message_header_calback(msg_const);
  // The base callback should keep this one because it's related to elapsed time
  EXPECT_EQ(base_kept, 1);
  // The header callback should not keep this one because the header timestamp is the same as the old one
  EXPECT_EQ(header_timestamp_kept, 0);

  // Advance the header timestamp and check that the header timestamp based throttle keeps it
  point_msg.header.stamp += ros::Duration(1.0);
  msg_const = boost::make_shared<const geometry_msgs::PointStamped>(point_msg);
  message_header_calback(msg_const);

  // The header callback should keep this one because the header timestamp is advanced more than the throttle period
  EXPECT_EQ(header_timestamp_kept, 1);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "throttled_callback_test");
  auto spinner = ros::AsyncSpinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
