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
#ifndef FUSE_CORE_THROTTLED_CALLBACK_H
#define FUSE_CORE_THROTTLED_CALLBACK_H

#include <ros/subscriber.h>

#include <functional>
#include <utility>


namespace fuse_core
{

/**
 * @brief A throttled callback that encapsulates the logic to throttle a callback so it is only called after a given
 * period in seconds (or more). The dropped calls can optionally be received in a dropped callback, that could be used
 * to count the number of calls dropped.
 *
 * @tparam Callback The std::function callback
 */
template <class Callback>
class ThrottledCallback
{
public:
  /**
   * @brief Constructor
   *
   * @param[in] keep_callback   The callback to call when kept, i.e. not dropped. Defaults to nullptr
   * @param[in] drop_callback   The callback to call when dropped because of the throttling. Defaults to nullptr
   * @param[in] throttle_period The throttling period duration in seconds. Defaults to 0.0, i.e. no throttling
   * @param[in] use_wall_time   Whether to use ros::WallTime or not. Defaults to false
   */
  ThrottledCallback(Callback&& keep_callback = nullptr,  // NOLINT(whitespace/operators)
                    Callback&& drop_callback = nullptr,  // NOLINT(whitespace/operators)
                    const ros::Duration& throttle_period = ros::Duration(0.0), const bool use_wall_time = false)
    : keep_callback_(keep_callback)
    , drop_callback_(drop_callback)
    , throttle_period_(throttle_period)
    , use_wall_time_(use_wall_time)
  {
  }

  /**
   * @brief Throttle period getter
   *
   * @return The current throttle period duration in seconds being used
   */
  const ros::Duration& getThrottlePeriod() const
  {
    return throttle_period_;
  }

  /**
   * @brief Use wall time flag getter
   *
   * @return True if using ros::WallTime, false otherwise
   */
  bool getUseWallTime() const
  {
    return use_wall_time_;
  }

  /**
   * @brief Throttle period setter
   *
   * @param[in] throttle_period The new throttle period duration in seconds to use
   */
  void setThrottlePeriod(const ros::Duration& throttle_period)
  {
    throttle_period_ = throttle_period;
  }

  /**
   * @brief Use wall time flag setter
   *
   * @param[in] use_wall_time Whether to use ros::WallTime or not
   */
  void setUseWallTime(const bool use_wall_time)
  {
    use_wall_time_ = use_wall_time;
  }

  /**
   * @brief Keep callback setter
   *
   * @param[in] keep_callback The new keep callback to use
   */
  void setKeepCallback(const Callback& keep_callback)
  {
    keep_callback_ = keep_callback;
  }

  /**
   * @brief Drop callback setter
   *
   * @param[in] drop_callback The new drop callback to use
   */
  void setDropCallback(const Callback& drop_callback)
  {
    drop_callback_ = drop_callback;
  }

  /**
   * @brief Last called time
   *
   * @return The last time the keep callback was called
   */
  const ros::Time& getLastCalledTime() const
  {
    return last_called_time_;
  }

  /**
   * @brief Callback that throttles the calls to the keep callback provided. When dropped because
   * of throttling, the drop callback is called instead.
   * @details The decision to throttle is made using ros::Time::now when use_wall_time_ is false and ros::WallTime::now
   * when it is true. The first message after the most recent whole-number multiple of the throttle_period_ is published
   * and all others are dropped.
   *
   * @param[in] args The input arguments
   */
  template <class... Args>
  void callback(Args&&... args)
  {
    const ros::Time now = use_wall_time_ ? ros::Time(ros::WallTime::now().toSec()) : ros::Time::now();
    callbackImpl(now, std::forward<Args>(args)...);
  }

  /**
   * @brief Operator() that simply calls the callback() method forwarding the input arguments
   *
   * @param[in] args The input arguments
   */
  template <class... Args>
  void operator()(Args&&... args)
  {
    callback(std::forward<Args>(args)...);
  }

protected:
  /**
   * @brief The implementation of the throttled callback
   *
   * @param[in] time The time used for throttling
   * @param[in] args The input arguments
   */
  template <class... Args>
  void callbackImpl(const ros::Time& time, Args&&... args)
  {
    // Keep the callback if:
    //
    // (a) The throttle period is zero, so we should always keep the callbacks
    // (b) The time is past the next whole number multiple of the throttle period
    //  If this is the first time this has been called, just store what the most recent phase-aligned publish time
    //  would have been so we know the next publish time.
    bool keep = false;
    if (throttle_period_.isZero())
    {
      keep = true;
    }
    else if (!last_called_time_set_ || time > last_called_time_ + throttle_period_)
    {
      if (last_called_time_set_)
      {
        keep = true;
      }

      // Use the nearest phase-aligned multiple of the throttle period as our reference time to improve determinism
      const double time_s = time.toSec();
      const double last_called_time_s =  time_s - std::fmod(time_s, throttle_period_.toSec());
      last_called_time_.fromSec(last_called_time_s);
      last_called_time_set_ = true;
    }

    if (keep)
    {
      if (keep_callback_)
      {
        keep_callback_(std::forward<Args>(args)...);
      }
    }
    else if (drop_callback_)
    {
      drop_callback_(std::forward<Args>(args)...);
    }
  }

  Callback keep_callback_;            //!< The callback to call when kept, i.e. not dropped
  Callback drop_callback_;            //!< The callback to call when dropped because of throttling
  ros::Duration throttle_period_;     //!< The throttling period duration in seconds
  bool use_wall_time_;                //<! The flag to indicate whether to use ros::WallTime or not
  bool last_called_time_set_{false};  //<! The flag to indicate whether the last called time has been set
  ros::Time last_called_time_;        //!< The last time the keep callback was called
};

/**
 * @brief Throttled callback for ROS messages, option to throttle using header timestamps with `message_stamp_callback`
 *
 * @tparam M The ROS message type, which should have the M::ConstPtr nested type
 */
template <class M>
class ThrottledMessageCallback : public ThrottledCallback<std::function<void(const typename M::ConstPtr&)>>
{
public:
  using Callback = std::function<void(const typename M::ConstPtr&)>;

  ThrottledMessageCallback(Callback&& keep_callback = nullptr,  // NOLINT(whitespace/operators)
                           Callback&& drop_callback = nullptr,  // NOLINT(whitespace/operators)
                           const ros::Duration& throttle_period = ros::Duration(0.0),
                           const bool use_wall_time = false,
                           const bool use_header_timestamps = false)
    : ThrottledCallback<Callback>(std::move(keep_callback), std::move(drop_callback), throttle_period,
                                  use_wall_time), use_header_timestamps_(use_header_timestamps)
    {
      if (use_wall_time && use_header_timestamps)
      {
        throw std::runtime_error("Cannot specify both wall time and header timestamps for callback throttling."
                                 " Use only one.");
      }
    }

 /**
 * @brief Callback that selects the correct timestamp to throttle the calls to the keep callback provided based
 * the configuration arguments.
 *
 * @param[in] args The callback args, will be forwarded to the callback
 */
  template <class... Args>
  void callback(Args&&... args)
  {
    if (use_header_timestamps_)
    {
      messageStampCallback(std::forward<Args>(args)...);
    }
    else
    {
      // Use base-class version
      ThrottledCallback<Callback>::callback(std::forward<Args>(args)...);
    }
  }

  /**
   * @brief Operator() that simply calls the callback() method forwarding the input arguments
   * @note Needs to be duplicated because virtual templates are not allowed
   *
   * @param[in] args The input arguments
   */
  template <class... Args>
  void operator()(Args&&... args)
  {
    callback(std::forward<Args>(args)...);
  }


  /**
   * @brief Callback that uses message header timestamps to throttle the calls to the keep callback provided.
   * When dropped because of throttling, the drop callback is called instead.
   * @details The first message after the most recent whole-number multiple of the throttle_period_ is published
   * and all others are dropped.
   *
   * @param[in] msg The message to be throttled
   */
  template <typename = typename std::enable_if_t<ros::message_traits::HasHeader<M>::value>>
  void messageStampCallback(const typename M::ConstPtr& msg)
  {
    const ros::Time msg_stamp_time(msg->header.stamp.sec, msg->header.stamp.nsec);
    this->callbackImpl(msg_stamp_time, msg);
  }

  /**
   * @brief Use header timestamps flag setter
   *
   * @param[in] use_header_timestamps Whether to use msg header timestamps or not
   */
  void setUseHeaderTimestamps(const bool use_header_timestamps)
  {
    if (use_header_timestamps && this->use_wall_time_)
    {
      throw std::runtime_error("Cannot specify both wall time and header timestamps for callback throttling."
                               " Use only one.");
    }
    use_header_timestamps_ = use_header_timestamps;
  }

protected:
  bool use_header_timestamps_{ false };  //<! Whether to use message header timestamps or current time
};



}  // namespace fuse_core

#endif  // FUSE_CORE_THROTTLED_CALLBACK_H
