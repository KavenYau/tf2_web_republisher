/*********************************************************************
 *
 *  Copyright (c) 2014, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.

 *  Author: Julius Kammerl (jkammerl@willowgarage.com)
 *
 */

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <tf2_web_republisher/action/tf_subscription.hpp>
#include <tf2_web_republisher/msg/tf_array.hpp>
#include <tf2_web_republisher/srv/republish_t_fs.hpp>

#include "tf_pair.h"

class TFRepublisher : public rclcpp::Node {
 protected:
  typedef tf2_web_republisher::action::TFSubscription TFTransformServer;
  typedef rclcpp_action::ServerGoalHandle<TFTransformServer> GoalHandle;
  typedef tf2_web_republisher::srv::RepublishTFs RepublishTFService;
  typedef rclcpp::Publisher<tf2_web_republisher::msg::TFArray> TFArrayPublisher;

  rclcpp::Node::SharedPtr client_node_;

  rclcpp_action::Server<TFTransformServer>::SharedPtr action_server_;
  rclcpp::Service<RepublishTFService>::SharedPtr tf_republish_service_;

  // base struct that holds information about the TFs
  // a client (either Service or Action) has subscribed to
  struct ClientInfo {
    std::vector<TFPair> tf_subscriptions_;
    unsigned int client_ID_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

  // struct for Action client info
  struct ClientGoalInfo : ClientInfo {
    std::shared_ptr<GoalHandle> handle;
  };

  // struct for Service client info
  struct ClientRequestInfo : ClientInfo {
    TFArrayPublisher::SharedPtr pub_;
    std::shared_ptr<rclcpp::Duration> unsub_timeout_;
    rclcpp::TimerBase::SharedPtr unsub_timer_;
  };

  std::list<std::shared_ptr<ClientGoalInfo> > active_goals_;
  std::mutex goals_mutex_;

  std::list<std::shared_ptr<ClientRequestInfo> > active_requests_;
  std::mutex requests_mutex_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex tf_buffer_mutex_;

  unsigned int client_ID_count_;

 public:
  TFRepublisher(const std::string& name) : rclcpp::Node(name), client_ID_count_(0) {
    auto options = rclcpp::NodeOptions().arguments(
        {"--ros-args", "-r", std::string("__node:=") + std::string(this->get_name()) + "_rclcpp_node", "--"});
    client_node_ = std::make_shared<rclcpp::Node>("_", options);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface =
        std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    action_server_ = rclcpp_action::create_server<TFTransformServer>(
        this, "tf2_web_republisher",
        std::bind(&TFRepublisher::goalCB, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TFRepublisher::cancelCB, this, std::placeholders::_1),
        std::bind(&TFRepublisher::acceptedCB, this, std::placeholders::_1));

    tf_republish_service_ = this->create_service<RepublishTFService>(
        "republish_tfs", std::bind(&TFRepublisher::requestCB, this, std::placeholders::_1, std::placeholders::_2));
  }

  ~TFRepublisher() {}

  void acceptedCB(const std::shared_ptr<GoalHandle> goal_handle) {
    const auto& goal = goal_handle->get_goal();

    std::shared_ptr<ClientGoalInfo> goal_info = std::make_shared<ClientGoalInfo>();
    goal_info->handle = goal_handle;
    goal_info->client_ID_ = client_ID_count_++;

    setSubscriptions(goal_info, goal->source_frames, goal->target_frame, goal->angular_thres, goal->trans_thres);

    {
      double rate = goal->rate;
      if (rate == 0) {
        rate = 10.0;
      }

      std::function<void()> callback = std::bind(&TFRepublisher::processGoal, this, goal_info);
      goal_info->timer_ = this->create_wall_timer(
          rclcpp::Duration::from_seconds(1.0 / rate).to_chrono<std::chrono::milliseconds>(), callback);
    }

    {
      std::unique_lock<std::mutex> l(goals_mutex_);
      // add new goal to list of active goals/clients
      active_goals_.push_back(goal_info);
    }
  }

  rclcpp_action::CancelResponse cancelCB(const std::shared_ptr<GoalHandle> goal_handle) {
    std::unique_lock<std::mutex> l(goals_mutex_);

    RCLCPP_DEBUG(get_logger(), "GoalHandle canceled");

    // search for goal handle and remove it from active_goals_ list
    for (std::list<std::shared_ptr<ClientGoalInfo> >::iterator it = active_goals_.begin(); it != active_goals_.end();) {
      ClientGoalInfo& info = **it;
      if (info.handle->get_goal_id() == goal_handle->get_goal_id()) {
        it = active_goals_.erase(it);
        info.timer_->cancel();

        if (info.handle != nullptr && info.handle->is_active()) {
          auto result = std::make_shared<TFTransformServer::Result>();

          if (info.handle->is_canceling()) {
            info.handle->canceled(result);
          } else {
            info.handle->abort(result);
          }
        }

        return rclcpp_action::CancelResponse::ACCEPT;
      } else {
        ++it;
      }
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  const std::string cleanTfFrame(const std::string frame_id) const {
    if (frame_id[0] == '/') {
      return frame_id.substr(1);
    }
    return frame_id;
  }

  /**
   * Set up the contents of \p tf_subscriptions_ in
   * a ClientInfo struct
   */
  void setSubscriptions(std::shared_ptr<ClientInfo> info, const std::vector<std::string>& source_frames,
                        const std::string& target_frame_, float angular_thres, float trans_thres) const {
    std::size_t request_size_ = source_frames.size();
    info->tf_subscriptions_.resize(request_size_);

    for (std::size_t i = 0; i < request_size_; ++i) {
      TFPair& tf_pair = info->tf_subscriptions_[i];

      std::string source_frame = cleanTfFrame(source_frames[i]);
      std::string target_frame = cleanTfFrame(target_frame_);

      tf_pair.setSourceFrame(source_frame);
      tf_pair.setTargetFrame(target_frame);
      tf_pair.setAngularThres(angular_thres);
      tf_pair.setTransThres(trans_thres);
    }
  }

  rclcpp_action::GoalResponse goalCB(const rclcpp_action::GoalUUID& uuid,
                                     std::shared_ptr<const TFTransformServer::Goal> goal) {
    RCLCPP_DEBUG(get_logger(), "GoalHandle request received");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  bool requestCB(const std::shared_ptr<RepublishTFService::Request> req,
                 std::shared_ptr<RepublishTFService::Response> res) {
    RCLCPP_DEBUG(get_logger(), "RepublishTF service request received");
    // generate request_info struct
    std::shared_ptr<ClientRequestInfo> request_info = std::make_shared<ClientRequestInfo>();

    request_info->client_ID_ = client_ID_count_;
    std::stringstream topicname;
    topicname << "tf_repub_" << client_ID_count_++;

    request_info->pub_ =
        client_node_->create_publisher<tf2_web_republisher::msg::TFArray>(topicname.str(), rclcpp::SystemDefaultsQoS());

    // add the tf_subscriptions to the ClientGoalInfo object
    setSubscriptions(request_info, req->source_frames, req->target_frame, req->angular_thres, req->trans_thres);

    if (req->timeout.sec != 0 || req->timeout.nanosec != 0) {
      request_info->unsub_timeout_ = std::make_shared<rclcpp::Duration>(req->timeout);
    } else {
      request_info->unsub_timeout_ = std::make_shared<rclcpp::Duration>(rclcpp::Duration::from_seconds(10.0));
    }

    {
      std::function<void()> callback = std::bind(&TFRepublisher::unadvertiseCB, this, request_info);
      request_info->unsub_timer_ = this->create_wall_timer(
          request_info->unsub_timeout_->to_chrono<std::chrono::milliseconds>(), callback);  // only fire once
    }

    {
      double rate = req->rate;
      if (rate == 0) {
        rate = 10.0;
      }

      std::function<void()> callback = std::bind(&TFRepublisher::processRequest, this, request_info);
      request_info->timer_ = this->create_wall_timer(
          rclcpp::Duration::from_seconds(1.0 / rate).to_chrono<std::chrono::milliseconds>(), callback);
    }

    {
      std::unique_lock<std::mutex> l(requests_mutex_);
      // add new request to list of active requests
      active_requests_.push_back(request_info);
    }
    res->topic_name = request_info->pub_->get_topic_name();
    RCLCPP_INFO_STREAM(get_logger(), "Publishing requested TFs on topic " << res->topic_name);

    return true;
  }

  void unadvertiseCB(std::shared_ptr<ClientRequestInfo> request_info) {
    RCLCPP_INFO_STREAM(get_logger(), "No subscribers on tf topic for request "
                                         << request_info->client_ID_ << " for "
                                         << request_info->unsub_timeout_->seconds()
                                         << " seconds. Unadvertising topic:" << request_info->pub_->get_topic_name());
    request_info->pub_.reset();
    request_info->unsub_timer_->cancel();
    request_info->timer_->cancel();

    // search for ClientRequestInfo struct and remove it from active_requests_ list
    for (std::list<std::shared_ptr<ClientRequestInfo> >::iterator it = active_requests_.begin();
         it != active_requests_.end(); ++it) {
      ClientRequestInfo& info = **it;
      if (info.client_ID_ == request_info->client_ID_) {
        active_requests_.erase(it);
        return;
      }
    }
  }

  void updateSubscriptions(std::vector<TFPair>& tf_subscriptions,
                           std::vector<geometry_msgs::msg::TransformStamped>& transforms) {
    // iterate over tf_subscription vector
    std::vector<TFPair>::iterator it;
    std::vector<TFPair>::const_iterator end = tf_subscriptions.end();

    for (it = tf_subscriptions.begin(); it != end; ++it) {
      geometry_msgs::msg::TransformStamped transform;

      try {
        // protecting tf_buffer
        std::unique_lock<std::mutex> lock(tf_buffer_mutex_);

        // lookup transformation for tf_pair
        transform = tf_buffer_->lookupTransform(it->getTargetFrame(), it->getSourceFrame(), rclcpp::Time(0));

        // If the transform broke earlier, but worked now (we didn't get
        // booted into the catch block), tell the user all is well again
        if (!it->is_okay) {
          it->is_okay = true;
          RCLCPP_INFO_STREAM(get_logger(), "Transform from " << it->getSourceFrame() << " to " << it->getTargetFrame()
                                                             << " is working again at time "
                                                             << rclcpp::Time(transform.header.stamp).seconds());
        }
        // update tf_pair with transformtion
        it->updateTransform(transform);
      } catch (tf2::TransformException ex) {
        // Only log an error if the transform was okay before
        if (it->is_okay) {
          it->is_okay = false;
          RCLCPP_ERROR(get_logger(), "%s", ex.what());
        }
      }

      // check angular and translational thresholds
      if (it->updateNeeded()) {
        transform.header.stamp = this->now();
        transform.header.frame_id = it->getTargetFrame();
        transform.child_frame_id = it->getSourceFrame();

        // notify tf_subscription that a network transmission has been triggered
        it->transmissionTriggered();

        // add transform to the array
        transforms.push_back(transform);
      }
    }
  }

  void processGoal(std::shared_ptr<ClientGoalInfo> goal_info) {
    auto feedback = std::make_shared<TFTransformServer::TFSubscription::Feedback>();

    updateSubscriptions(goal_info->tf_subscriptions_, feedback->transforms);

    if (feedback->transforms.size() > 0) {
      // publish feedback
      goal_info->handle->publish_feedback(feedback);
      RCLCPP_DEBUG(get_logger(), "Client %d: TF feedback published:", goal_info->client_ID_);
    } else {
      RCLCPP_DEBUG(get_logger(), "Client %d: No TF frame update needed:", goal_info->client_ID_);
    }
  }

  void processRequest(std::shared_ptr<ClientRequestInfo> request_info) {
    if (request_info->pub_->get_subscription_count() == 0) {
      if (request_info->unsub_timer_->is_canceled()) {
        request_info->unsub_timer_->reset();
      }
    } else {
      request_info->unsub_timer_->cancel();
    }

    auto array_msg = std::make_unique<tf2_web_republisher::msg::TFArray>();
    updateSubscriptions(request_info->tf_subscriptions_, array_msg->transforms);

    if (array_msg->transforms.size() > 0) {
      // publish TFs
      request_info->pub_->publish(std::move(array_msg));
      RCLCPP_DEBUG(get_logger(), "Request %d: TFs published:", request_info->client_ID_);
    } else {
      RCLCPP_DEBUG(get_logger(), "Request %d: No TF frame update needed:", request_info->client_ID_);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFRepublisher>("tf2_web_republisher");

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
