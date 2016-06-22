/* This class wraps NITE2 skeletal tracking library
* and is intended to extend openni2_driver to
* also output people skeletal tracks along with
* rgb/depth information.
*
*   Author: Ivan Vishniakou (ivan.vishniakou@smail.inf.h-brs.de)
*/

#include "openni2_camera/nite_skeletal_tracker.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>

namespace openni2_wrapper
{

NITESkeletalTracker::NITESkeletalTracker(ros::NodeHandle& n, ros::NodeHandle& pnh) :
    nh_(n),
    pnh_(pnh)
{
  ROS_WARN("CREATIN NITE SKELETRACK");
}



void NITESkeletalTracker::initialize() {
  ROS_INFO_STREAM("Initialising NiTE...");
  nite::NiTE::initialize();
  nite_rc_ = user_tracker_.create();
  if (nite_rc_ != nite::STATUS_OK)
  {
    ROS_WARN("Couldn't create user tracker\n");
    boost::this_thread::sleep(boost::posix_time::seconds(3));
  } else {
    ROS_INFO_STREAM("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");
  }

  niteStart_ = pnh_.advertiseService("people_tracking/start", &NITESkeletalTracker::startNiteCb, this);
  niteStop_ = pnh_.advertiseService("people_tracking/stop", &NITESkeletalTracker::stopNiteCb, this);  
  ROS_WARN_ONCE("PEOPLE");
  pub_people_track_ = pnh_.advertise<openni2_camera::NitePeople>("people", 5);
  user_tracker_update_timer_ = nh_.createTimer(ros::Duration(0.1), &NITESkeletalTracker::userTrackerUpdate, this);
}


bool NITESkeletalTracker::startNiteCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    ROS_INFO("Staring people tracking.");
    user_tracker_update_timer_.start();
    return true;
}

bool NITESkeletalTracker::stopNiteCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    user_tracker_update_timer_.stop();
    ROS_INFO("Stopping people tracking.");
    return true;
}


void NITESkeletalTracker::userTrackerUpdate(const ros::TimerEvent& event)
{
  nite_rc_ = user_tracker_.readFrame(&user_tracker_frame_);
  if (nite_rc_ == nite::STATUS_OK)
  {    
    openni2_camera::NitePeople people_msg;
    const nite::Array<nite::UserData>& users = user_tracker_frame_.getUsers();
    for (int i = 0; i < users.getSize(); ++i)
    {
      const nite::UserData& user = users[i];
      updateUserState(user,user_tracker_frame_.getTimestamp());
      if (user.isNew())
      {
        user_tracker_.startSkeletonTracking(user.getId());
        ROS_INFO_STREAM("Found a new user.");
      }
      else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
      {
        openni2_camera::NiteSkeleton skeleton;
        skeleton.user_id = user.getId();
        people_msg.skeletons.push_back(skeleton);
        ROS_INFO_STREAM("Now tracking user " << user.getId());
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_HEAD)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_NECK)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_TORSO)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT)));
        skeleton.joints.push_back(SkeletonJointToNiteJointMsg(user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT)));
        skeleton.position.x = user.getCenterOfMass().x/1000;
        skeleton.position.y = user.getCenterOfMass().y/1000;
        skeleton.position.z = user.getCenterOfMass().z/1000;
        people_msg.skeletons.push_back(skeleton);
      }
    }
    pub_people_track_.publish(people_msg);
  }
  else
  {
    ROS_WARN("Get next frame failed.");
  }
}


void NITESkeletalTracker::updateUserState(const nite::UserData& user, unsigned long long ts)
{
  if (user.isNew())
    ROS_WARN("New");
  else if (user.isVisible() && !g_visible_users_[user.getId()])
    ROS_WARN("Visible");
  else if (!user.isVisible() && g_visible_users_[user.getId()])
    ROS_WARN("Out of Scene");
  else if (user.isLost())
    ROS_WARN("Lost");

  g_visible_users_[user.getId()] = user.isVisible();


  if(g_skeleton_states_[user.getId()] != user.getSkeleton().getState())
  {
    switch(g_skeleton_states_[user.getId()] = user.getSkeleton().getState())
    {
    case nite::SKELETON_NONE:
      ROS_WARN("Stopped tracking.");
      break;
    case nite::SKELETON_CALIBRATING:
      ROS_WARN("Calibrating...");
      break;
    case nite::SKELETON_TRACKED:
      ROS_WARN("Tracking!");
      break;
    case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
    case nite::SKELETON_CALIBRATION_ERROR_HANDS:
    case nite::SKELETON_CALIBRATION_ERROR_LEGS:
    case nite::SKELETON_CALIBRATION_ERROR_HEAD:
    case nite::SKELETON_CALIBRATION_ERROR_TORSO:
      ROS_WARN("Calibration Failed... :-|");
      break;
    }
  }
}


openni2_camera::NiteJoint NITESkeletalTracker::SkeletonJointToNiteJointMsg(nite::SkeletonJoint joint)
{
  openni2_camera::NiteJoint joint_msg;
  joint_msg.confidence = joint.getPositionConfidence();
  joint_msg.position.x = joint.getPosition().x/1000;
  joint_msg.position.y = joint.getPosition().y/1000;  
  joint_msg.position.z = joint.getPosition().z/1000;
  return joint_msg;
}

}