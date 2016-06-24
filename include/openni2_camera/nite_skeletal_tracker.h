/* This class wraps NITE2 skeletal tracking library
* and is intended to extend openni2_driver to
* also output people skeletal tracks along with
* rgb/depth information.
*
* 	Author: Ivan Vishniakou (ivan.vishniakou@smail.inf.h-brs.de)
*/

#include <ros/ros.h>
#include <string>
#include <libnite2/NiTE.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include "openni2_camera/NitePeople.h"
#include "openni2_camera/NiteSkeleton.h"
#include "openni2_camera/NiteJoint.h"

namespace openni2_wrapper
{

enum EnumNITETrackerState
{
    eStateStopped = 0,
    eStateRunning = 1,
    eStatePaused = 2
};


class NITESkeletalTracker 
{
public:
  NITESkeletalTracker(ros::NodeHandle& n, ros::NodeHandle& pnh);
  void initialize();
private:
  #define MAX_USERS 10
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  ros::ServiceServer niteStart_;
  ros::ServiceServer niteStop_;
  bool startNiteCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool stopNiteCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  ros::Publisher pub_people_track_;

  std::string frame_id_;
  bool g_visible_users_[MAX_USERS];
  nite::SkeletonState g_skeleton_states_[MAX_USERS];
  nite::UserTracker user_tracker_;
  nite::Status nite_rc_;
  nite::UserTrackerFrameRef user_tracker_frame_;
  ros::Timer user_tracker_update_timer_;
  void userTrackerUpdate(const ros::TimerEvent&);
  void updateUserState(const nite::UserData& user, unsigned long long ts);
  EnumNITETrackerState people_tracker_state_;
  openni2_camera::NiteJoint SkeletonJointToNiteJointMsg(nite::SkeletonJoint skeleton_joint);
};


}