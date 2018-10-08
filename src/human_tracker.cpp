#include <ros/ros.h>
// STL Header
#include <iostream>
// 1. include NiTE Header
#include <NiTE.h>
// using namespace
using namespace std;
  
int main( int argc, char** argv )
{
  ros::init(argc, argv, "human_tracker");
  ros::NodeHandle nh, nh_priv("~");
  // 2. initialize NiTE
  nite::NiTE::initialize();
  
  // 3. create user tracker
  nite::UserTracker mUserTracker;
  mUserTracker.create();
  
  nite::UserTrackerFrameRef mUserFrame;

  ros::Rate r(10);
  while(ros::ok()) 
  {
    // 4. get user frame
    mUserTracker.readFrame( &mUserFrame );
  
    // 5. get users' data
    const nite::Array<nite::UserData>& aUsers = mUserFrame.getUsers();

    for( int i = 0; i < aUsers.getSize(); ++ i ) // 每次循环，发布每个人的位置？
    {
      // assgin an id for every user
      const nite::UserData& rUser = aUsers[i];
      if( rUser.isNew() )
      {
        cout << "New User [" << rUser.getId() << "] found." << endl;
        // 5a. start tracking skeleton of the current user!
        mUserTracker.startSkeletonTracking( rUser.getId() );
      }
      if( rUser.isLost() )
      {
        cout << "User [" << rUser.getId()  << "] lost." << endl;
      }
  
      // 5b. get skeleton
      const nite::Skeleton& rSkeleton = rUser.getSkeleton();
      if( rSkeleton.getState() == nite::SKELETON_TRACKED )
      {
        // if is tracked, get joints
        const nite::SkeletonJoint& rHead
                   = rSkeleton.getJoint( nite::JOINT_HEAD );
        const nite::Point3f& rPos = rHead.getPosition();

        cout << "[" << rUser.getId() << "]" << " > " << rPos.x << "/" << rPos.y << "/" << rPos.z;
        cout << " (" << rHead.getPositionConfidence() << ")" << endl;
      }

    }
    
    ros::spinOnce();
    r.sleep();
  }

  nite::NiTE::shutdown();
  
  return 0;
}