#include <ros/ros.h>
#include <iostream> // STL Header
// 1. include NiTE Header
#include <NiTE.h>
using namespace std; // using namespace
 
int main( int argc, char** argv )
{
  ros::init(argc, argv, "human_detection");
  ros::NodeHandle nh, nh_priv("~");
  
  // 2. initialize NiTE
  nite::NiTE::initialize();
 
  // 3. create user tracker
  nite::UserTracker mUserTracker;
  mUserTracker.create();
 
  nite::UserTrackerFrameRef mUserFrame;

  ros::Rate r(10);
  while(ros::ok()) // just detect 300 times!
  {
    // 4. get user frame
    mUserTracker.readFrame( &mUserFrame );
 
    // 5. get users' data
    const nite::Array<nite::UserData>& aUsers = mUserFrame.getUsers();
    for( int i = 0; i < aUsers.getSize(); ++ i )
    {
      const nite::UserData& rUser = aUsers[i];
      if( rUser.isNew() )
        cout << "New User [" << rUser.getId() << "] found." << endl;
      if( rUser.isLost() )
        cout << "User [" << rUser.getId()  << "] lost." << endl;
    }
    
    ros::spinOnce();
		r.sleep();
  }


  nite::NiTE::shutdown();
 
  return 0;
}