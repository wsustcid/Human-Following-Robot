// STL Header
#include <iostream>
 
// 1. include NiTE Header
#include <NiTE.h>
 
// using namespace
using namespace std;
 
int main( int argc, char** argv )
{
  // 2. initialize NiTE
  nite::NiTE::initialize();
 
  // 3. create user tracker
  nite::UserTracker mUserTracker;
  mUserTracker.create();
 
  nite::UserTrackerFrameRef mUserFrame;
  for( int i = 0; i < 300; ++ i )
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
  }
  nite::NiTE::shutdown();
 
  return 0;
}