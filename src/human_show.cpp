// STL Header 
#include  <iostream> 
// OpenCV Header 
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 

// o1. OpenNI Header 
#include <OpenNI.h> // /usr/include/openni2
// n1. NiTE Header 
#include <NiTE.h> 

// namespace 
using namespace std;
using namespace openni;
using namespace nite;
int main( int argc, char **argv )  
{
  // o2. Initial OpenNI 
  OpenNI ::initialize();

  // o3. Open Device 
  Device   mDevice;  
  mDevice.open (ANY_DEVICE);

  // o4. create depth stream 
  VideoStream mDepthStream;  
  mDepthStream.create( mDevice, SENSOR_DEPTH );

  // o4a. set video mode 
  VideoMode mDMode;
  mDMode.setResolution( 640, 480 );
  mDMode.setFps( 30 );
  mDMode.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
  mDepthStream.setVideoMode( mDMode);
  
  // o5. Create color stream 
  VideoStream mColorStream;
  mColorStream.create( mDevice, SENSOR_COLOR );
  
  // o5a. set video mode
  VideoMode mCMode;
  mCMode.setResolution( 640, 480 );
  mCMode.setFps( 30 );
  mCMode.setPixelFormat( PIXEL_FORMAT_RGB888 );
  mColorStream.setVideoMode( mCMode);
   
  // o6. image registration 
  mDevice.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );


  // n2. Initial NiTE 
  NiTE::initialize();

  // n3. create user tracker 
  UserTracker mUserTracker;  
  mUserTracker.create( &mDevice );
  mUserTracker.setSkeletonSmoothingFactor( 0.1f );
  
  // create OpenCV Window 
  cv::namedWindow("User Image", CV_WINDOW_AUTOSIZE);
 
  // p1. start  
  mColorStream.start ();
  mDepthStream.start ();
  
  while ( true )
  {
    // main loop  
    // p2. prepare background 
    cv::Mat cImageBGR;
    
    // p2a. get color frame 
    VideoFrameRef mColorFrame;
    mColorStream.readFrame( &mColorFrame );
    
    // p2b. convert data to OpenCV format 
    const  cv::Mat mImageRGB( mColorFrame.getHeight(), mColorFrame.getWidth(),
                          CV_8UC3 , ( void *)mColorFrame.getData() );
    // p2c. convert form RGB to BGR 
    cv::cvtColor( mImageRGB, cImageBGR, CV_RGB2BGR );
    
    // p3. get user frame 
    UserTrackerFrameRef   mUserFrame; 
    mUserTracker.readFrame( &mUserFrame );
 
    // p4. get users data 
    const nite:: Array < UserData >& aUsers = mUserFrame.getUsers();
    for ( int i = 0; i < aUsers.getSize(); ++ i )
    {
        const  UserData & rUser = aUsers[i];
        // p4a. check user status 
        if ( rUser.isNew () )
        {
            // start tracking for new user
            mUserTracker.startSkeletonTracking( rUser.getId () );
        }
 
        if ( rUser.isVisible() )
        {
            // p4b. get user skeleton 
            const  Skeleton & rSkeleton = rUser.getSkeleton();
            if ( rSkeleton.getState() == SKELETON_TRACKED )
            {
                // p4c. build joints array 
                SkeletonJoint aJoints[15];
                aJoints[ 0] = rSkeleton.getJoint( JOINT_HEAD );
                aJoints[ 1] = rSkeleton.getJoint( JOINT_NECK );
                aJoints[ 2] = rSkeleton.getJoint( JOINT_LEFT_SHOULDER );
                aJoints[ 3] = rSkeleton.getJoint( JOINT_RIGHT_SHOULDER );
                aJoints[ 4] = rSkeleton.getJoint( JOINT_LEFT_ELBOW );
                aJoints[ 5] = rSkeleton.getJoint( JOINT_RIGHT_ELBOW );
                aJoints[ 6] = rSkeleton.getJoint( JOINT_LEFT_HAND );
                aJoints[ 7] = rSkeleton.getJoint( JOINT_RIGHT_HAND );
                aJoints[ 8] = rSkeleton.getJoint( JOINT_TORSO );
                aJoints[ 9] = rSkeleton.getJoint( JOINT_LEFT_HIP );
                aJoints[10] = rSkeleton.getJoint( JOINT_RIGHT_HIP );
                aJoints[11] = rSkeleton.getJoint( JOINT_LEFT_KNEE );
                aJoints[12] = rSkeleton.getJoint( JOINT_RIGHT_KNEE );
                aJoints[13] = rSkeleton.getJoint( JOINT_LEFT_FOOT );
                aJoints[14] = rSkeleton.getJoint( JOINT_RIGHT_FOOT );
                
                // p4d. convert joint position to image 
                cv::Point2f aPoint[15];
                for ( int s = 0; s < 15; ++ s ) 
                {
                    const  Point3f & rPos = aJoints[s].getPosition();
                    mUserTracker.convertJointCoordinatesToDepth( 
                                 rPos.x, rPos.y, rPos.z,
                                 &(aPoint[s].x), &(aPoint[s].y) );
                }
 
                // p4e. draw line 
                cv::line( cImageBGR, aPoint[ 0], aPoint[ 1], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[ 1], aPoint[ 2], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[ 1], aPoint[ 3], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[ 2], aPoint[ 4], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[ 3], aPoint[ 5], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[ 4], aPoint[ 6], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[ 5], aPoint[ 7], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[ 1], aPoint[ 8], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[ 8], aPoint[ 9], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[ 8], aPoint[10], cv::Scalar ( 255 , 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[ 9], aPoint[11], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[10], aPoint[12], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[11], aPoint[13], cv::Scalar ( 255, 0, 0 ), 3 );
                cv::line( cImageBGR, aPoint[12], aPoint[14], cv::Scalar ( 255, 0, 0 ), 3 );
                
                // p4f. draw joint 
                for ( int   s = 0; s < 15; ++ s ) 
                {
                    if ( aJoints[s].getPositionConfidence() > 0.5 )
                        cv::circle( cImageBGR, aPoint[s], 3, cv::Scalar ( 0, 0, 255 ), 2 );
                    else 
                        cv::circle( cImageBGR, aPoint[s], 3, cv::Scalar ( 0, 255, 0 ), 2 );
                }
            }
        }
    }

    // p5. show image 
    cv::imshow( "User Image" , cImageBGR );

    // p6. check keyboard 
    if ( cv ::waitKey( 1 ) == 'q' )
       break ;  
  }
  
  // p7. stop
  mUserTracker.destroy();
  mColorStream.destroy();
  mDepthStream.destroy();
  mDevice.close ();
  NiTE ::shutdown();
  OpenNI ::shutdown();
  
  return 0;  
  
}