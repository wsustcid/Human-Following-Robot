/*
v1.0: publish the position of head
V1.5: showing the skeleton image by combining NITE code.
V2:   Rewrite the previous version code into standard c++ code; 
V3:   Publish all joint position by using tf --do some research work!
V4:   Hand gesture recongnition.
*/

#include <ros/ros.h>
#include <iostream>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include "geometry_msgs/Point.h" //added
#include "std_msgs/String.h"
#include <sstream>

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <NiTE.h>

using std::string;
using geometry_msgs::Point; //added
using namespace cv;
using namespace std;

#define MAX_USERS 10
#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
nite::UserTracker userTracker;
nite::Status niteRc;
ros::Publisher position_pub,position_pub1; // define publisher topic
geometry_msgs::Point position; //ws 
float flag=0;
nite::HandTracker mHandTracker;
nite::UserTrackerFrameRef userTrackerFrame;
nite::HandTrackerFrameRef mHandFrame;

void updateUserState(const nite::UserData& user, unsigned long long ts);
void skeletonTracker();
void gestureDetection();

int main(int argc, char **argv) {

    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh, nh_priv("~");
    cv::namedWindow( "Skeleton Image");


    nite::NiTE::initialize();


    position_pub = nh.advertise<geometry_msgs::Point>("/tracker/position", 1); // ws
    position_pub1=nh.advertise<std_msgs::String>("/tracker/position1",1000);


	ros::Rate r(20);
    std::string frame_id("openni_depth_frame");
    nh_priv.getParam("camera_frame_id", frame_id);

	niteRc = userTracker.create();
	if (niteRc != nite::STATUS_OK)
	{
		printf("Couldn't create user tracker\n");
		return 3;
	}
	printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

    mHandTracker.create();
    mHandTracker.startGestureDetection( nite::GESTURE_CLICK );
    mHandTracker.startGestureDetection( nite::GESTURE_WAVE );


	while (ros::ok())
	{
	    flag=0;

	    gestureDetection();
	    skeletonTracker();
	    std_msgs::String msg;

    std::stringstream ss;
    ss << "position.x "<< position.x; // added 
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
   // ROS_INFO("%s", msg.data.c_str());
    position_pub1.publish(msg); // added 
    ros::spinOnce();
		r.sleep();
	}
	return 0;
}


void gestureDetection() // pub tasks to robots according to gestures and pub coordinats of hands
{
        niteRc=mHandTracker.readFrame( &mHandFrame );
		if(niteRc==nite::STATUS_OK)
        {
            const nite::Array<nite::GestureData>& aGestures = mHandFrame.getGestures();

            for( int i = 0; i < aGestures.getSize(); ++ i )
            {
                const nite::GestureData& rGesture = aGestures[i];

                // 对找到的手势进行类型判断，并输出类型
                printf( "Detect gesture ");
                if(rGesture.getType()==nite::GESTURE_CLICK)
                {
                    printf("前进并收回手势---click");
                    flag=1;
                }
                else if(rGesture.getType()==nite::GESTURE_WAVE)
                {
                      flag=2;
                      printf("挥手---wave");
                }
                // 得到的手势信息中还包含了当前手势的坐标位置
                const nite::Point3f& rPos = rGesture.getCurrentPosition();
                cout << " 手势位置为： （" << rPos.x << ", " << rPos.y << ", " << rPos.z << ")" << endl;

                // 以及手势状态，完成状态和进行状态
                if( rGesture.isComplete() )
                    cout << "  手势完成";
                if( rGesture.isInProgress() )
                    cout << "  手势正在进行";

                    cout << endl;
            }
        }
        else
        {
            ROS_WARN_STREAM("Get next frame failed.");
        }
}

void skeletonTracker()
{
     niteRc = userTracker.readFrame(&userTrackerFrame);
		if (niteRc == nite::STATUS_OK)
		{
		    const cv::Mat mHandDepth( userTrackerFrame.getDepthFrame().getHeight(), userTrackerFrame.getDepthFrame().getWidth(), CV_16UC1,
            (void*)userTrackerFrame.getDepthFrame().getData());
            cv::imshow( "Skeleton Image", mHandDepth );
            // 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
            cv::Mat mScaledHandDepth, thresholdDepth;
            mHandDepth.convertTo( mScaledHandDepth, CV_8U, 255.0 / 10000 );
            // 二值化处理，为了显示效果明显
            cv::threshold(mScaledHandDepth, thresholdDepth, 50, 255, 0);

			const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
			for (int i = 0; i < users.getSize(); ++i)
			{
				const nite::UserData& user = users[i];
				updateUserState(user,userTrackerFrame.getTimestamp());
				if (user.isNew())
				{
					userTracker.startSkeletonTracking(user.getId());
					ROS_INFO_STREAM("Found a new user.");
				}
				else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
				{
					ROS_INFO_STREAM("Now tracking user " << user.getId());
					const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD); // other joint is ok
                    float depth_x,depth_y;
                    userTracker.convertJointCoordinatesToDepth(head.getPosition().x, head.getPosition().y, head.getPosition().z, &depth_x, &depth_y);
                    if (head.getPositionConfidence() > .5)
                       printf("%d. (%5.2f, %5.2f, %5.2f) (%5.2f,%5.2f) flag=%f\n", user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z,depth_x,depth_y,flag);
                    if(i==0)//将第一个user信息发送出去
                    {
                        position.x=head.getPosition().x/1000;
                        position.y=head.getPosition().y/1000;
                        position.z=head.getPosition().z/1000;
                        //position.angular.x=depth_x; // ??
                        //position.angular.y=depth_y;
                        //position.angular.z=flag;
                        //printf("%d. (%5.2f, %5.2f, %5.2f) (%5.2f,%5.2f) flag=%f\n", user.getId(), position.linear.x, head.getPosition().y, head.getPosition().z,depth_x,depth_y,flag);
                        position_pub.publish(position);
                    }

                    cv::Point point((int)depth_x, (int)depth_y);
                    circle(thresholdDepth, point, 50, CV_RGB(255,0,0));
                    circle(thresholdDepth, point, 30, CV_RGB(0,255,0));
                    circle(thresholdDepth, point, 10, CV_RGB(0,0,255));
                    cv::imshow( "Skeleton Image", thresholdDepth );//不知道为什么显示不出来啊！！


				}
			}
		}
		else
		{
			ROS_WARN_STREAM("Get next frame failed.");
		}
}

void updateUserState(const nite::UserData& user, unsigned long long ts)
{
	if (user.isNew())
		USER_MESSAGE("New")
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		USER_MESSAGE("Visible")
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		USER_MESSAGE("Out of Scene")
	else if (user.isLost())
		USER_MESSAGE("Lost")

	g_visibleUsers[user.getId()] = user.isVisible();


	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}
