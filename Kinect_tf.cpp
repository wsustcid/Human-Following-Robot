<p align="left"></p><pre code_snippet_id="2233768" snippet_file_name="blog_20170228_1_8533170" name="code" class="cpp">/*************************************************************************************  
*Name:kinect_tf.cpp  
*Date:2017-02-28  
*Author:Zhanghuijuan  
*Function:follow people instantly.  
*************************************************************************************/  
//头文件  
#include <ros/ros.h>  
#include <tf/transform_listener.h>  
#include <geometry_msgs/Twist.h>  
#include <sound_play/sound_play.h>//播放音频  
#include <unistd.h>  
#include "std_msgs/String.h"  
  
const char *str_distance，*str_angle;  
int count_running = 0;  
/*主函数*/  
int main (int argc, char** argv)  
{  
    ros::init (argc, argv, "kinect_tf ");//发布的topic  
    ros::NodeHandle nh;  
    sound_play::SoundClient sc;   
    geometry_msgs::Twist cmd;  
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist> ("/cmd_extvel", 10);//发布消息到 topic /cmd_extvel  
    tf::TransformListener listener;  
    ros::Rate rate(10.0);  
    while (node.ok()){  
        //接听消息  
        tf::StampedTransform tf, tf_right_hand, tf_right_shoulder; //tf变换的变量  
        try{          
            listener.lookupTransform("/openni_depth_frame", "/torso_1", ros::Time(0), tf);  //接听话题/torso_1消息，我们查询listener为一个指定的转换，有四个主要参数：  
            //1，2我们想要从1转换到2，3转换的时间，4用transform来存储转换结果  
            double pose_x = tf.getOrigin().x();  
            double pose_y = tf.getOrigin().y();  
            double pose_theta = tf::getYaw(tf.getRotation());  
            //ROS_INFO("Pose of User: [x,y,theta]: [%f, %f, %f]...", x, y, theta);  
            listener.lookupTransform("/openni_depth_frame", "/right_hand_1", ros::Time(0), tf_right_hand);//接听right_hand_1消息  
            double tf_right_hand_z = tf_right_hand.getOrigin().z();  
            listener.lookupTransform("/openni_depth_frame", "/right_shoulder_1", ros::Time(0), tf_right_shoulder);//接听right_shoulder_1消息  
            double tf_right_shoulder_z = tf_right_shoulder.getOrigin().z();  
        }  
        catch (tf::TransformException& ex){  
            ROS_ERROR("Received an exception trying to get information: %s", ex.what());//抛出异常  
            ros::Duration(1.0).sleep();  
            continue;  
        }  
        /*step1:根据距离信息判断，从而发送控制速度指令，针对全向平台*/  
        cmd.linear.x = 0.0;//初始速度x方向为0  
        cmd.linear.y = 0.0;  
        cmd.angular.z = 0.0;  
        //X方向  
        if (pose_x < 1.6)    //距离远近，行为定义  
        {  
            str_distance = "knee thai jean ler";//近了  
            cmd.linear.y = 0.25;  
        }  
        else if (pose_x > 2.0)  
        {  
            str_distance = "knee thai yuan ler";//远了  
            cmd.linear.y = -0.25;  
        }  
        else//距离正好  
        {  
            str_distance = "hen how";  
            cmd.linear.y = 0.0;  
        }  
        //Y方向  
        if (pose_y > 0.05)  
        {  
            str_angle = "zai Geo bian";//偏左  
            cmd.angular.z = 0.15;  
        }  
        else if (pose_y < -0.05)  
        {  
            str_angle = "zai you bian";//偏右  
            cmd.angular.z = -0.15;  
        }  
        else//正好  
        {  
            str_angle = "zai june jain";  
            cmd.angular.z = 0.0;  
        }  
        /*step2:手势指令*/  
        if (tf_right_hand_z >= tf_right_shoulder_z)//手势停止：当右手高于右肩膀时，机器人停止  
        {  
            cmd.linear.x = 0.0;  
            cmd.linear.y = 0.0;  
            cmd.angular.z = 0.0;  
            pub.publish(cmd);  
            ROS_INFO("cmd published: [v_x,v_y,w]:[%f, %f, %f]", cmd.linear.x, cmd.linear.y, cmd.angular.z);  
            //播放声音  
            const char *str1 = "I am stopping now!";  
            sc.repeat(str1);  
            sleep(4);  
            sc.stopSaying(str1);  
            loop.sleep();  
            continue;  
        }  
        //发布速度  
        pub.publish(cmd);  
        ROS_INFO("cmd published: [v_x,v_y,w]:[%f, %f, %f]", cmd.linear.x, cmd.linear.y, cmd.angular.z);  
        count_running++;  
        if (count_running > 30)  
        {  
            const char *str2 = "/home/cnimi/Downloads/wavetest/apert.wav";//调用音频文件  
            sc.startWave(str2);  
            //sleep(1);  
            //sc.stopWave(str2);  
            count_running = 0;  
        }  
    }             
    return 0;  
}  
</pre><br>  
<br>  
<p></p>  
<pre></pre>  
<p></p>
