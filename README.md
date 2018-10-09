# 1. OpenNI and Skeleton Tracking

1. The ROS [openni_tracker](http://wiki.ros.org/openni_tracker) package can use the depth data from a Kinect to **track the joint positions** of a person standing in front of the camera. 
2. Using this data, one can program a robot to follow gesture commands signaled by the user. One example of how to do this using ROS and Python can be found in the [pi_tracker](http://wiki.ros.org/pi_tracker) package.
3. Note: all the above resources are based on the kinect v1,  you can <font color=blue> try other drivers provided in <https://github.com/ros-drivers> </font> to achieving some tasks on Kinect V2.

## 1. 1 Installing Openni2, NITE2 and openni2_tracker for Kinect V2

**Related link:** https://blog.csdn.net/myhALAN/article/details/53069901?locationNum=2&fps=11

### 1.1.1 Preparation

- Installing drivers of kinect V2 and the live images can be obtained by running 

  roslaunch kinect2_bridge kinect2_bridge 

### 1.1.2 Install the Openni2

- (This setp can be ingored if you have installed the drives for kinect2)

```language
sudo apt-add-repository ppa:deb-rob/ros-trusty && sudo apt-get update
sudo apt-get install libopenni2-dev
```

- Then refer the path of libfreenect2 to the tird-party applications:

```language
cmake  ..  -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2
```

- test: 

```language
NiViewer2
```

### 1.1.3 Install NITE2

- Download the [NiTE X64-2.2](https://zh.osdn.net/projects/sfnet_roboticslab/downloads/External/nite/NiTE-Linux-x64-2.2.tar.bz2/)binary package
- Unzip and extract the archive to a location of your choice (e.g. ~/tmp)

```language
tar jxvf NiTE-Linux-x64-2.2.tar.bz2
cd NiTE-Linux-x64-2.2/
sudo ./install.sh
cd Samples/Bin/
vim OpenNI.ini
add Repository=/usr/lib/OpenNI2/Drivers the opened file
```

- test: (in the path ~/package/NiTE-Linux-x64-2.2/Samples/Bin)

```language
./UserViewer
```

### 1.1.4 Install Sensorkinect (Optional)

- [download](https://github.com/avin2/SensorKinect/blob/unstable/Bin/SensorKinect093-Bin-Linux-x64-v5.1.2.1.tar.bz2)
- ./install

### 1.1. 5 Install the openni2_tracker package

```language
cd ~/catkin_ws/src
git clone https://github.com/wsustcid/openni2_tracker.git
cd ~/catkin_ws
catkin_make
rospack profile
```

Remark: the original openni2_tracker package can be found here: https://github.com/ros-drivers/openni2_tracker.git

- modify the CMakeLists.txt of the openni2_tracker:

```language
find_package(orocos_kdl) # add this line
# change the include and library path of NIte2 to path that you installed.
find_path(Nite2_INCLUDEDIR
      NAMES NiTE.h
      HINTS /home/software/NiTE-Linux-x64-2.2/Include) 
find_library(Nite2_LIBRARY
         NAMES NiTE2
         HINTS /home/software/NiTE-Linux-x64-2.2/Redist
         PATH_SUFFIXES lib)
```

<font color=blue> **Test:** The openni2_tracker nodes  should be executed in the following directory: /home/package/NiTE-Linux-x64-2.2/Redist </font>

```language
rosrun openni2_tracker openni2_tracker
```

## 1.2 NITE2

NiTE是 PrimeSense 針對 OpenNI 這個深度感應器程式開發 Framework 所推出的一套 middleware，他主要的功能，包括了使用者的偵測、人體骨架的分析與追蹤、手部的追蹤、姿勢手勢辨識等等。

基本设定详见：https://kheresy.wordpress.com/2013/01/07/basic-usage-of-nite2/

### 1.2.1 基本架构

NiTE 2 的架構、和使用概念，基本上都和 OpenNI 2（請參考《[OpenNI 2 簡介](https://kheresy.wordpress.com/2012/12/21/basic-openni-2/)》）非常地相似，所以如果已經知道 OpenNI 2 的程式怎麼寫，應該很容易就可以上手。

主要包括了：

- ##### NITE::NITE

  NiTE 2 的整體環境控制的類別，基本上是用來控制 NiTE 的初始化和停止。

  和 openni::OpenNI 一樣，他所有的函示都是 static 的，使用時不需要實體化變數出來。

- ##### NITE::USERTRACKER

  用來追蹤使用者的類別，包括了使用者的管理，以及骨架追蹤、姿勢偵測等功能；性質和 OpenNI 1.x 的 User Generator 類似。

  讀取出來的資料類型，是 **nite::UserTrackerFrameRef**。

- ##### NITE::HANDTRACKER

  用來進行手部追蹤的類別，除了追蹤手部位置外，也包含了手勢的偵測。基本上算是把 OpenNI 1.x 的 Hands Generator 和 Gesture Generator 的功能加在一起。

  讀取出來的資料類型，是 nite::HandTrackerFrameRef。


基本上，NiTE 只有在程序的開始、結束的時候需要用到，中間基本上是用不到的；使用概念和 openni::OpenNI 一樣。而 UserTracker 和 HandTracker 的使用概念，則也和 openni::VideoStream 相似。更完整的說明，可以參考官方的文件，檔案預設是在 C:\Program Files\PrimeSense\NiTE2\Documentation。

### 1.2.2 基本使用

NiTE 2 雖然需要 OpenNI 2 的功能（NiTE.h 就會去使用 OpenNI.h），但是實際上，如果只是單純要使用 NiTE 所提供的功能的話，在程式碼裡面，是可以完全不出現 OpenNI 的東西的～他最基本的使用流程，大致如下：

1. include NiTE.h 這個 header 檔，之後 NiTE C++ API 的東西，都會在 nite 這個 namespace 下。

2. 呼叫 nite::NiTE::initialize() 來進行整個 NiTE 環境的初始化。

3. 创建一個 NiTE 的对象，如果是要做使用者／骨架的追蹤的話，就是使用 nite::UserTracker，如果是要做手部相關的處理的話，則是用 nite::HandTracker。

   之後再透過他所提供的 create() 函式，來完成 NiTE Tracker 的建立。

   - create() 這個函式有一個參數，可以指定要使用哪一個 openni::Device，如果不指定的話，NiTE 會自己去找一個可以用的，自己開啟來使用。

4. 和 OpenNI 的 VideoStream 不同，NiTE 2 提供的兩個 Tracker，都沒有 start() 和 stop() 的函示可以用來控制開始和結束。所以要使用的話，就是直接進入主迴圈，透過 readFrame() 這個函式，來取得對應的 frame reference（nite::UserTrackerFrameRef 或 nite::HandTrackerFrameRef）；而之後會用到的資料，基本上都在讀取到的 frame reference 中。

5. 程式結束時，呼叫 Tracker 的 destory() 函式，把 Track 關掉。

6. 最後，呼叫 nite::NiTE::shutdown()，關閉整個 NiTE 環境。

### 1.2.3 簡單的範例

上面算是概要性的講了一下 NiTE 2 要怎麼使用，接下來，則是一個極簡單的 NiTE 的範例。在這個範例程式裡面，基本上是直接去使用 NiTE2 的 UserTracker 來進行使用者的追蹤，完全不會直接使用到 OpenNI 的介面。

human_detection.cpp

可以看到，整個流程基本上和 OpenNI 2 非常的相近，最大的差別，就在於 **UserTracker 讀取到的資料，是 UserTrackerFrameRef 的形式**，資料的存取方法和 OpenNI 的 VideoStream 不同而已。

像在個範例面，Heresy 就是先透過 UserTrackerFrameRef 的 getUsers() 這個函式，來取得使用者的列表；這邊的列表，會是一個陣列 nite::UserData 的陣列（和 OpenNI 2 一樣，NiTE 2 又自己定義一個 Array…他們完全沒有想過可以直接用 OpenNI 2 寫好的，或直接用 STL 現成的嗎？ orz），裡面儲存的是目前偵測到的所有使用者。

不過由於這篇文章 Heresy 還不會仔細提 UserTrackerFrameRef 的各項功能細節，所以這邊就只有先使用他提供的 isNew() 和 isLost() 這兩個函式，來判斷這個使用者是剛被偵測到？還是不見了。包括骨架追蹤在內的細節，就等之後的文章再來說明了。

另外，如果要進行錯誤偵測的話，基本上也和 OpenNI 2 的方法（參考《[OpenNI 2 的錯誤處理](https://kheresy.wordpress.com/2012/12/26/openni-error-handle/)》）類似，只是回傳的狀態型別從 openni::Status 變成 nite::Status 而已。

---

### 1.2.4 人体骨架追踪

源代码见：human_tracker.cpp

#### 基本說明

這個範例程式基本上是從《[NiTE2 基本使用](https://kheresy.wordpress.com/2013/01/07/basic-usage-of-nite2/)》這個範例做延伸的，黃底的部分，就是增加的部分。

首先，在主迴圈內，每次透過 UserTracker 的 readFrame() 來取得新的資料的時候，在讀取出來的 mUserFrame 內，都可以透過 getUsers()這個函式，來取得當下的使用者列表；而每一個使用者的資料，都是一個 UserData 的对象，裡面儲存著使用者的資料、以及狀態。由於使用者的偵測，是 UserTracker 自己會進行的，所以這邊不需要其他的步驟，只要把使用者列表讀出來就可以了。

而在骨架追蹤的部分，由於 NiTE 基本上是讓程式開發者自行決定要針對那些使用者進行骨架的追蹤，所以並不會在找到使用者的時候，就自行開始追蹤骨架；因此，**如果要針對使用者進行骨架追蹤的話，就需要呼叫 UserTracker 的 startSkeletonTracking() 這個函式，指定要針對哪一個使用者，進行骨架的追蹤**。

在最簡單的狀況下，就是在每一次更新的時候，針對每一個使用者，都透過 isNew() 這個函式來判斷是否為新的使用者，如果是新的使用者的話，就開始追蹤這個使用者的人體骨架；這部分，就是上面範例程式裡面，「5a」的部分了。－－（测试：可以同时追踪多个吗？还是仅追踪最新的？－－如果可以多个追踪的话把每一个都标记发布出去）

**如果有需要停止使用者的骨架追蹤的話，也可以使用 stopSkeletonTracking() 來針對個別的使用者**，停止骨架的追蹤。

另外，NiTE 2 也捨棄了 OpenNI 1.x 可以選擇要追蹤那些關節的功能，現在都是固定去追蹤全身的骨架，不能像以前一樣，可以只追蹤上半身或下半身了。

------

#### 讀取骨架、關節資料

針對每一個有被追蹤的使用者，則可以透過 UserData 的 getSkeleton()這個函式，來取得該使用者的骨架資料；getSkeleton() 回傳的資料會是 nite::Skeleton 這個型別的資料，他基本上只有兩個函式可以用，一個是 **getState()**，是用來取得目前的骨架資料的狀態的，另一個則是 **getJoint()**，是用來取得特定關節點的資訊的。

基本上，在使用讀取骨架的資料之前，最好先對骨架資料的狀態，做一個簡單的確認；如果是有被正確追蹤的話，得到的狀態應該會是 nite::SKELETON_TRACKED，這樣才有繼續使用的意義。

當確定這筆骨架資料是有用的之後，接下來就可以透過 getJoint() 這個函式，來取得各個關節點的資料了～在 NiTE 2 裡可以使用的關節點，是定義成 nite::JointType 這個列舉型別，它包含了下列十五個關節：

![img](https://public.blu.livefilestore.com/y1pFekDIg26sEv9RsDIgQS3zp5y9vmOeYxLAdNfccfmxzKWfJeR_QpbY1_moWI3KgOW2nQDd2LTJNVpwjB2BUQ1pA/NITE_skeleton.png)



1. JOINT_HEAD
2. JOINT_NECK
3. JOINT_LEFT_SHOULDER
4. JOINT_RIGHT_SHOULDER
5. JOINT_LEFT_ELBOW
6. JOINT_RIGHT_ELBOW
7. JOINT_LEFT_HAND
8. JOINT_RIGHT_HAND
9. JOINT_TORSO
10. JOINT_LEFT_HIP
11. JOINT_RIGHT_HIP
12. JOINT_LEFT_KNEE
13. JOINT_RIGHT_KNEE
14. JOINT_LEFT_FOOT
15. JOINT_RIGHT_FOOT

這十五個關節點，基本上和 OpenNI 1.x 所支援的是相同的，很遺憾，還是不支援手腕和腳踝。

而各關節點透過 getJoint() 這個函式所取得出來的的資料，型別則是 nite::SkeletonJoint，裡面記錄了他是哪一個關節（JointType），以及這個關節目前的**位置（position）和方向（orientation）**；而和 OpenNI 1.x 時相同，他也同時有紀錄位置和方向的可靠度（confidence）。

而如果是要取得關節點的位置的話，就是使用 SkeletonJoint 所提供的 getPosition() 這個函式， 來取得該關節點的位置；而得到的資料的型別會是 nite::Point3f，裡面包含了 x、y、z 三軸的值，代表他在空間中的位置。**他的座標系統基本上就是之前介紹過的、在三度空間內所使用的「世界座標系統」（參考《[OpenNI 2 的座標系統轉換](https://kheresy.wordpress.com/2013/01/14/coordinate-converter-in-openni-2)》），如果需要把它轉換到深度影像上的話，可以使用 UserTracker 所提供的 convertJointCoordinatesToDepth() 這個函式來進行轉換。**（如果要用 OpenNI 的 CoordinateConverter 應該也是可以的。）

不過，由於關節不見得一定準確，如果肢體根本是在攝影機的範圍之外的話，NiTE 也就只能靠猜的，來判斷位置了…而在這種狀況下，位置的準確性會相當低。所以在使用關節位置的時候，個人會建議最好也要透過 getPositionConfidence() 這個函式，來確認該關節位置的可靠度，作為後續處理的參考；他回傳的值會是一個浮點數，範圍是 0 ~ 1 之間，1 代表最可靠、而 0 則是代表純粹是用猜的。

而在上面的例子裡，就是去讀取頭部這個關節點（JOINT_HEAD）的資料，並把它的位置、可靠度都做輸出；如果要得到全身的骨架的資料的話，只要依序針對 15 個關節做讀取就可以了。

而至於關節的方向性的部分，如果需要的話，則是使用 getOrientation()來做讀取；而讀取出來的資料，則不是像之前 OpenNI 1.x 一樣是一個陣列，**而是採用「Quaternion」來代表他的方向**。由於他在概念上算是比較複雜一點的東西，所以在這邊就先不提了，等之後有機會再來講吧…

------

#### 關節資訊的平滑化

由於關節點的資訊在計算的時候，有可能會因為各式各樣的因素，導致有誤差的產生，進一步在人沒有動的情況下，有抖動的問題，所以這個時候，就可能會需要針對計算出來的骨架資訊，做平滑化的動作。和在 OpenNI 1.x 的時候相同，NiTE 2 一樣可以控制人體骨架追蹤的平滑化的參數。**在 NiTE 2 裡，透過 UserTracker 提供的 setSkeletonSmoothingFactor()，設定一個 0 ~ 1 之間的福點數，就可以調整關節資訊的平滑化程度了～**

如果給 0 的話，就是完全不進行平滑化，值愈大、平滑化的程度越高，但是如果給 1 的話，則是會讓關節完全不動。至於要用多大的值？這點就要看個人的應用來決定了。

### 1.2.5 使用OpenCV 画出NiTE2 的人体骨架

而这个范例程式所做的事，主要就是透过OpenNI 2的VideoStream来读取彩色影像当作背景，并透过NiTE 2的UserTracker来读取人体骨架关节点的资讯，并以圆和线、画出来。最后的结果，应该会像右图这样子。

![img](https://kheresy.files.wordpress.com/2013/01/skeleton.jpg?w=500)

下面就是这个程式的主要架构：



```

```

基本上，在前面「o2」到「o6」的部分，都是OpenNI的初始化与设定。在这段程式码里面，除了进行OpenNI的初始化外，还建立出深度、以及彩色影像用的VideoStream；这边之后虽然都不会直接读取到深度影像，不过因为NiTE 2的UserTracker会使用深度影像的资料，所以为了让彩色影像和深度影相的大小是一致的（这边是640×480），所以要建立出深度影像的VideoStream（mDepthStream），并针对他进行相关的设定。

而之后，「n2」、「n3」的部分，则是NiTE 2的初始化，以及UserTracker的建立、设定了。

接下来，就是透过OpenCV，建立一个名为「User Image」的视窗，准备将来用来做显示之用；都好了之后，就是开始OpenNI VideoStream的资料读取（p1），并进入主回圈了～在上面的程式码里面，这部分是先省略掉的，会在接下来的部分做说明。

而里面的「p6」的部分，则是去检查是否有按下键盘的「q」，如果有的话，就离开主回圈，并将NiTE 和OpenNI 的所有物件都关闭、并停止程式（p7 ）。

------

至于回圈里主要处理的部分，程式码基本上就是下面的样子：

```

```

在「p2」的部分，就是从mColorStream里，读取出彩色感应器的影像，并请转换成OpenCV的格式，也就是cImageBGR这个物件，并在之后当作背景来使用。

接下来的「p3」，则是从mUserTracker里，读取出当下的UserTracker分析结果。在「p4」，则是取出分析结果中，使用者的阵列aUser，并针对里面每一个user、依序做处理。而处理的第一部，就是「p4a」，也就是去检查这个user是否是新发现的使用者，如果是的话，就呼叫UserTracker的startSkeletonTracking()这个函式，开始对这个user进行骨架的追踪。

而如果在使用者是可以看的到的状况下，就是要开始处理骨架的资料了～在「p4b」就是先取出使用者的骨架资料，并确定目前正在追踪他的骨架。接下来，则是建立一个大小为15的SkeletonJoint的阵列，并依序把15个关节点的资料都读取出来（p4c）。

由于关节点的位置是在世界座标系统上，所以如果要用OpenCV把他们画到彩色影像上的话，需要先做座标系统的转换，把关节点的位置转换到深度座标系统上；这边，就是上方「p4d」的部分，转换过后的点位资料，会储存在aPoint这个cv::Point2f型别的阵列里面。

到上面为止，基本上都算是关节资料的前置处理。处理之后，接下来就是要把关节的相关资讯画出来了～在「p4e」的部分，是透过cv::line()把对应的关节和关节之间，连线连起来画在cImageBGR上。而在「p4f」的部分，则是在把每一个关节点，依序用cv::circle()，画出一个一个圆；Heresy这边稍微特别一点的，是有去检查各个关节点位置的可靠度，如果可靠度大于0.5的话，就用红色画，不然就用绿色。

最后，就是「p5」的部分，用cv::imshow()把最后的结果、也就是cImageBGR画出来了～



### 1.2.6 NiTE 2 的姿势侦测

https://kheresy.wordpress.com/2013/01/23/pose-detection-in-nite-2/



### Related resources:

- <https://blog.csdn.net/myhALAN/article/details/53069901?locationNum=2&fps=11
- <https://www.cnblogs.com/yemeishu/tag/Kinect/>
- https://kheresy.wordpress.com/2013/01/07/basic-usage-of-nite2/
- <https://github.com/ipa320> --important



#### 10.9.2 Viewing Skeletons in RViz

The ROS openni_tracker package connects to a PrimeSense device such as a Kinect or Asus Xtion and broadcasts a ROS frame transform for each skeleton joint detected in front of the camera. The tf transforms are defined relative to the openni_depth_frame which is embedded inside the camera behind the depth sensor.

#### 10.9.3 Accessing Skeleton Frames in your Programs