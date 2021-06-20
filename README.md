# Automatic-food-delivery-robot-Turtlebot-3-

# 1. Overview

우리는 로봇이 장애물을 피해 특정 위치로 이동한 다음, 사물함을 열고 그 안에 있는 물건(음식)을 집고 원래 위치로 돌아오는 것을 목표로 한다. 우리는 이러한 작업을 진행하기 위해 아래와 같은 플랫폼을 사용하게 되었다.

## 1.1. Platform

### 1.1.1. Turtle bot 3 waffle pi

로봇팔을 사물함 앞으로 이동시켜 줄 이동수단이 필요하기에 아래와 같이 라이다가 있고, pi 카메라가 내장되어 있는 로보티즈 사의 Turtle bot 3 waffle pi를 사용하게 되었다.

![image](https://user-images.githubusercontent.com/81222069/122668657-15a08980-d1f4-11eb-83a3-2916409de6e6.png)
관련 메뉴얼: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

### 1.1.2. Open manipulator x

물건을 집을 로봇팔이 필요하기에 아래와 같이 Dtnamixel x430 서보모터 5개가 결합된 로보티즈 사의 Open manipulator x를 사용하게 되었다. 이 로봇 팔은 Turtle bot 위에 부착되어 같이 움직일 것이다.

![image](https://user-images.githubusercontent.com/81222069/122668829-fe15d080-d1f4-11eb-8f96-39a02a23b992.png)
관련 메뉴얼: https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/#overview

## 1.2. 시나리오

①  사물함까지 네비게이팅 (맵핑은 사전 작업)

②  사물함에 붙은 ar 마크를 통해 사물함 버튼의 정확한 위치 파악 후 이동

③  로봇 팔을 통해 사물함을 염

④  사물함 안의 물건에 위치한 AR 마크를 통해 물건의 정확한 위치 파악 후 이동

⑤  로봇 팔을 통해 물건을 집음

⑥  global localization을 통해 자신의 위치를 알아냄

⑦  원점 (0,0)으로 돌아감

⑧  물체를 바닥에 내려놓음


# 2. Methodology & Results

우리는 1.2.의 시나리오를 이루기 위해서 총 4개의 part (A,B,C)로 나누어 작업을 진행했다.


## 2.1. Part A: 라이다를 이용한 Navigation

### 2.2.1. SLAM을 이용한 mapping과 Rviz를 이용한 navigation

로봇을 대략적으로 물체 앞으로 이동시키기 위해서 이번 프로젝트에서는 rviz를 이용한 navigation module을 사용하기로 했다. 해당 module은 사용자가 원하는 위치를 입력하면 로봇의 경로를 장애물을 회피하여 생성시켜준다. Navigation module을 사용하기 위해선 map에 대한 정보가 필요하게 되는데 이는 SLAM을 이용했다. SLAM을 통해 얻은 map 정보는 아래와 같다.

![image](https://user-images.githubusercontent.com/81222069/122674430-1cd59080-d210-11eb-84b7-f0fba8c6b2c5.png)

이번 프로젝트에서 사용한 SLAM은 라이더 센서를 이용한 SLAM이다. SLAM을 통해 로봇은 map의 특징점을 뽑아낼 수 있게 되며 이를 바탕으로 현재 자신의 위치가 map에서 어디에 위치해있는지를 확인할 수 있게 된다. SLAM을 실행시킨뒤 map 정보를 얻을때는 키보드조작을 통해 로봇을 움직이며 map에 대한 정보를 얻었다. SLAM을 통해 얻은 map 정보를 로봇에게 전달한 뒤 rviz를 이용한 ros의 navigation 모듈을 이용해 로봇을 원하는 위치로 대략적으로 보내줄 수 있다. 아래는 navigation 모듈을 통해 로봇을 이동시키는 모습이다.

https://user-images.githubusercontent.com/81222069/122676556-902fd000-d219-11eb-9e77-f6feba8b5478.mp4

### 2.2.2. Global localization

SLAM을 통해 map에대한 정보를 로봇이 안다고 하더라도 로봇의 초기 위치는 전혀 모른다. 이를 global localiztion을 통해 알아낼 수 있다. 이번 프로젝트에선 초기 위치를 (0,0)으로 주었기 때문에 처음 시작할 때에는 global localiztion이 필요하지 않게 된다. 만약 다른 위치에서 시작하고 싶다면 그때 global localization을 사용해야 한다. 이번 프로젝트에서는 물체를 집은뒤에 global localization을 해주었다. 로봇이 물체를 집은 뒤 다시 navigation 모듈을 통해 초기 위치로 돌아와야하기 때문에 다시 rviz를 실행시켜야 한다. 이때 초기위치가 (0,0)이 아닌 박스 앞이기 때문에 map정보와 로봇이 인지하는 주변 환경과 매칭이 되지 않게 된다. global localization을 하기 위해 searching.py라는 코드를 작성했다. 해당 코드는 로봇을 회전시키는 코드로 회전을 하면서 로봇이 인지한 주변 환경과 map정보를 일치시키는 코드이다. 이후 CM.py 코드를 작성해 맵을 깔끔히 만들었다. 해당 코드와 과정은 appendix에서 자세히 나와있다.


## 2.2. Part B: AR 마커 인식 및 이동

### 2.2.1. Intel d345 camera의 사용 (기존 계획)

기존 계획은 AR maker 인식이 아닌 물체를 직접 인식한 후 인식한 물체의 depth 정보를 이용해 물체 앞까지 이동하는 것이었다. 이를 위해 raspberry pi에서 이번 프로젝트에서 사용할 예정이였던 Intel d345 camera를 인식하게 하기위해 관련된 package를 설치하려고 했다. 아래 주소를 통해서 해당 package를 설치했다.

*Intel d345 camera package:* https://github.com/IntelRealSense/librealsense/blob/development/doc/installation.md

하지만 설치하는데 매우 오랜 시간이 걸리고, 설치를 다 했음에도 불구하고 많은 오류들과 함께 d345 camera를 인식하지 못했다. 그 이유에 대해서 찾아보니 intel d345 camera를 사용하기 위해서 주로 raspberry pi 4가 사용되는데 이번 프로젝트에서 사용되는 버전은 3이기 때문에 전혀 인식하지 못했다. 이 때문에 raspberry pi 말고 새로운 mini computer가 필요했고, 새로운 mini computer를 사용하기 위해서는 많은 시간과 다루는 방식을 다시 알아가야했기 때문에 다른 방식을 찾았고 이에 대한 해결 방안으로 AR marker를 인식하는 방식으로 바꾸게 되었다.

### 2.2.2. AR 마커 인식 및 이동 코드

AR maker를 인식한 후 해당 AR marker앞으로 이동하는 코드는 turtlebot3 github에 있는 turtlebot3_automatic_parking_vision을 이용했다. 해당 코드는 AR marker를 인식한 후 AR marker의 기존 크기를 이용해 거리를 측정하여 이동하는 방식이다. 해당 코드를 이용해 우리가 원하는 만큼 문이나 물체의 앞으로 가도록 코드를 수정하였다.

*turtlebot3_automatic_parking_vision 코드:* https://github.com/ROBOTIS-GIT/turtlebot3_applications/tree/master/turtlebot3_automatic_parking_vision

이때 선행되어야하는 작업이 camera calibration 작업이다. camera calibration작업은 왜곡보정을 해주기 때문에 반드시 필요하다. 이번 프로젝트에서는 정밀하게 물체나 문 앞으로 로봇이 이동해야하기 때문에 calibration작업을 거치지 않고서 camera를 이용한다면 이상한 곳으로 로봇이 이동할 것이다. 이번 프로젝트에서 사용한 카메라는 turtlebot에 부착되어 있는 pi camera를 사용했고 이를 calibration할 수 있는 코드는 ros-kinetic파일에 있다. 코드를 사용하는 자세한 방법은 appendix에서 설명되어 있다.

calibration을 완료한 뒤 turtlbot3_automatic_parking_vision의 코드도 수정하였다. 코드는 많은 함수로 구성되있다. 크게 로봇의 위치를 가져오는 함수, AR marker의 위치를 가져오는 함수, 로봇을 회전, 전진, 후진시키는 함수, 로봇의 위치와 AR marker의 위치 정보를 비교하여 로봇이 얼만큼 회전, 전진해야 하는지 결정하는 함수로 구성되어있다. 수정한 코드는 로봇을 회전, 병진시키는 함수와 로봇이 얼만큼 전진, 회전해야 하는지 결정하는 함수 부분이다. 로봇을 회전, 병진시키는 함수에서는 우선 속도를 느리게 만들었다. 너무 빠르게 로봇이 이동하면 그만큼 오차도 커지기 때문에 조금 느린 속도록 회전, 병진운동 하게 만들었다. 로봇이 얼만큼 전진, 회전해야 하는지 결정하는 함수에서는 로봇이 멈추는 거리도 바꾸었다. AR marker와 약 30cm 정도 떨어진 거리에서 멈출 수 있도록 거리를 설정하였다. 또한 코드를 실행시켜보며 로봇이 실제상황에서 어떻게 움직이는지 관찰한 후에 로봇이 이동하는 y축 거리를 조절해주었다. 로봇이 움직이는 순서가 우선 AR marker와 x을 맞추기 위해 y축 방향으로 이동한뒤 90도 회전을 하여 설정한 x축 거리만큼 다가가도록 x축 방향으로 움직인다. 코드의 수정한 부분은 appendix에 있다.

이를 통해 최종적으로 로봇이 움직이는 결과는 동영상 아래에서 확인할 수 있다.

https://user-images.githubusercontent.com/81222069/122676619-c66d4f80-d219-11eb-8596-5dcb9c239527.mp4

## 2.3. Part C: 로봇 팔 제어

### 2.3.1. c++ 언어를 이용한 로봇 팔 제어 (초기)

우리에 프로젝트에서 로봇팔이 수행해야 하는 역할은 2가지이다. 첫 번째는 문을 여는 것이고 두 번째는 물건을 집는 것이다. 사실 이 모든 것은 카메라나 다른 센서 없이는 불가능하다. 하지만, 우리는 로봇팔에 카메라를 다는 것을 실패했고 다른 방식 즉 ar markar을 최대한 활용해서 물건을 집는 방식을 채택하였다. ar marker를 통해 최대한 같이 위치로 로봇을 이동시킬 수 있다면 남은 것은 정해진 좌표를 따라 로봇팔이 움직여주기만 하면 되는 것이다. 로봇팔을 정해진 위치로 이동시키기 위해서는 기구학과 역기구학을 이용해야 한다. ROS에서는 이런 역기구학과 기구학을 풀어주는 함수가 기본적으로 내장되어 있으므로 최대한 이를 활용하기로 하였다. 우리는 ROS에 내장된 함수 중에 moveit 라이브러리를 활용하였고 moveit 라이브러리의 도식도는 아래와 같다.

![image](https://user-images.githubusercontent.com/81222069/122676034-0a128a00-d217-11eb-98ec-fdc4f8ecaaf3.png)

처음에는 c++ 코드를 사용했기 때문에 위에 도식도 중에 move_group_interface를 활용했었다. 로봇팔이 문을 열거나 물건을 잡기 위한 과정은 다음과 같다. 첫째로 manipulation_gui를 통해 로봇팔이 가야 하는 좌표들을 찾아낸다. 두 번째로 그 좌표들을 순서대로 역기구학 함수에 집어넣는다. 세 번째로 스텝별로 적절한 시간을 부여하며 최적의 시간을 찾아낸다. 우리가 열어야 하는 사물함은 아래 사진과 같았고 로봇팔이 이 문을 여는 영상도 아래에 첨부하였다.

https://user-images.githubusercontent.com/81222069/122676056-26aec200-d217-11eb-9b63-3ffbd36be987.mp4

영상을 보면 알 수 있듯이 로봇팔이 문을 열긴 하지만 조금 부자연스럽게 움직이는 것을 확인할 수 있다. 로봇팔이 이렇게 부자연스럽게 움직이는 이유는 역기구학을 통해서만 로봇팔을 움직였기 때문이다. 로봇팔의 끝점이 우리가 입력해놓은 좌표로 이동하는 방법은 여러 가지가 있기에 내장되어있는 역기구학 함수만으로는 우리가 원하는 경로를 설정할 수 없었다. 따라서, 여러번 반복할수록 로봇팔의 형태가 기괴한 형태로 변했고 이를 해결하기 위해 실행 전후로 각 축의 각도를 0도로 초기화해주는 코드를 만들어 문을 열기 전후로 집어 넣어주었다.

https://user-images.githubusercontent.com/81222069/122676084-4c3bcb80-d217-11eb-82dc-efd0925cfb2e.mp4

이제는 로봇팔이 좀 더 자연스럽게 움직이는 것을 확인할 수 있다. 하지만, 아직 해결해야 할 문제가 많았다. 일단 사물함이 완벽하게 고정이 되지 않아 사물함이 매번 같이 당겨지는 문제가 있었다. 또 다른 문제는 영상을 보면 알 수 있듯이 로봇팔의 움직임만으로는 사물함의 문을 완벽하게 열 수 없었다. 우리가 받은 로봇팔은 4축으로 x축(앞뒤), z축(위아래)밖에 움직일 수 없었기 때문이다. 가장 큰 문제는 로봇팔을 움직이는 언어가 c++이라는 것인데 로봇팔 이외의 코드들이 python으로 짜여있어 통합하기 어려웠다. 그래서 사물함 설계를 버튼을 누르면 여는 방식으로 다시 설계하였고 코드는 python으로 다시 짰다.

파이썬은 c++과 다르게 위의 도식도 중 moveit_command를 사용하게 되었다. moveit_command 사용과 관련해서는 아래는 깃허브를 참고하게 되었다. 이를 통해 기본적인 moveit_command의 사용법을 익힐 수 있었다.

*moveit_command 관련 참고 코드:* https://github.com/minwoominwoominwoo7/op_moveit_client

깃허브에 쓰여있는 코드를 바탕으로 하되 좀 더 편하게 활용하기 위해서 클래스를 만들었다. 그리고 먼저 이 코드들이 잘 작동하는지 확인하기 위해 가제보 상에서 확인해보았다.

https://user-images.githubusercontent.com/81222069/122676339-ad17d380-d218-11eb-8e27-4f5c81780ac3.mp4

가제보에서 잘 작동하는 것을 확인하였고 이를 실제 상황에 바로 적용시켜보았다.

https://user-images.githubusercontent.com/81222069/122676377-cde02900-d218-11eb-8960-8d6823ac5e5a.mp4

로봇팔이 사물함의 문을 잘 여는 것을 확인할 수 있다. 이와 비슷하게 물건을 잡는 코드도 짜 주었다.

# 3. Appendix

## 3.1. Part A: 라이다를 이용한 Navigation

### 3.1.1. SLAM & Navigation 코드

우선 remote pc에서 roscore를 실행시킨다. 그 후 pc에서 로봇에 접속하기 위해 아래 명령어를 통해 접속한다.

    ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}
    
이번 프로젝트에서 사용한 turtlebot 모델은 waffle_pi이기 때문에 remote pc에서 아래 코드를 통해 로봇의 모델을 전달한다.

    export TURTLEBOT3_MODEL=waffle_pi
    
로봇을 초기 위치에 둔 이후 remote pc에서 SLAM을 아래 코드로 실행한다.

    roslaunch turtlebot3_slam turtlebot3_slam.launch

위의 코드를 실행 시키면 아래 그림과 같이 SLAM이 실행된다.

![image](https://user-images.githubusercontent.com/81222069/122677451-3fba7180-d21d-11eb-9a3b-85b19c8a7fef.png)

이후 아래 코드를 이용해 키보드 조작으로 통해 로봇을 움직이며 map에 대한 정보를 얻는다.

    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    
이번 프로젝트에서 사용하는 turtlebot의 특성상 뒤쪽의 map 정보는 잘 얻지 못했기 때문에 충분한 회전도 필요하다. map 정보를 충분히 얻었다면 아래 코드를 이용해 pc에 저장한다.

    rosrun map_server map_saver –f ~/map
    
이후 로봇을 다시 초기 위치인 (0,0)에 위치한 후 아래 코드를 통해 navigation 모듈을 실행시킨다.

    roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
    
위의 코드로 실행시키게 되면 원하는 위치를 손으로 지정해주어야 한다. 자동화를 위해 정해진 위치로 보내고 싶다면 map 정보를 저장한 뒤 patrol.py코드를 이용한다면 정해진 위치로 자동으로 navigate할 수 있다.

## 3.2. Part B: AR 마커 인식 및 이동

### 3.2.1. Camera Calibration 코드

우선 computer에서 roscore를 실행한다. calibration을 진행하기 위해 raspberry pi에서 아래 코드를 실행시킨다.

    roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch

해당 코드는 pi camera를 실행시키는 코드이다. 위의 코드를 실행시킨뒤 rostopic list 명령어를 통해 image와 camera가 어떤 노드의 이름으로 발행되는지 확인할 수 있다. 이를 기억한 뒤 아래 코드를 computer에서 실행한다.

    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/rosberrypi_cam/image_raw camera:=/rosberrypi_cam

위의 코드중 image:= 다음과 camera:= 다음 부분을 앞서 rostopic으로 확인한 image와 camera의 발행 노드 이름으로 바꾸어야 한다. 해당 코드를 실행시키면 아래와 같은 화면이 나온다.

![image](https://user-images.githubusercontent.com/81222069/122677489-6ed0e300-d21d-11eb-8973-41685fe55c81.png)

calibration을 위해서 특정 체스판을 필요로 하는데 아래 링크에서 체스판을 다운 받을 수 있다.

*calibration 체스판:* https://github.com/ROBOTIS-GIT/turtlebot3_autorace/blob/master/turtlebot3_autorace_camera/data/checkerboard_for_calibration.pdf

체스판을 앞뒤로, 좌우로 기울여가면서 X, Y, Size, Skew 항목을 어느정도 채우게 되면 calibrate 버튼이 활성화  되고, calibrate된 후에 commit 버튼을 통해 calibration한 정보를 저장할 수 있다. commit이 완료된다면 camera calibration 과정은 완료된 것이다.

### 3.2.2. turtlebot3_automatic_parking_vision 코드

본 코드를 실행시키기에 앞서 코드를 수정해야한다. 우선 로봇을 회전, 병진 시키는 코드는 아래와 같다.

![image](https://user-images.githubusercontent.com/81222069/122672996-6ff81500-d209-11eb-8724-2f722a128bfe.png)

해당 코드에서 fnGoStraight() 함수의 twist.linear.x 부분은 0.1이하로 바꿔줘야한다. 마찬가지로 fnTrackMarker() 함수의 twist.angular.z를 –angular_z * 0.7로 바꾼다.

AR marker에 얼마나 가까이 갈지 정하는 함수는 아래와 같다.

![image](https://user-images.githubusercontent.com/81222069/122673036-9453f180-d209-11eb-91e7-e1b4d530c1cd.png)

위의 코드에서 if abs(self.marker_2d_pose_x) < 0.19 부분을 0.19에서 0.30으로 바꾸어 주었다. 약 30cm 거리를 두고 AR marker 앞에서 멈추게 하는 것이 목표이기 때문이다. 만약 로봇을 x, y축을 동시에 움직이고 싶다면 추가적으로 코드를 수정하면 된다. 하지만 오차를 최대한 줄이기 위해서는 linear하게만 움직이는 것이 최선의 방법이므로 이번 프로젝트에서도 y축으로먼저 움직이고 이후 x축으로만 움직였다.

코드를 수정한뒤 automatic_parking_vision을 실행시키는 순서는 아래와 같다.

    sudo apt-get install ros-kinetic-ar-track-alvar
    sudo apt-get install ros-kinetic-ar-track-alvar-msgs
    cd ~/catkin_ws/src
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications.git
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs.git
    cd ~/catkin_ws && catkin_make
    (remote pc) roscore
    (turtlebot) roslaunch turtlebot3_bringup turtlebot3_robot.launch
    (turtlebot) roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch
    (remote pc) rosrun image_transport republish compressed in:=raspicam_node/image raw out:=raspicam_node/image
    (remote pc) ROS_NAMESPACE =raspicam_node rosrun image_proc image_proc image_raw:=image _approximate_s =true _queue_size:=20
    (remote pc) roslaunch turtlebot3_automatic_parking_vision turtlebot3_automatic_parking_vision.launch
    
## 3.3. Part C: 로봇 팔 제어

### 3.3.1. 가제보 실행 명령어 순서

    roscore
    roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
    roslaunch turtlebot3_manipulation_moveit_config move_group.launch
    roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
    rosrun arm_control arm_move.py

### 3.3.2. 실제 Turtle bot 구동 실행 명령어 순서

    roscore
    ssh pi@192.168.0.13 (ras)
    roslaunch turtlebot3_bringup turtlebot3_robot.launch (Pc)
    roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
    roslaunch turtlebot3_manipulation_moveit_config move_group.launch
    roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
    rosrun arm_control arm_move.py

## 3.4. 사물함 설계 관련

### 3.4.1. 사물함의 프로토타입 설계 및 제작 (V1)

로봇 팔이 문을 열고 물건을 꺼내는 시나리오에서의 사물함을 제작하였다. 먼저 전체적인 기능을 간략한 형태로 구현한 프로토타입을 제작하여, 성능과 구현 가능성, 내구성 등을 평가하였다. 프로토타입을 제작하기 위해 하드보드지를 테이프와 글루건으로 고정하여 개략적으로 설계하였다. 벽면, 벽 보강재, 문, 손잡이 등의 구조는 하드보드지로 제작하였다. 문은 자석을 이용하여 손잡이를 당겨 열 수 있도록 설계하였으며, 경첩을 이용하여 문의 최대 및 최소 열림 각도를 조절하였고, 탄성이 있는 PP 플라스틱 판을 겹쳐 문 손잡이를 조금만 당겨도 최대 각도까지 열릴 수 있도록 제작하였다.

![image](https://user-images.githubusercontent.com/81222069/122673387-33c5b400-d20b-11eb-9566-66720f61227a.png)
![image](https://user-images.githubusercontent.com/81222069/122673381-2f010000-d20b-11eb-980c-da8d57476dd8.png) *제작된 사물함 (V1)*

### 3.4.2. 문제점 파악

프로토타입을 통해 로봇의 그리퍼가 사물함 문을 여는 시나리오를 구현하였을 때, 여러 문제점이 발생하였다. 첫 번째로, 사물함이 고정되지 않아 그리퍼가 문을 제대로 여는 데 어려움이 있었다. 문을 열 때 자석에 연결된 부분이 같이 당겨지는 문제가 있었다. 두 번째로, 벽면과 기타 구조물들에 사용된 하드보드지 재질의 강성(stiffness)이 작아, 외부의 작은 힘에도 크게 변형이 일어났다. 특히 로봇의 그리퍼가 문 손잡이를 잡고 당길 때, 전체적인 구조가 크게 변형되었다. 마지막 세 번째로, 여러 번의 테스트를 거친 이후 구조물에 소성변형이 일어나, 테스트 이전과 이후의 구조가 변하였다. PP 플라스틱 판의 탄성력을 이용하여 문이 스스로 열리도록 설계하였으나, 몇 번의 테스트 이후 PP 플라스틱 판에 발생한 소성변형 때문에 구조물을 부분적으로 교체해야 했다.

![image](https://user-images.githubusercontent.com/81222069/122673475-961eb480-d20b-11eb-822c-0c921ed0ff4e.png) *사물함 문을 여는 시나리오 테스트*

### 3.4.3. 사물함의 프로토타입 설계 및 제작 (V2)

대부분의 문제점이 구조물의 재질 자체의 내구성 때문에 발생하였다. 따라서 기존에 사용된 하드보드지 재질을 강성이 높은 재료로 변경하였다. 이 때 사용된 재료는 600×900 두께 10mm 우드락이며, 가공이 쉽고 강성이 높아 시나리오를 구현하는 데 적합하다고 판단하였다. 또한 로봇으로 인해 발생한 힘이 사물함의 문 부분에 집중되었기 때문에, 사물함의 문 부분의 강도를 높여야 시나리오를 안정적으로 수행할 수 있었다. 따라서 3D 프린터를 통해 제작한 보강재를 문 부분에 부착하여, 외부 힘에 대한 구조물의 강도를 높일 수 있었다. 3D 프린터로 출력하기 위한 보강재의 3D 모델링은 Solidworks 프로그램을 이용하여 제작하였으며, 사용된 3D 프린터는 Flashforge Finder 2.0 이고, 출력물의 강성과 내구성이 높아야 하기 때문에 PLA 재질의 필라멘트를 사용하였다.

![image](https://user-images.githubusercontent.com/81222069/122673515-c403f900-d20b-11eb-8e16-31a6d211f713.png) *Solidworks를 통해 제작한 문 보강재의 3D 모델링*

![image](https://user-images.githubusercontent.com/81222069/122673530-d41bd880-d20b-11eb-8aa9-93e98eabd30e.png)
![image](https://user-images.githubusercontent.com/81222069/122673644-5c01e280-d20c-11eb-9824-c94a1aa4a0c7.png) *Flashforge Finder 2.0 와 PLA filament*

또 사물함의 문을 여는 방법을 바꿔 프로토타입에서 발생한 문제를 해결하고자 했다. 기존의 문 손잡이를 당겨 여는 방법은 사물함 구조물에 큰 힘을 가하여, 구조물에 부담이 될 수 있었다. 또한 손잡이를 당겨 문을 열었을 때, 탄성이 있는 재질을 이용하여 스스로 문이 열리도록 하는 방법은, 탄성이 있는 재질에 소성변형이 발생하였을 때 구조물을 부분적으로 교체해야 했기 때문에 내구성 측면에서 부실하였다. 따라서 로봇의 그리퍼가 문에 부착된 버튼을 누르면, 문의 회전축 부분에 부착된 서보모터(SG90)가 회전하여 열리도록 설계를 변경하였다. 입력장치와 서보보터를 동작시키기 위해서 아두이노 uno 보드가 사용되었다.

![image](https://user-images.githubusercontent.com/81222069/122673994-370e6f00-d20e-11eb-87a4-e3a68d04a0fb.png) *제작된 사물함 (V2)*

최종적으로 제작된 사물함의 작동영상은 아래와 같다.

https://user-images.githubusercontent.com/81222069/122676808-91153180-d21a-11eb-8fe5-8819cbc764cd.mp4

# 4. 시연영상 및 전체코드 다운로드

아래 드라이브 링크 내 영상은 위의 과정들을 거쳐 만들어진 종합적인 Task를 수행하는 장면을 담고 있다.
https://drive.google.com/file/d/19kDkX1ZS2LS7qNJWg1KdEW_JKrjYBiFB/view?usp=sharing

![image](https://user-images.githubusercontent.com/81222069/122677466-5660c880-d21d-11eb-9aaf-c1b6e933cf36.png)

또한 전체적인 코드를 압축하여 아래 드라이브 링크에 올려두었다.

https://drive.google.com/file/d/14X-BTSVutT-NmOuz50ipUG4lrQy5I7ul/view?usp=sharing
