# Automatic-food-delivery-robot-Turtlebot-3-

# 1. Overview

우리는 로봇이 장애물을 피해 특정 위치로 이동한 다음, 사물함을 열고 그 안에 있는 물건(음식)을 집고 원래 위치로 돌아오는 것을 목표로 한다.
우리는 이러한 작업을 진행하기 위해 아래와 같은 플랫폼을 사용하게 되었다.

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

## 2.2. Part B: AR 마커 인식 및 이동

### 2.2.1. Intel d345 camera의 사용 (기존 계획)

기존 계획은 AR maker 인식이 아닌 물체를 직접 인식한 후 인식한 물체의 depth 정보를 이용해 물체 앞까지 이동하는 것이었다. 이를 위해 raspberry pi에서 이번 프로젝트에서 사용할 예정이였던 Intel d345 camera를 인식하게 하기위해 관련된 package를 설치하려고 했다. 아래 주소를 통해서 해당 package를 설치했다.

Intel d345 camera package: https://github.com/IntelRealSense/librealsense/blob/development/doc/installation.md

하지만 설치하는데 매우 오랜 시간이 걸리고, 설치를 다 했음에도 불구하고 많은 오류들과 함께 d345 camera를 인식하지 못했다. 그 이유에 대해서 찾아보니 intel d345 camera를 사용하기 위해서 주로 raspberry pi 4가 사용되는데 이번 프로젝트에서 사용되는 버전은 3이기 때문에 전혀 인식하지 못했다. 이 때문에 raspberry pi 말고 새로운 mini computer가 필요했고, 새로운 mini computer를 사용하기 위해서는 많은 시간과 다루는 방식을 다시 알아가야했기 때문에 다른 방식을 찾았고 이에 대한 해결 방안으로 AR marker를 인식하는 방식으로 바꾸게 되었다.

### 2.2.2. AR 마커 인식 및 이동 코드

AR maker를 인식한 후 해당 AR marker앞으로 이동하는 코드는 turtlebot3 github에 있는 turtlebot3_automatic_parking_vision을 이용했다. 해당 코드는 AR marker를 인식한 후 AR marker의 기존 크기를 이용해 거리를 측정하여 이동하는 방식이다. 해당 코드를 이용해 우리가 원하는 만큼 문이나 물체의 앞으로 가도록 코드를 수정하였다.

turtlebot3_automatic_parking_vision 코드: https://github.com/ROBOTIS-GIT/turtlebot3_applications/tree/master/turtlebot3_automatic_parking_vision

이때 선행되어야하는 작업이 camera calibration 작업이다. camera calibration작업은 왜곡보정을 해주기 때문에 반드시 필요하다. 이번 프로젝트에서는 정밀하게 물체나 문 앞으로 로봇이 이동해야하기 때문에 calibration작업을 거치지 않고서 camera를 이용한다면 이상한 곳으로 로봇이 이동할 것이다. 이번 프로젝트에서 사용한 카메라는 turtlebot에 부착되어 있는 pi camera를 사용했고 이를 calibration할 수 있는 코드는 ros-kinetic파일에 있다. 코드를 사용하는 자세한 방법은 appendix에서 설명되어 있다.

calibration을 완료한 뒤 turtlbot3_automatic_parking_vision의 코드도 수정하였다. 코드는 많은 함수로 구성되있다. 크게 로봇의 위치를 가져오는 함수, AR marker의 위치를 가져오는 함수, 로봇을 회전, 전진, 후진시키는 함수, 로봇의 위치와 AR marker의 위치 정보를 비교하여 로봇이 얼만큼 회전, 전진해야 하는지 결정하는 함수로 구성되어있다. 수정한 코드는 로봇을 회전, 병진시키는 함수와 로봇이 얼만큼 전진, 회전해야 하는지 결정하는 함수 부분이다. 로봇을 회전, 병진시키는 함수에서는 우선 속도를 느리게 만들었다. 너무 빠르게 로봇이 이동하면 그만큼 오차도 커지기 때문에 조금 느린 속도록 회전, 병진운동 하게 만들었다. 로봇이 얼만큼 전진, 회전해야 하는지 결정하는 함수에서는 로봇이 멈추는 거리도 바꾸었다. AR marker와 약 30cm 정도 떨어진 거리에서 멈출 수 있도록 거리를 설정하였다. 또한 코드를 실행시켜보며 로봇이 실제상황에서 어떻게 움직이는지 관찰한 후에 로봇이 이동하는 y축 거리를 조절해주었다. 로봇이 움직이는 순서가 우선 AR marker와 x을 맞추기 위해 y축 방향으로 이동한뒤 90도 회전을 하여 설정한 x축 거리만큼 다가가도록 x축 방향으로 움직인다. 코드의 수정한 부분은 appendix에 있다.

이를 통해 최종적으로 로봇이 움직이는 결과는 동영상 아래에서 확인할 수 있다.


## 2.3. Part C: 로봇 팔 제어

### 2.3.1. c++ 언어를 이용한 로봇 팔 제어 (초기)

로봇 팔 제어를 위한 첫 시도는 c++ 언어를 기반으로 하여 작성되었다. 따라서 move_group_interface를 사용하여 inverse kinematics와 forward kinematics를 풀었다.

![image](https://user-images.githubusercontent.com/81222069/122669941-6fa44d80-d1fa-11eb-9841-aa25e5143fbd.png)

아래와 같이 C++ 코드의 일부를 가져와보았다. x, y, z 좌표를 입력해서 인버스 키네매틱을 푸는 함수 이다.

![image](https://user-images.githubusercontent.com/81222069/122670062-fd803880-d1fa-11eb-818d-00c43f2ff529.png)

 x, y, z 값을 적절히 입력하여 그래스핑 포인트를 찾고 일련의 순서로 사물함을 열어보는 코드를 작성하여 아래와 같이 실험해보았다.
 
 https://user-images.githubusercontent.com/81222069/122670276-21904980-d1fc-11eb-9c5f-90f85fcc5744.mp4

하자민 위의 과정에서 아래와 같이 4 가지 문제점들이 존재했다.

① End effector의 좌표를 설정하는 방식이라 그리퍼가 있는 부분의 각도가 이상하게 설정되는 경우가 꽤 있었다.

② 여러 번 반복할 수록 그리퍼 각도가 점점 이상해졌다. 

③ 사물함이 고정되어 있어야 로봇 팔이 제대로 문을 열 수 있었다.

④ 사물함이 온전히 열리지 않았다.

그래서 포워드 키네매틱 함수를 통해 매번 실행 전후로 각 축의 각도를 0으로 초기화 시켜주었다. 각도 초기화를 통해 올바른 경로로 움직이게 설정한 모습은 아래와 같다.

https://user-images.githubusercontent.com/81222069/122670380-cdd23000-d1fc-11eb-9a2d-cada782063b3.mp4

하지만 아직 아래와 같은 3 가지의 여전히 남은 문제점들이 존재했다.

① 사물함이 같이 땅겨지는 문제

② 다른 언어(c++)로 짜여 있어 통합이 어려운 문제

③ 사물함이 끝까지 열리지 않는 문제

따라서 사물함 설계를 밀어서 여는 방식으로 다시 설계하였고 코드를 python으로 다시 짜게 되었다.

### 2.3.2. python 언어를 이용한 로봇 팔 제어 (최종)

파이썬은 moveit_command를 사용한다. moveit_command는 아래의 깃허브 주소를 통해 기본적인 사용법을 익힐 수 있다.

moveit_command 관련 참고 링크: https://github.com/minwoominwoominwoo7/op_moveit_client

![image](https://user-images.githubusercontent.com/81222069/122670885-2e626c80-d1ff-11eb-8b92-7cceca2e9fe2.png)

깃허브에 쓰여있는 코드를 바탕으로 쓰기 편하게 클래스를 만들었다. 아래의 함수는 클래스에 정의 되어있는 함수의 일부이다.

![image](https://user-images.githubusercontent.com/81222069/122671203-8188ef00-d200-11eb-85e3-1c9da117d74f.png)

먼저 이 코드들이 잘 작동하는지 확인하기 위해 가제보 상에서 확인해보았다.

    roscore
    roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
    roslaunch turtlebot3_manipulation_moveit_config move_group.launch
    roslaunch turtlebot3_manipulation_moveit_config moveit_rviz.launch
    rosrun arm_control arm_move.py

https://user-images.githubusercontent.com/81222069/122671290-d9bff100-d200-11eb-9ecb-d2add7af04e0.mp4

# 3. Appendix

## 3.1. Part A: 라이다를 이용한 Navigation

## 3.2. Part B: AR 마커 인식 및 이동

## 3.3. Part C: 로봇 팔 제어

### 3.3.1. Camera Calibration

우선 computer에서 roscore를 실행한다. calibration을 진행하기 위해 raspberry pi에서 아래 코드를 실행시킨다.

    roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch

해당 코드는 pi camera를 실행시키는 코드이다. 위의 코드를 실행시킨뒤 rostopic list 명령어를 통해 image와 camera가 어떤 노드의 이름으로 발행되는지 확인할 수 있다. 이를 기억한 뒤 아래 코드를 computer에서 실행한다.

    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/rosberrypi_cam/image_raw camera:=/rosberrypi_cam

위의 코드중 image:= 다음과 camera:= 다음 부분을 앞서 rostopic으로 확인한 image와 camera의 발행 노드 이름으로 바꾸어야 한다. 해당 코드를 실행시키면 아래와 같은 화면이 나온다.

![image](https://user-images.githubusercontent.com/81222069/122672717-1216fd80-d208-11eb-8db1-3f929a86a159.png)

calibration을 위해서 특정 체스판을 필요로 하는데 아래 링크에서 체스판을 다운 받을 수 있다.

calibration 체스판: https://github.com/ROBOTIS-GIT/turtlebot3_autorace/blob/master/turtlebot3_autorace_camera/data/checkerboard_for_calibration.pdf

체스판을 앞뒤로, 좌우로 기울여가면서 X, Y, Size, Skew 항목을 어느정도 채우게 되면 calibrate 버튼이 활성화  되고, calibrate된 후에 commit 버튼을 통해 calibration한 정보를 저장할 수 있다. commit이 완료된다면 camera calibration 과정은 완료된 것이다.

### 3.3.2. Camera Calibration

