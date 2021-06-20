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


