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
