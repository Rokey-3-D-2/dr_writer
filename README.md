<!--

# dr_writer
로봇팔 글 따라쓰기 프로젝트

## Requirement
### 1. Set Coord and Tool/TCP in Teaching Pendent

1.1 set coord
- type : 점
- 544.800, -348.390, 619.310, 90.0, -91.825, 90.0

1.2 tool/tcp
- tool : Tool Weight
- tcp : GripperDA_v1

### 2. Add & Build
```
# ~/ros2_ws/src/doosan-robot2/dsr_common2/imp/DSR_ROBOT2.py
# move reference
DR_WHITE_BOARD2 = 110  # your coord id
```

## How to use it

### Terminal 1
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real model:=m0609 host:=192.168.1.100

### Terminal 2
ros2 run dr_write multi_stroke_drawing

### Terminal 3
ros2 run dr_write multi_stroke_board

### reference

[참고영상](https://drive.google.com/file/d/1LCZrqFsJVc3LZubnmM08KKs--QvzH3l-/view?usp=sharing)

-->





# DrawBot
ROS2를 활용한 로봇 자동화 공정 시스템 구현 프로젝트

## 프로젝트 개요

- **목표**: 단순한 공정 자동화를 넘어, 사람의 창작 활동을 로봇이 따라가는 휴먼 인터페이스 기반 협업 시스템 구현
- **주요 기능**: 사용자/이미지의 입력을 로봇이 화이트 보드에 출력
- **사용 장비**: Doosan m0609, RG2 gripper
- **개발 환경**: Ubuntu 22.04, ROS2 Humble
- **주요 기술 스택**: ROS2, DRL
- **기간**: 2025.05.30 ~ 2025.06.05

## 시연 영상

- 영상 링크: [Draw](https://youtu.be/NOHitKVgsI4)
- 영상 링크: [Erase](https://youtu.be/iGi9yeOYFWk)

<div align="center">

[Drawing](https://github.com/user-attachments/assets/f547e5a1-995f-4640-8ace-7e6315c9316e)

[Erase](https://github.com/user-attachments/assets/134f90ca-ee6f-42eb-8520-8dc8504165ad)

</div>

## 다이어그램

<div align="center">

![Flowcahrt](https://github.com/user-attachments/assets/9a451a1b-6d50-435c-8902-3749067926d0)

![Class Diagram](https://github.com/user-attachments/assets/51778c67-cfdf-4e92-8360-83c9d434dc33)

</div>

## 프로젝트 기여자

- 김재권: kimjg1219@gmail.com
- 이호준: hojun7889@gmail.com
- 위석환: llaayy.kr@gmail.com
- 최초인: choinchoi0331@gmail.com

## 교육과정 및 참고자료

### 교육과정

<div align="center">

| 주차 | 기간 | 구분 | 강의실 |
| --- | --- | --- | --- |
| <3주차> | 2025.05.23(금) ~ 2025.05.29(목) | 협동-1 | * 별관 : B-2호 |
| <4주차> | 2025.05.30(금) ~ 2025.06.05(목) | 협동-1 | * 별관 : B-2호 |

| 차시 | 학습내용 | 실습항목 |
| --- | --- | --- |
| 1 | 로보틱스 이론<br>협동로봇 이해 | 이론 교육<br>Python을 이용한 로보틱스 시뮬레이션 |
| 2~4 | 두산 협동로봇 기본 동작 학습 | Dart Platform을 이용한 두산 협동로봇 동작 |
| 5~7 | ROS2를 이용한 두산 협동로봇 운용 | ROS2를 이용한 두산 협동로봇 동작 |
| 8~9 | ROS2 + Moveit을 이용한 협동로봇 동작 | Moveit 이론<br>ROS2 + Moveit 사용법 실습 |
| 10 | 팀 프로젝트 시연 및 발표 |  |

</div>

### 참고자료

- https://manual.doosanrobotics.com/ko/programming-manual/3.3.0/publish/

