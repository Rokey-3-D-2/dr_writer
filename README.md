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

[Drawing](https://github.com/user-attachments/assets/f547e5a1-995f-4640-8ace-7e6315c9316e)

[Erase](https://github.com/user-attachments/assets/134f90ca-ee6f-42eb-8520-8dc8504165ad)

## 다이어그램

![Flowcahrt](https://github.com/user-attachments/assets/9a451a1b-6d50-435c-8902-3749067926d0)

![Class Diagram](https://github.com/user-attachments/assets/51778c67-cfdf-4e92-8360-83c9d434dc33)

## 프로젝트 기여자

- 김재권: kimjg1219@gmail.com
- 이호준: hojun7889@gmail.com
- 위석환: llaayy.kr@gmail.com
- 최초인: choinchoi0331@gmail.com

## 참고 자료

- https://manual.doosanrobotics.com/ko/programming-manual/3.3.0/publish/

