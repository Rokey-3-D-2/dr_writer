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

[참고영상](https://drive.google.com/file/d/1qMzNq5Bk4jpOpDGAaUjZX8tPxBqWFZZe/view?usp=sharing)
