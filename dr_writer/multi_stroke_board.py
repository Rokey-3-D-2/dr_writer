import rclpy
import DR_init
from std_msgs.msg import Float32MultiArray
import numpy as np
import queue, time, math

from dr_writer import config
ROBOT_ID = config.ROBOT_ID
ROBOT_MODEL = config.ROBOT_MODEL
ROBOT_TOOL = config.ROBOT_TOOL
ROBOT_TCP = config.ROBOT_TCP
VEL = config.VEL 
ACC = config.ACC
DRAWING_PAHT = config.DRAWING_PATH
SAMPLE_THRESHOLD = config.SAMPLE_THRESHOLD

# 전역 큐
strokes_queue = queue.Queue()

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

get_end = lambda start: time.time() - start

def listener_callback(msg):
    data = msg.data
    if len(data) % 2 != 0:
        print('길이가 2의 배수가 아닌 데이터를 받았습니다!')
        return
    points = []
    for i in range(0, len(data), 2):
        points.append([data[i], data[i+1]])
    # print(f"[Subscriber] 복원된 좌표 리스트: {points}")
    strokes_queue.put(points)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("multi_stroke_board", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DR_common2 import posx, posj
    from DSR_ROBOT2 import (
        get_tcp, get_tool,
        set_tcp, set_tool, 
        set_ref_coord,
        
        movej, movel, movesx, amovesx,
        check_motion, mwait,
        
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        get_current_posx,
        get_tool_force,
        get_joint_torque,
        
        set_user_cart_coord,
        get_user_cart_coord,

        DR_WHITE_BOARD,
        DR_WHITE_BOARD2,
        DR_AXIS_Z,

        DR_FC_MOD_REL
    )

    tcp, tool = get_tcp(), get_tool()
    print(f'tcp: {tcp}, tool: {tool}')
    if tcp != ROBOT_TCP or tool != ROBOT_TOOL:
        node.destroy_node()
        rclpy.shutdown()
        return

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    # set_ref_coord(DR_WHITE_BOARD)
    set_ref_coord(DR_WHITE_BOARD2)

    node.create_subscription(
        Float32MultiArray,
        DRAWING_PAHT,
        listener_callback,
        10
    )

    white_board_home = posx([0, 0, 0, 0, 0, 0])

    def move_to_home():
        movel(white_board_home, VEL, ACC)
        node.get_logger().info('move to home')

    def split_strokes(strokes):
        """
        strokes: n x 2 numpy array 또는 list of [x, y]
        음수값을 만날 때마다 새로 stroke를 분리해서, 각 stroke를 list로 반환
        """
        splited_strokes = []
        curr_stroke = []

        for pt in strokes:
            x, y = pt
            if x < 0 or y < 0:
                if curr_stroke:
                    splited_strokes.append(curr_stroke)
                    curr_stroke = []
                # 음수인 점은 새 stroke의 시작점이 아님(그냥 건너뜀)
            else:
                curr_stroke.append([x, y])

        if curr_stroke:
            splited_strokes.append(curr_stroke)

        return splited_strokes

    def sample_points(points, max_middle=SAMPLE_THRESHOLD):
        """
        - points: (N, 2) 형태의 list 또는 np.ndarray
        - 시작점, 끝점은 반드시 포함
        - 중간 경유점은 10%만 균등간격 샘플링, 최대 max_middle개
        - 반환값: (샘플링된 경로 list)
        """
        n = len(points)
        if n <= 2:
            # 점이 2개 이하면 그대로 반환
            return points
        
        # 중간점 개수 계산
        middle_points = points[1:-1]
        num_middle = min(max_middle, max(1, int((n - 2) * 0.1)))
        if num_middle < 1:
            # 중간 경유점이 아예 없거나 너무 적음
            return points
        
        idx = np.linspace(0, len(middle_points)-1, num_middle, dtype=int)
        sampled_middle = [middle_points[i] for i in idx]
        sampled = np.vstack([points[0], sampled_middle, points[-1]])
        return sampled.tolist()
    
    def sample_points2(points):

        # we have to develop another algorithm for increasing the detail of the drawing
        
        pass

    def _get_cur_posx():
        start = time.time()
        while time.time() - start < 5:
            try:
                cur_posx = get_current_posx()
                return cur_posx
            except IndexError as e:
                node.get_logger().warn(f'{e}')
                time.sleep(0.1)
                continue
        node.get_logger().error('can not get posx from [get_current_posx]')
        return posx([0,0,0,0,0,0])

    def convert_to_posx(sampled_points):
        return [posx([pt[0], pt[1], _get_cur_posx()[0][2], 0, 0, 0]) for pt in sampled_points]
        
    def release():
        release_compliance_ctrl()
        release_force()

    def pen_down():
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, 20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    def pen_up():
        current_posx = _get_cur_posx()[0]
        current_posx[2] -= 5
        release()
        time.sleep(0.1)
        movel(current_posx, VEL, ACC)

    def check_touch():
        while check_force_condition(DR_AXIS_Z, min=5, max=21): pass
        release_force()
        set_desired_force(fd=[0, 0, 2, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        node.get_logger().info('touch on board!')

    def draw_on_board(traj):
        ret = amovesx(traj, vel=VEL, acc=ACC)
        node.get_logger().info(f'after amovesx: {ret}')

    def traj_length(traj):
        """posx의 3차원 거리 누적 합계(mm 단위)"""
        dist = 0.0
        for i in range(1, len(traj)):
            p0 = traj[i-1]
            p1 = traj[i]

            # posx 앞 3개(x, y, z)만 거리 계산
            d = math.sqrt((p1[0] - p0[0])**2 + (p1[1] - p0[1])**2 + (p1[2] - p0[2])**2)
            dist += d
        return dist  # mm 단위
    
    def estimate_draw_time(traj, vel, acc):
        """전체 경로길이/속도로 예상 소요시간(sec) 반환"""
        length_mm = traj_length(traj)
        if vel <= 0 or acc <= 0 or length_mm <= 0:
            return 0.0

        # we have to develop algorithm for proper waiting time
        
        # return 30
        base_time = length_mm / vel  # [mm] / [mm/s] = [s]
        return base_time * 1.5

    def check_done(traj):
        expected_time = estimate_draw_time(traj, VEL, ACC)
        buffer_time = 2
        total_wait = expected_time + buffer_time
        node.get_logger().info(f'경로길이: {traj_length(traj):.1f}mm, 예상시간: {expected_time:.2f}s (버퍼포함 {total_wait:.2f}s)')
        
        node.get_logger().info('waiting until drawing is done')
        start = time.time()
        before_posx, cur_cnt, max_cnt = _get_cur_posx()[0], 0, 5
        while get_end(start) < total_wait:
            if check_motion() == 0:
                node.get_logger().info(f'[Drawing Success] done: {get_end(start):.2f}s')
                return True

            if before_posx == _get_cur_posx()[0]:
                cur_cnt += 1
            else:
                cur_cnt = 0

            if cur_cnt > max_cnt:
                node.get_logger().warn(f'[Drawing Failure] abnormal behavior')
                return False

            time.sleep(0.1)

        node.get_logger().warn(f'[Drawing Failure] time out: {get_end(start):.2f}s')
        return False

    try:
        move_to_home()
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not strokes_queue.empty():
                strokes = strokes_queue.get()
                splited_strokes = split_strokes(strokes)
                
                for stroke in splited_strokes:
                    node.get_logger().info(f'stroke: {stroke}')

                    sampled_points = sample_points(stroke)
                    node.get_logger().info(f'sampled_points: {sampled_points}')
                    
                    traj = convert_to_posx(sampled_points)
                    node.get_logger().info(f'traj: {traj}')

                    movel(traj[0], VEL, ACC)
                    
                    pen_down()

                    check_touch()

                    draw_on_board(traj)

                    check_done(traj)

                    pen_up()

                move_to_home()
    except KeyboardInterrupt:
        release()
        node.get_logger().info('Shutting down...')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
