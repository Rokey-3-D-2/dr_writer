import rclpy
from rclpy.node import Node
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
SAMPLE_RATIO = config.SAMPLE_RATIO

import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DR_common2 import posx

ON, OFF = 1, 0

get_end = lambda start: time.time() - start

class Gripper:
    def set_dependencies(
            self,
            
            set_digital_output,
            get_digital_input,
            wait,
        ):

        self.get_digital_input = get_digital_input
        self.wait = wait
        self.set_digital_output = set_digital_output

    def wait_digital_input(self, sig_num):
        while not self.get_digital_input(sig_num):
            self.wait(0.5)
            print("[wait_digital_input]")
            pass

    def open_grip(self):
        self.set_digital_output(2, ON)
        self.set_digital_output(1, OFF)
        # self.wait_digital_input(2)
        self.wait(0.5)

    def close_grip(self):
        self.open_grip()
        self.set_digital_output(1, ON)
        self.set_digital_output(2, OFF)
        # self.wait_digital_input(1)
        self.wait(0.5)

class Mover:
    white_board_home = posx([0, 0, 0, 0, 0, 0])
    pen_holder = posx([2.970, 574.210, -136.250, 90.0, 88.17, -91.58])
    eraser_holder = posx([145.0, 615.0, -155.0, 90.0, 90.0, -90.0])

    holder_mode = {
        1: pen_holder,
        2: eraser_holder
    }
    
    def set_dependencies(
            self,

            movel, 

            get_current_posx,
        ):

        self.movel = movel
        self.get_current_posx = get_current_posx

    def move_to_home(self):
        self.movel(self.white_board_home, VEL, ACC)

    def move_to_start(self, traj):
        self.movel(traj[0], VEL, ACC)

    # def move_to_pen_holder(self):
    #     self.movel(self.pen_holder, VEL, ACC)

    # def down_to_pen_holder(self):
    #     self.pen_holder[1] += 8
    #     self.movel(self.pen_holder, VEL, ACC)
    #     self.pen_holder[1] += 2

    # def up_from_pen_holder(self):
    #     self.pen_holder[1] -= 10
    #     self.movel(self.pen_holder, VEL, ACC)

    def move_to_holder(self, mode):
        self.movel(self.holder_mode[mode], VEL, ACC)

    def move_to_above_holder(self, mode):
        self.holder_mode[mode][1] -= 30
        self.move_to_holder(mode)

    def down_to_hold(self, mode):
        self.holder_mode[mode][1] += 30
        self.move_to_holder(mode)

    def up_from_holder(self, mode):
        self.holder_mode[mode][1] -= 20
        self.move_to_holder(mode)
    
    def down_to_release(self, mode):
        self.holder_mode[mode][1] += 18
        self.move_to_holder(mode)
        self.holder_mode[mode][1] += 2

    def pen_down(self, callback):
        """로봇 펜을 보드(종이)로 누르는 동작을 수행합니다."""
        callback()

    def pen_up(self, callback):
        """로봇 펜을 보드(종이)에서 들어올립니다."""
        current_posx = self._get_cur_posx()[0]
        current_posx[2] -= 5
        callback()
        time.sleep(0.1)
        self.movel(current_posx, VEL, ACC)

    def _get_cur_posx(self):
        """
        현재 로봇의 작업 좌표계(Task Space Position, posx)를 읽어오는 함수.

        주요 동작:
            - 최대 5초 동안 반복해서 get_current_posx()를 호출하여 posx 정보를 시도한다.
            - IndexError가 발생하면 0.1초 후 재시도한다.
            - 5초 이내에 posx를 정상적으로 얻으면 해당 값을 반환한다.
            - 5초가 지나도 값을 얻지 못하면 오류 로그를 남기고, 모든 값이 0인 posx([0,0,0,0,0,0])를 반환한다.

        Returns:
            posx: 6차원 작업좌표(posx) 리스트 객체 또는 실패시 [0,0,0,0,0,0]
        """
        start = time.time()
        while time.time() - start < 5:
            try:
                cur_posx = self.get_current_posx()
                return cur_posx
            except IndexError as e:
                time.sleep(0.1)
                continue
        return posx([0,0,0,0,0,0])
    
class Forcer:
    def set_dependencies(
            self,

            check_force_condition,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force,

            DR_AXIS_Z,

            DR_FC_MOD_REL
        ):

        self.task_compliance_ctrl = task_compliance_ctrl
        self.set_desired_force = set_desired_force
        self.release_force = release_force
        self.release_compliance_ctrl = release_compliance_ctrl
        self.check_force_condition = check_force_condition
        self.DR_AXIS_Z = DR_AXIS_Z
        self.DR_FC_MOD_REL = DR_FC_MOD_REL

    def force_on(self, fd=[0, 0, 20, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0]):
        """로봇 펜/지우개을 보드에 누르는 동작을 수행합니다."""
        self.task_compliance_ctrl()
        time.sleep(0.1)
        self.set_desired_force(fd=fd, dir=dir, mod=self.DR_FC_MOD_REL)

    def force_off(self):
        """로봇의 힘/컴플라이언스(유연제어) 제어를 모두 해제합니다."""
        self.release_force()
        self.release_compliance_ctrl()

    def check_touch(self, mode, fd = [0, 0, 2, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0]):
        """Z축 힘(접촉력)이 5~21 사이가 될 때까지 대기"""
        if mode == 2: fd[2] = 5 # erase mode
        while self.check_force_condition(self.DR_AXIS_Z, min=5, max=21): pass
        self.release_force()
        self.set_desired_force(fd=fd, dir=dir, mod=self.DR_FC_MOD_REL)

# class DrawerAndEraser(Node):
class DrawerAndEraser:
    def __init__(self, node: Node):
        # super().__init__('drawer_and_eraser', namespace=ROBOT_ID)

        self.node = node

        self.gripper = Gripper()
        self.mover = Mover()
        self.forcer = Forcer()

        self.strokes_queue = queue.Queue()

        self.node.create_subscription(
            Float32MultiArray,
            DRAWING_PAHT,
            self.listener_callback,
            10
        )
        # self.create_timer(0.1, self.timer_callback)

    def set_dependencies(
            self, 
            
            set_digital_output,
            get_digital_input,
            wait,
            
            movel, amovesx,
            check_motion,
            
            check_force_condition,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force,

            get_current_posx,

            DR_AXIS_Z,

            DR_FC_MOD_REL
        ):

        self.amovesx = amovesx
        self.check_motion = check_motion

        self.gripper.set_dependencies(
            set_digital_output,
            get_digital_input,
            wait,
        )
        self.mover.set_dependencies(
            movel, 

            get_current_posx,
        )
        self.forcer.set_dependencies(
            check_force_condition,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force,

            DR_AXIS_Z,

            DR_FC_MOD_REL
        )

    def listener_callback(self, msg):
        data = msg.data
        if len(data) % 2 != 0:
            print('길이가 2의 배수가 아닌 데이터를 받았습니다!')
            return
        points = []
        for i in range(0, len(data), 2):
            points.append([data[i], data[i+1]])
        # print(f"[Subscriber] 복원된 좌표 리스트: {points}")
        self.strokes_queue.put(points)

    def timer_callback(self):
        if self.strokes_queue.empty():
            return  # strokes_queue에 토픽 없으면 종료
        
        print()
        print("---------------input mode---------------")
        print("1: pencil mode, 2: eraser mode, 3: exit")
        mode = int(input("select mode: "))

        self.mover.move_to_home()

        self.node.get_logger().info('Start to hold the pen')
        self.gripper.open_grip()                # 그리퍼 열고
        self.mover.move_to_above_holder(mode)   # 홀더 위로 가서
        self.mover.down_to_hold(mode)           # 내려가서
        self.gripper.close_grip()               # 타겟 쥐고
        self.mover.up_from_holder(mode)         # 살짝 위로 올라오기
        self.node.get_logger().info('Now holding the pen')

        self.mover.move_to_home()

        strokes = self.strokes_queue.get()
        splited_strokes = self.split_strokes(strokes)

        for stroke in splited_strokes:
            self.node.get_logger().info(f'stroke: {stroke}, {len(stroke)}')
            
            sampled_points  = self.sample_points(stroke)
            self.node.get_logger().info(f'sampled_points: {sampled_points}, {len(sampled_points)}')

            traj            = self.convert_to_posx(sampled_points)
            self.node.get_logger().info(f'traj: {traj}, {len(traj)}')

            self.node.get_logger().info('[move_to_start] start')
            self.mover.move_to_start(traj)
            self.node.get_logger().info('[move_to_start] done')

            self.node.get_logger().info('[pen_down] start')
            self.mover.pen_down(self.forcer.force_on)
            self.node.get_logger().info('[pen_down] done')
            
            self.node.get_logger().info('[check_touch] start')
            self.forcer.check_touch(mode)
            self.node.get_logger().info('[check_touch] done')

            self.draw_on_board(traj)
            self.check_done(traj)

            self.node.get_logger().info('[pen_up] start')
            self.mover.pen_up(self.forcer.force_off)
            self.node.get_logger().info('[pen_up] done')

        # self.mover.move_to_home()

        self.node.get_logger().info('Start to release the pen')
        self.mover.move_to_holder(mode)     # 홀더로 가서
        self.mover.down_to_release(mode)    # 살짝 내려가서
        self.gripper.open_grip()            # 그리퍼 풀기
        self.node.get_logger().info('Now releasing the pen')

        self.mover.move_to_home()
    
    def split_strokes(self, strokes):
        """
        연속된 점들(strokes)에서 음수 좌표([-1, -1] 등)가 등장할 때마다
        스트로크(획)를 분할하여 각 스트로크를 리스트로 반환합니다.

        Args:
            strokes (list 또는 numpy.ndarray): 
                - n x 2 형태의 [x, y] 좌표 리스트 또는 배열.
                - 음수 좌표([x<0 또는 y<0])는 새로운 스트로크의 구분점으로 간주합니다.

        Returns:
            list: 각 스트로크별 [x, y] 좌표 리스트로 이루어진 2중 리스트.
        """
        splited_strokes = []
        curr_stroke = []

        for pt in strokes:
            x, y = pt
            if x < 0 or y < 0:
                # 음수인 점은 새 stroke의 끝점
                if curr_stroke:
                    splited_strokes.append(curr_stroke)
                    curr_stroke = []
            else:
                curr_stroke.append([x, y])

        if curr_stroke:
            splited_strokes.append(curr_stroke)

        return splited_strokes
    
    def sample_points(self, points, max_middle=SAMPLE_THRESHOLD):
        """
        입력된 경로(points)에서 일부 점을 샘플링하여 경로를 단순화하는 함수.

        Args:
            points (list 또는 np.ndarray): 
                (N, 2) 형태의 [x, y] 점 리스트 또는 배열.
            max_middle (int): 
                중간 경유점 샘플 최대 개수 (기본값: SAMPLE_THRESHOLD).

        주요 동작:
            - 시작점과 끝점은 항상 포함합니다.
            - 중간 경유점은 전체의 10%만 균등 간격으로 샘플링하고, 최대 max_middle개를 넘지 않습니다.
            - 입력 점이 2개 이하이면 입력을 그대로 반환합니다.

        Returns:
            list: 샘플링된 [x, y] 좌표 리스트
        """
        n = len(points)
        if n <= 2:
            # 점이 2개 이하면 그대로 반환
            self.node.get_logger().info(f'[sample_points]: {points}')
            return points
        
        # 중간점 개수 계산
        middle_points = points[1:-1]
        num_middle = min(max_middle, max(1, int((n - 2) * SAMPLE_RATIO)))
        if num_middle < 1:
            # 중간 경유점이 아예 없거나 너무 적음
            self.node.get_logger().info(f'[sample_points]: {points}')
            return points
        
        idx = np.linspace(0, len(middle_points)-1, num_middle, dtype=int)
        sampled_middle = [middle_points[i] for i in idx]
        sampled_points = np.vstack([points[0], sampled_middle, points[-1]]).tolist()
        self.node.get_logger().info(f'[sample_points]: {sampled_points}')
        return sampled_points
    
    def convert_to_posx(self, sampled_points):
        """
        샘플링된 2차원 점 좌표들을 로봇의 posx 포맷(6D task space)으로 변환합니다.
        - Z축은 현재 로봇 위치의 z값을 사용합니다.
        - orientation은 [0, 0, 0]으로 고정됩니다.

        Args:
            sampled_points (list): 2D [x, y] 좌표 리스트

        Returns:
            list: posx 타입의 6차원 작업 좌표 리스트
        """
        z = self.mover._get_cur_posx()[0][2]
        return [posx([pt[0], pt[1], z, 0, 0, 0]) for pt in sampled_points]

    def draw_on_board(self, traj):
        """입력된 trajectory(traj)대로 비동기 방식(amovesx)으로 보드에 선을 그립니다"""
        ret = self.amovesx(traj, vel=VEL, acc=ACC)
        self.node.get_logger().info(f'[draw_on_board]: {ret}')

    def check_done(self, traj):
        """
        주어진 traj(궤적)이 모두 완료될 때까지 대기하며, 정상 종료 여부를 판단하는 함수.

        동작 방식:
            1. 예상 소요시간(estimate_draw_time) + 버퍼(buffer_time)만큼 반복적으로 모션 상태를 확인
            2. check_motion() == 0이면 그리기 성공으로 간주하여 True 반환
            3. 로봇의 posx(작업좌표)가 일정 횟수 이상(멈춤 상태 max_cnt 이상) 변화가 없으면 이상 동작으로 간주하여 False 반환
            4. 대기 시간(total_wait) 초과 시 타임아웃으로 간주하여 False 반환

        Args:
            traj (list): 로봇이 따라 그릴 trajectory(궤적) 데이터

        Returns:
            bool: 그리기 정상 종료 시 True, 이상 동작/타임아웃 시 False
        """

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

            base_time = length_mm / vel  # [mm] / [mm/s] = [s]
            return base_time * 1.5 if base_time > 20 else 30

        expected_time = estimate_draw_time(traj, VEL, ACC)
        buffer_time = 2
        total_wait = expected_time + buffer_time
        self.node.get_logger().info(f'경로길이: {traj_length(traj):.1f}mm, 예상시간: {expected_time:.2f}s (버퍼포함 {total_wait:.2f}s)')
        
        self.node.get_logger().info('[check_done] waiting until drawing is done')
        start = time.time()
        before_posx, cur_cnt, max_cnt = self.mover._get_cur_posx()[0], 0, 5
        while get_end(start) < total_wait:
            if self.check_motion() == 0:
                self.node.get_logger().info(f'[Drawing Success] done: {get_end(start):.2f}s')
                return True

            if before_posx == self.mover._get_cur_posx()[0]:
                cur_cnt += 1
            else:
                cur_cnt = 0

            if cur_cnt > max_cnt:
                self.node.get_logger().warn(f'[Drawing Failure] abnormal behavior')
                return False

            time.sleep(0.1)

        self.node.get_logger().warn(f'[Drawing Failure] time out: {get_end(start):.2f}s')
        return False

def main():
    rclpy.init()
    node = rclpy.create_node('drawer_and_eraser', namespace=ROBOT_ID)
    drawer_and_eraser = DrawerAndEraser(node)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (
        get_tcp, get_tool,
        set_tcp, set_tool, 
        set_ref_coord,

        set_digital_output,
        get_digital_input,
        wait,
        
        movel, amovesx,
        check_motion,
        
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        get_current_posx,

        DR_WHITE_BOARD2,
        DR_AXIS_Z,

        DR_FC_MOD_REL
    )

    drawer_and_eraser.set_dependencies(
        set_digital_output,
        get_digital_input,
        wait,

        movel, amovesx,
        check_motion,
        
        check_force_condition,
        task_compliance_ctrl,
        release_compliance_ctrl,
        set_desired_force,
        release_force,

        get_current_posx,

        DR_AXIS_Z,

        DR_FC_MOD_REL
    )

    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_ref_coord(DR_WHITE_BOARD2)

    tcp, tool = get_tcp(), get_tool()
    print(f'tcp: {tcp}, tool: {tool}')
    if tcp != ROBOT_TCP or tool != ROBOT_TOOL:
        drawer_and_eraser.node.destroy_node()
        return

    try:
        print('Spin start')
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            drawer_and_eraser.timer_callback()
    except KeyboardInterrupt:
        drawer_and_eraser.forcer.force_off()
        print('Shutting down...')
        pass        

    drawer_and_eraser.node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
