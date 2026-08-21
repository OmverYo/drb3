import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30.0, 30.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class BraillePrinterNode(Node):
    def __init__(self):
        super().__init__('braille_printer_node')
        DR_init.__dsr__node = self

        try:
            # 사용할 로봇 제어 함수들을 전역으로 가져옵니다.
            global movej, movel, task_compliance_ctrl, set_desired_force
            global release_force, release_compliance_ctrl, wait, set_ref_coord
            global posx, posj, DR_TOOL, DR_BASE

            from DSR_ROBOT2 import (
                movej, movel, task_compliance_ctrl, set_desired_force, 
                release_force, release_compliance_ctrl, wait, set_ref_coord
            )
            from DR_common2 import posx, posj, DR_TOOL, DR_BASE
            
        except ImportError as e:
            self.get_logger().error(f"Error importing DSR_ROBOT2 : {e}")
            return

        # 2진 배열(예: [1, 0, 0, 1, 0, 0])을 받을 Subscriber 생성
        self.sub = self.create_subscription(
            Int32MultiArray,
            'braille_data',
            self.braille_callback,
            10
        )

        self.is_printing = False

        # ================== 점자 세팅 ==================
        self.GAP_X = 2.5 # 가로 점 간격 (mm)
        self.GAP_Y = 2.5 # 세로 점 간격 (mm)

        # 1번 점(좌측 상단) 위쪽 대기(어프로치) 위치 (사용 환경에 맞게 수정 필요)
        self.START_POS = posx(350.0, 0.0, 100.0, 0.0, 180.0, 0.0) 
        
        # 홈 위치
        self.HOME_POS = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
        # ===============================================

        self.get_logger().info("초기 위치로 이동합니다.")
        movej(self.HOME_POS, vel=VELOCITY, acc=ACC)
        self.get_logger().info("점자 출력 준비 완료. 데이터를 기다립니다.")

    def braille_callback(self, msg):
        if self.is_printing:
            self.get_logger().warn("출력 중입니다. 새 명령을 무시합니다.")
            return

        dots = msg.data
        if len(dots) != 6:
            self.get_logger().error("수신 데이터 길이가 6이 아닙니다. (2x3 배열 필요)")
            return

        self.get_logger().info(f"점자 데이터 수신: {list(dots)}")
        
        # 로봇 제어 함수들이 ROS2 콜백을 블로킹하지 않도록 별도 스레드에서 실행합니다.
        thread = threading.Thread(target=self.print_braille_task, args=(dots,))
        thread.start()

    def print_braille_task(self, dots):
        self.is_printing = True

        try:
            # 점자 배열: [1번, 2번, 3번, 4번, 5번, 6번]
            for i, val in enumerate(dots):
                if val == 1:
                    self.get_logger().info(f"-> {i+1}번째 점 찍기 시작")
                    
                    # 1~3번 점은 0열, 4~6번 점은 1열
                    col = i // 3 
                    row = i % 3
                    
                    # 기준 좌표로부터의 이동량 계산
                    dx = col * self.GAP_X
                    dy = row * self.GAP_Y # 방향에 따라 - 부호를 붙여야 할 수 있습니다.

                    target_x = self.START_POS[0] + dx
                    target_y = self.START_POS[1] + dy
                    target_z = self.START_POS[2]
                    rx, ry, rz = self.START_POS[3], self.START_POS[4], self.START_POS[5]
                    
                    point_approach = posx(target_x, target_y, target_z, rx, ry, rz)

                    # 1. 점의 바로 위(어프로치 위치)로 이동
                    movel(point_approach, vel=VELOCITY, acc=ACC, ref=DR_BASE)

                    # 2. 기준 좌표계를 툴(DR_TOOL)로 임시 변경 (힘 제어 방향을 위해)
                    set_ref_coord(DR_TOOL)

                    # 3. Z축 강성 낮추기 (순응 제어)
                    stx = [3000.0, 3000.0, 500.0, 200.0, 200.0, 200.0]
                    task_compliance_ctrl(stx, time=0.0)

                    # 4. 툴의 Z축 방향으로 5N 힘 가하기 (바닥으로 펜 누르기)
                    # (툴 Z축의 역방향으로 찔러야 한다면 -5.0 으로 변경)
                    fd = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
                    fctrl_dir = [0, 0, 1, 0, 0, 0]
                    set_desired_force(fd, dir=fctrl_dir)

                    # 5. 충분히 점이 찍히도록 0.5초 대기
                    wait(0.5)

                    # 6. 힘 제어 해제
                    release_force(time=0.0)
                    release_compliance_ctrl()

                    # 7. 좌표계를 다시 베이스(DR_BASE)로 복귀
                    set_ref_coord(DR_BASE)

                    # 8. 어프로치 위치로 다시 위로 빠져나오기
                    movel(point_approach, vel=VELOCITY, acc=ACC, ref=DR_BASE)

            self.get_logger().info("완료되었습니다.")
            
        except Exception as e:
            self.get_logger().error(f"출력 중 로봇 에러: {e}")
        
        finally:
            self.is_printing = False

def main(args=None):
    rclpy.init(args=args)
    node = BraillePrinterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("프로그램 정지")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()