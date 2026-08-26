import rclpy
from rclpy.node import Node
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "GripperDA_v1"
ROBOT_TOOL = "Tool Weight"
 

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

class StampTask:
    def __init__(self, move_vel=150.0, move_acc=150.0, z_vel=50.0, z_acc=50.0, press_force=50.0):
        self.MOVE_VEL = move_vel
        self.MOVE_ACC = move_acc
        self.Z_VEL = z_vel
        self.Z_ACC = z_acc
        self.PRESS_FORCE = press_force  # 5N (약한 힘으로 테스트)

    def execute(self, logger):
        from DSR_ROBOT2 import (movej, movel, set_digital_output, wait, 
                                set_ref_coord, task_compliance_ctrl, set_desired_force, 
                                release_force, release_compliance_ctrl, check_force_condition,
                                DR_TOOL, DR_BASE, DR_AXIS_Z, DR_MV_MOD_REL)
        from DR_common2 import posx, posj
        
        success = False
        try:
            Q_HOME = posj([0.0, 25.0, 55.0, 0.0, 100.0, 0.0])
            
            # 도장 위치 (인주 묻히는 곳)
            pos_ink_above = posx([640.0, -2.0, 200.0, 0.0, 180.0, 0.0])
            pos_ink       = posx([640.0, -2.0, 157.0, 0.0, 180.0, 0.0])
            
            # 도장 찍는 위치 (종이)
            pos_stamp_above = posx([516.0, -43.0, 200.0, 90.0, 180.0, 0.0])
            pos_stamp       = posx([516.0, -43.0, 137.0, 90.0, 180.0, 0.0])

            # 1. 도장 위치로 이동 및 집기
            logger.info("도장 픽업 및 인주 묻히기 위치로 이동")
            set_digital_output(1, 0); set_digital_output(2, 1) # 그리퍼 열기
            
            movel(pos_ink_above, vel=self.MOVE_VEL, acc=self.MOVE_ACC, ref=DR_BASE)
            movel(pos_ink, vel=self.Z_VEL, acc=self.Z_ACC, ref=DR_BASE)
            
            set_digital_output(1, 1); set_digital_output(2, 0) # 그리퍼 닫기 (집기)
            wait(1.0)
            
            # 2. 힘 제어로 인주 묻히기
            logger.info("힘 제어로 인주 묻히기 시작")
            set_ref_coord(DR_TOOL)
            task_compliance_ctrl([3000.0, 3000.0, 1000.0, 200.0, 200.0, 200.0], time=0.2)
            set_desired_force([0.0, 0.0, self.PRESS_FORCE, 0.0, 0.0, 0.0], dir=[0, 0, 1, 0, 0, 0], time=0.2, mod=0)
            wait(2.0) # 인주가 충분히 묻도록 대기
            release_force(time=0.2)
            release_compliance_ctrl()
            set_ref_coord(DR_BASE)

            # 3. 안전 높이로 상승 및 찍을 위치로 이동
            movel(pos_ink_above, vel=self.Z_VEL, acc=self.Z_ACC, ref=DR_BASE)
            logger.info("도장 찍을 위치로 이동")
            movel(pos_stamp_above, vel=self.MOVE_VEL, acc=self.MOVE_ACC, ref=DR_BASE)
            movel(pos_stamp, vel=self.Z_VEL, acc=self.Z_ACC, ref=DR_BASE)

            # 4. 힘 제어로 도장 찍기
            logger.info("힘 제어로 도장 찍기 시작")
            set_ref_coord(DR_TOOL)
            task_compliance_ctrl([3000.0, 3000.0, 1000.0, 200.0, 200.0, 200.0], time=0.2)
            set_desired_force([0.0, 0.0, self.PRESS_FORCE, 0.0, 0.0, 0.0], dir=[0, 0, 1, 0, 0, 0], time=0.2, mod=0)
            wait(2.0) # 도장이 선명하게 찍히도록 대기
            release_force(time=0.2)
            release_compliance_ctrl()
            set_ref_coord(DR_BASE)

            # 5. 안전 높이로 상승 후 원위치
            movel(pos_stamp_above, vel=self.Z_VEL, acc=self.Z_ACC, ref=DR_BASE)
            movel(pos_ink_above, vel=self.MOVE_VEL, acc=self.MOVE_ACC, ref=DR_BASE)
            movel(pos_ink, vel=self.Z_VEL, acc=self.Z_ACC, ref=DR_BASE)
            set_digital_output(1, 0); set_digital_output(2, 1)
            wait(1.0)
            movel(pos_ink_above, vel=self.Z_VEL, acc=self.Z_ACC, ref=DR_BASE)
            movej(Q_HOME, vel=self.MOVE_VEL, acc=self.MOVE_ACC)
            
            logger.info("도장 찍기 테스트 완료")
            success = True

        except Exception as e:
            logger.error(f"도장 찍기 에러: {e}")
            
        return success

def initialize_robot():
    """로봇의 Tool과 TCP 및 초기 모드를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, release_force, release_compliance_ctrl, set_robot_mode, ROBOT_MODE_AUTONOMOUS 
    
    set_tool(ROBOT_TOOL) 
    set_tcp(ROBOT_TCP) 
    print("로봇 Tool/TCP 초기화 완료") 

def main(args=None):
    rclpy.init(args=args)
    # 네임스페이스 추가 완료
    node = Node('stamp_test_node', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    
    initialize_robot()
    stamper = StampTask()
    stamper.execute(node.get_logger())
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()