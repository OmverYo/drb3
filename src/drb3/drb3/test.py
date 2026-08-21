import rclpy
import DR_init

# 로봇 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30.0, 30.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("sun_gear_assembly_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej, movel, set_digital_output, wait,
            task_compliance_ctrl, set_desired_force, amove_periodic,
            check_position_condition, stop, release_force, release_compliance_ctrl
        )
        from DR_common2 import (
            posx, posj, ON, OFF, DR_TOOL, DR_BASE, DR_AXIS_Z, DR_SSTOP
        )
    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2 : {e}")
        return

    # 그리퍼 제어 함수
    def grasp():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(1.0)

    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1.0)

    # 초기 위치 및 Z축 하강 오프셋
    Q1 = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
    down_z = 85.0
    pos_down = posx(0.0, 0.0, down_z, 0.0, 0.0, 0.0)

    # TODO: 실제 환경에 맞게 티칭된 Global 좌표값 입력 필요
    Global_p1 = posx(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) 
    Global_p2 = posx(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    Global_p3 = posx(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    Global_p4 = posx(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    Global_p5 = posx(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    Global_p6 = posx(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    Global_c1 = posx(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    Global_c2 = posx(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    try:
        node.get_logger().info("작업 시작")
        release()
        movej(Q1, vel=VELOCITY, acc=ACC)

        # 1 Cycle
        movel(Global_p1, vel=VELOCITY, acc=ACC)
        movel(pos_down, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        grasp()
        movel(Global_p1, vel=VELOCITY, acc=ACC)
        
        movel(Global_p4, vel=VELOCITY, acc=ACC)
        movel(pos_down, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        release()

        # 2 Cycle
        movel(Global_p4, vel=VELOCITY, acc=ACC)
        movel(Global_p2, vel=VELOCITY, acc=ACC)
        movel(pos_down, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        grasp()
        movel(Global_p2, vel=VELOCITY, acc=ACC)

        movel(Global_p5, vel=VELOCITY, acc=ACC)
        movel(pos_down, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        release()

        # 3 Cycle
        movel(Global_p5, vel=VELOCITY, acc=ACC)
        movel(Global_p3, vel=VELOCITY, acc=ACC)
        movel(pos_down, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        grasp()
        movel(Global_p3, vel=VELOCITY, acc=ACC)

        movel(Global_p6, vel=VELOCITY, acc=ACC)
        movel(pos_down, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        release()

        # 조립 준비 및 어프로치
        movel(Global_p6, vel=VELOCITY, acc=ACC)
        movel(Global_c1, vel=VELOCITY, acc=ACC)
        movel(pos_down, vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        grasp()
        movel(Global_c1, vel=VELOCITY, acc=ACC)

        movel(Global_c2, vel=VELOCITY, acc=ACC)

        # 순응-힘 제어 및 삽입 시작
        node.get_logger().info("순응 제어 및 삽입 모션 시작")
        stx = [500.0, 500.0, 500.0, 200.0, 200.0, 200.0]
        task_compliance_ctrl(stx, time=0.0)

        fd = [0.0, 0.0, -15.0, 0.0, 0.0, 0.0]
        fctrl_dir = [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir)

        amp = [0.0, 0.0, 0.0, 0.0, 0.0, 10.0]
        period = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        amove_periodic(amp, period, repeat=100, ref=DR_TOOL)

        # 삽입 감시 루프
        while True:
            pcon1 = check_position_condition(DR_AXIS_Z, max=70.0, ref=DR_BASE)
            if pcon1 == 1:
                stop(DR_SSTOP)
                node.get_logger().info("삽입 완료 감지")
                break
            wait(0.1)

        # 제어 해제
        release()
        release_force(time=0.0)
        release_compliance_ctrl()

    except KeyboardInterrupt:
        node.get_logger().info("프로그램 정지")
    except Exception as e:
        node.get_logger().error(f"로봇 에러 발생: {e}")
    finally:
        movej(Q1, vel=VELOCITY, acc=ACC)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
