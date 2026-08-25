import rclpy
import threading
import DR_init

# 생성하신 커스텀 서비스 임포트 (패키지명에 맞게 수정)
# from custom_interfaces.srv import PrintBraille

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 평상시 이동 속도 및 가속도
VELOCITY = 100
ACC = 100

# 펜/핀 승강 속도 및 가속도
PEN_Z_VELOCITY = 20
PEN_Z_ACC = 20

# 상대 이동량 0거리 이동 방지
EPS = 1e-3

# 글자(점자 한 칸) 사이 간격: 첫 글자 시작점 (0,0) 기준 X축으로 10mm씩 이동
NEXT_CHAR_OFFSET_X = 10.0
NEXT_CHAR_OFFSET_Y = 0.0

# 출력할 점자 데이터 (각 원소 = 점자 한 글자, 6비트: [1,4,2,5,3,6] 순서 아님 주의,
# 기존 print_braille_character 로직 기준 col=i//3, row=i%3 순서를 그대로 따름)
# 필요할 때 이 리스트만 교체하면 됨
BRAILLE_DATA = [
    [1, 0, 0, 1, 0, 0],
    [0, 1, 0, 0, 1, 0],
    [0, 0, 1, 0, 0, 1],
]

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


def grip_open():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 0)
    set_digital_output(2, 1)
    wait(1.0)


def grip_close():
    from DSR_ROBOT2 import set_digital_output, wait
    set_digital_output(1, 1)
    set_digital_output(2, 0)
    wait(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("braille_printer", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (wait, movej, movel, DR_TOOL, DR_BASE,
                        set_ref_coord, task_compliance_ctrl, set_desired_force, DR_FC_MOD_REL,
                        release_force, release_compliance_ctrl, check_force_condition, DR_AXIS_Z, DR_QSTOP,
                        get_current_posx)
    from DR_common2 import posx, posj

    # 시작 기준점 (원하시는 점자 시작 위치로 변경하세요)
    Q1 = posj(0.0, 25.0, 55.0, 0.0, 100.0, 0.0)
    Q2 = posx(10, 5, 0, 0, 0 ,0)

    # --- 핀/툴 상태 관리 ---
    pen_state = "down"

    def move_tool(dx, dy, dz=0.0, vel=VELOCITY, acc=ACC):
        if abs(dx) < EPS and abs(dy) < EPS and abs(dz) < EPS:
            return
        movel(posx(dx, dy, dz, 0.0, 0.0, 0.0), vel=vel, acc=acc, ref=DR_TOOL)

    def pen_up():
        nonlocal pen_state
        if pen_state == "down":
            move_tool(0.0, 0.0, -31.5, vel=PEN_Z_VELOCITY, acc=PEN_Z_ACC)
            pen_state = "up"

    def pen_down():
        nonlocal pen_state
        if pen_state == "up":
            move_tool(0.0, 0.0, 31.5, vel=PEN_Z_VELOCITY, acc=PEN_Z_ACC)
            pen_state = "down"

    # ==============================================================
    # 📌 점자 타격용 힘/순응 제어 함수 (테스트 시 이 값들을 조절하세요)
    # ==============================================================
    def punch_dot(force, hold_time):

        safe_pos = get_current_posx(ref=DR_BASE)[0]
        try:
            # 1. 빠른 접근 (위치 제어)
            # 종이에서 너무 멀리서 힘 제어를 시작하면 닿기 전에 타임아웃이 발생합니다.
            # 종이 닿기 직전까지 위치 제어로 빠르게 내려갑니다.
            # (만약 초기 높이가 너무 높다면 이 17.0 값을 상황에 맞게 조금씩 늘려보세요)
            move_tool(0.0, 0.0, 30.5, vel=PEN_Z_VELOCITY, acc=PEN_Z_ACC)

            print("1. 툴 좌표계 설정 및 순응 제어 켜기")
            set_ref_coord(DR_TOOL)
            
            # 2. Z축 강성을 낮춤 (충격 흡수)
            stx = [3000.0, 3000.0, 1000.0, 200.0, 200.0, 200.0]
            task_compliance_ctrl(stx, time=0.2)

            print(f"2. Z축 방향으로 누르기 (목표 힘: {force}N)")
            fd = [0.0, 0.0, force, 0.0, 0.0, 0.0]
            fctrl_dir = [0, 0, 1, 0, 0, 0]
            set_desired_force(fd, dir=fctrl_dir, time=0.2, mod=0)

            print("3. 목표 힘에 도달할 때까지 대기")
            # 3. 힘 조건 확인 (timeout 3초 필수!)
            # 설정한 힘의 80%에 도달했는지 최대 3초간 기다립니다.
            # target_force = force * 0.1
            fcon = check_force_condition(DR_AXIS_Z, min=0.95, ref=DR_TOOL)

            if fcon == 0:
                print("성공: 점자 타격 완료")
                # wait(hold_time)
                # 다시 안전한 높이로 들어올림 (내려갔던 거리 17.0 만큼 원상복구)
                release_force(time=0.2)
                release_compliance_ctrl()
                set_ref_coord(DR_BASE)
                movel(safe_pos, vel=PEN_Z_VELOCITY, acc=PEN_Z_ACC, ref=DR_BASE)
            else:
                print("실패: 허공에서 타임아웃 됨 (시작 높이가 너무 높거나 힘 설정 오류)")
                

        finally:
            print("4. 힘 제어 해제 및 상승")
            
            
            # 다시 안전한 높이로 들어올림 (내려갔던 거리 17.0 만큼 원상복구)
            
    # ==============================================================
    # 점자 1글자 그리기 로직
    # ==============================================================
    def print_braille_character(bits, offset_x=0.0, offset_y=0.0, advance=True):
        """
        bits: 길이 6의 1차원 배열 (예: [1, 0, 0, 1, 1, 0])
        offset_x, offset_y: 다음 글자로 넘어가기 위한 이동량 (이 글자의 로컬 원점 기준)
        advance: True면 글자를 다 찍은 뒤 offset만큼 이동. 마지막 글자는 False로 호출해
                 불필요한 이동을 없앤다.
        """
        GAP_X = 5  # 한 글자 내의 점 간격 (가로)
        GAP_Y = 5  # 한 글자 내의 점 간격 (세로)

        char_cur_x = 0.0
        char_cur_y = 0.0

        for i, val in enumerate(bits):
            if val == 1:
                # 1~3번 점은 0열, 4~6번 점은 1열
                col = i // 3
                row = i % 3

                target_x = col * GAP_X
                target_y = row * GAP_Y

                dx = target_x - char_cur_x
                dy = target_y - char_cur_y

                # 다음 타격 위치로 이동 (펜은 이미 위로 들려 있음)
                move_tool(dx, dy, 0.0)

                # 힘 제어 함수를 호출하여 점 찍기
                punch_dot(force= 15 , hold_time = 0.5)

                char_cur_x = target_x
                char_cur_y = target_y

        # 글자 출력이 끝나면 다음 글자의 시작점으로 툴 이동
        if advance:
            move_tool(offset_x - char_cur_x, offset_y - char_cur_y, 0.0)
        # advance=False (마지막 글자)인 경우 불필요한 이동 없이 종료한다.

    is_printing = False

    # ==============================================================
    # 서비스 통신 콜백 (필요 시 재사용 — 현재는 비활성 상태, 아래 직접 실행 로직 사용)
    # ==============================================================
    '''def braille_service_callback(request, response):
        nonlocal is_printing
        if is_printing:
            response.success = False
            response.message = "현재 작업 중입니다. 새 명령을 무시합니다."
            return response

        data = request.data
        if len(data) % 6 != 0:
            response.success = False
            response.message = "수신 데이터 길이가 6의 배수가 아닙니다."
            return response

        node.get_logger().info(f"점자 비트 데이터 수신, 길이: {len(data)}")

        def task():
            nonlocal is_printing
            is_printing = True
            try:
                num_chars = len(data) // 6
                for char_idx in range(num_chars):
                    char_bits = data[char_idx*6 : (char_idx+1)*6]
                    is_last = (char_idx == num_chars - 1)
                    print_braille_character(
                        char_bits,
                        offset_x=NEXT_CHAR_OFFSET_X,
                        offset_y=NEXT_CHAR_OFFSET_Y,
                        advance=not is_last,
                    )

                node.get_logger().info("점자 출력 완료")
            except Exception as e:
                node.get_logger().error(f"출력 에러: {e}")
            finally:
                is_printing = False

        threading.Thread(target=task).start()

        response.success = True
        response.message = "점자 출력을 시작합니다."
        return response
        '''

    # 서비스 서버 등록 (현재 미사용)
    #srv = node.create_service(PrintBraille, 'print_braille_srv', braille_service_callback)

    try:
        initialize_robot()
        grip_open()
        node.get_logger().info(f"Moving to joint position: {Q1}")
        movej(Q1, vel=VELOCITY, acc=ACC)
        movel(Q2, vel=VELOCITY, acc=ACC, ref =DR_TOOL)
        print("점필을 쥐어주세요 (5초 대기)")
        wait(5.0)
        grip_close()

        # 점자 출력 전 핀을 종이에서 띄우고 시작 상태 대기
        move_tool(0.0, 0.0, 20.0, vel=PEN_Z_VELOCITY, acc=PEN_Z_ACC)
        pen_state = "up"

        # ==========================================================
        # 받은 점자 데이터를 순서대로 바로 출력
        # 각 글자 시작점은 첫 글자 원점 (0,0) 기준 X축으로 NEXT_CHAR_OFFSET_X씩 이동
        # ==========================================================
        print(f"점자 출력 시작: {len(BRAILLE_DATA)}글자")
        for i, bits in enumerate(BRAILLE_DATA):
            is_last = (i == len(BRAILLE_DATA) - 1)
            print_braille_character(
                bits,
                offset_x=NEXT_CHAR_OFFSET_X,
                offset_y=NEXT_CHAR_OFFSET_Y,
                advance=not is_last,
            )
        print("점자 출력 완료")

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"An unexpected error occurred: {e}")
    finally:
        pen_up()
        movej(Q1, vel=VELOCITY, acc=ACC)
        grip_open()
        rclpy.shutdown()


if __name__ == "__main__":
    main()