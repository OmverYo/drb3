import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 평상시 이동/그리기 속도 및 가속도
VELOCITY = 50
ACC = 50

# 펜을 종이에 대거나 뗄 때(Z축) 사용할 속도/가속도
# 접촉 충격을 줄이기 위해 평상시보다 느리게 설정
PEN_Z_VELOCITY = 20
PEN_Z_ACC = 20

# 상대 이동량이 이 값보다 작으면 이동 명령 자체를 보내지 않음 (0거리 이동 방지)
EPS = 1e-3

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
    node = rclpy.create_node("gripper", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import wait, movej, movel, DR_TOOL
    from DR_common2 import posx, posj

    Q1 = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)

    # --- 펜 이동 및 상태 관리 ---
    # "up"   : 펜이 종이에서 떨어져 있는 상태
    # "down" : 펜이 종이에 닿아 있는 상태
    #
    # 전제: Q1 자세 + 그립 완료 시점에 펜 끝이 이미 종이에 닿아있다고 가정한다.
    # (실제로 떠 있다면 Q1 자체의 높이를 조정할 것 — 코드로 보정할 부분이 아님)
    pen_state = "down"

    def move_rel(dx, dy, dz=0.0, vel=VELOCITY, acc=ACC):
        # 이동량이 거의 0이면 불필요한 이동 명령을 보내지 않는다.
        if abs(dx) < EPS and abs(dy) < EPS and abs(dz) < EPS:
            return
        movel(posx(dx, dy, dz, 0.0, 0.0, 0.0), vel=vel, acc=acc, ref=DR_TOOL)

    def pen_up():
        nonlocal pen_state
        if pen_state == "down":  # 펜이 내려가 있을 때만 위로 올림
            move_rel(0.0, 0.0, -10.0, vel=PEN_Z_VELOCITY, acc=PEN_Z_ACC)
            pen_state = "up"

    def pen_down():
        nonlocal pen_state
        if pen_state == "up":  # 펜이 들려 있을 때만 아래로 내림
            move_rel(0.0, 0.0, 10.0, vel=PEN_Z_VELOCITY, acc=PEN_Z_ACC)
            pen_state = "down"

    def draw_letter(strokes, offset_x=0.0, offset_y=0.0, advance=True):
        """
        strokes : 획 리스트
        offset_x, offset_y : 다음 글자 시작점까지의 이동량 (이 글자의 로컬 원점 기준)
        advance : True면 글자를 다 그린 뒤 offset만큼 이동. 마지막 글자는 False로 호출해
                  불필요한 복귀 이동을 없앤다.
        """
        cur_x, cur_y = 0.0, 0.0
        first_stroke = True

        for stroke in strokes:
            start_x, start_y = stroke[0]

            if first_stroke and pen_state == "down":
                # 글 전체의 첫 획: 펜이 이미 종이에 닿아있는 상태이므로
                # 승강(up/down) 동작 없이 바로 시작점으로 이동해 쓰기 시작한다.
                move_rel(start_x - cur_x, start_y - cur_y)
            else:
                pen_up()
                move_rel(start_x - cur_x, start_y - cur_y)
                pen_down()

            cur_x, cur_y = start_x, start_y
            first_stroke = False

            for x, y in stroke[1:]:
                move_rel(x - cur_x, y - cur_y)
                cur_x, cur_y = x, y

        pen_up()
        if advance:
            move_rel(offset_x - cur_x, offset_y - cur_y)
        # advance=False (마지막 글자)인 경우, 로컬 원점으로 복귀하는 불필요한
        # 이동을 하지 않고 펜만 든 채로 종료한다. 이후 movej(Q1)이 알아서 복귀시킨다.

    # --- '감사합니다' 획 디자인 (0~40mm 스케일) ---
    gam = [
        [(0, 0), (15, 0), (15, 15)],
        [(25, 0), (25, 20)], [(25, 10), (30, 10)],
        [(5, 25), (20, 25), (20, 40), (5, 40), (5, 25)]
    ]
    sa = [
        [(15, 0), (5, 20)], [(15, 0), (25, 20)],
        [(35, 0), (35, 20)], [(35, 10), (40, 10)]
    ]
    hab = [
        [(15, 0), (15, 5)], [(5, 5), (25, 5)],
        [(10, 8), (20, 8), (20, 18), (10, 18), (10, 8)],
        [(30, 0), (30, 20)], [(30, 10), (35, 10)],
        [(5, 22), (5, 40)], [(25, 22), (25, 40)],
        [(5, 30), (25, 30)], [(5, 40), (25, 40)]
    ]
    ni = [
        [(5, 0), (5, 20), (25, 20)],
        [(35, 0), (35, 25)]
    ]
    da = [
        [(20, 0), (5, 0), (5, 20), (20, 20)],
        [(30, 0), (30, 20)], [(30, 10), (35, 10)]
    ]

    letters = [gam, sa, hab, ni, da]

    try:
        initialize_robot()
        grip_open()
        node.get_logger().info(f"Moving to joint position: {Q1}")
        movej(Q1, vel=VELOCITY, acc=ACC)

        print("펜을 쥐어주세요 (5초 대기)")
        wait(5.0)
        grip_close()

        # 글쓰기 시작 시점: Q1 자세에서 펜이 이미 종이에 닿아있다고 가정 (위 설명 참고)
        pen_state = "down"

        print("글쓰기 시작: 감사합니다")

        # 글자 간격: 왼쪽 -> 오른쪽으로 쓰도록 X축 방향으로 이동
        # (Tool 좌표계 +X가 실제로 오른쪽이 맞는지 실기에서 꼭 확인할 것.
        #  반대 방향으로 써지면 부호를 뒤집을 것)
        NEXT_LETTER_X = 45.0
        NEXT_LETTER_Y = 0.0

        for i, letter in enumerate(letters):
            is_last = (i == len(letters) - 1)
            draw_letter(
                letter,
                offset_x=NEXT_LETTER_X,
                offset_y=NEXT_LETTER_Y,
                advance=not is_last,
            )

        print("작업 완료")

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"An unexpected error occurred: {e}")
    finally:
        pen_up()  # 마지막에 펜 확실히 들기
        movej(Q1, vel=VELOCITY, acc=ACC)
        grip_open()
        rclpy.shutdown()


if __name__ == "__main__":
    main()