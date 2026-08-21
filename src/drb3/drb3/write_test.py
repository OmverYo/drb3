import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 이동 속도 및 가속도
VELOCITY = 30
ACC = 30

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

    # --- 펜 이동을 위한 헬퍼 함수 ---
    def move_rel(dx, dy, dz=0.0):
        # 현재 위치 기준 상대 이동
        movel(posx(dx, dy, dz, 0.0, 0.0, 0.0), vel=VELOCITY, acc=ACC, ref=DR_TOOL)

    def pen_up():
        move_rel(0.0, 0.0, -10.0) # 종이에서 떼기

    def pen_down():
        move_rel(0.0, 0.0, 10.0)  # 종이에 닿기

    def draw_letter(strokes, offset_x=0.0, offset_y=0.0):
        """2D 좌표로 정의된 획(strokes)을 그리고, 다음 글자를 위해 기준점을 이동합니다."""
        cur_x, cur_y = 0.0, 0.0
        
        for stroke in strokes:
            # 획의 시작점으로 이동
            start_x, start_y = stroke[0]
            pen_up()
            move_rel(start_x - cur_x, start_y - cur_y)
            pen_down()
            cur_x, cur_y = start_x, start_y
            
            # 획 긋기
            for x, y in stroke[1:]:
                move_rel(x - cur_x, y - cur_y)
                cur_x, cur_y = x, y
                
        # 글자 완성 후 다음 글자 시작 위치로 이동
        pen_up()
        move_rel(offset_x - cur_x, offset_y - cur_y)

    # --- '감사합니다' 획 디자인 (0~40mm 스케일) ---
    gam = [
        [(0,0), (15,0), (15,15)],                   # ㄱ
        [(25,0), (25,20)], [(25,10), (30,10)],      # ㅏ
        [(5,25), (20,25), (20,40), (5,40), (5,25)]  # ㅁ
    ]
    sa = [
        [(15,0), (5,20)], [(15,0), (25,20)],        # ㅅ
        [(35,0), (35,20)], [(35,10), (40,10)]       # ㅏ
    ]
    hab = [
        [(15,0), (15,5)], [(5,5), (25,5)],          # ㅎ 상단
        [(10,8), (20,8), (20,18), (10,18), (10,8)], # ㅎ 이응(사각형으로 대체)
        [(30,0), (30,20)], [(30,10), (35,10)],      # ㅏ
        [(5,22), (5,40)], [(25,22), (25,40)],       # ㅂ 세로
        [(5,30), (25,30)], [(5,40), (25,40)]        # ㅂ 가로
    ]
    ni = [
        [(5,0), (5,20), (25,20)],                   # ㄴ
        [(35,0), (35,25)]                           # ㅣ
    ]
    da = [
        [(20,0), (5,0), (5,20), (20,20)],           # ㄷ
        [(30,0), (30,20)], [(30,10), (35,10)]       # ㅏ
    ]

    try:
        initialize_robot()
        grip_open()
        node.get_logger().info(f"Moving to joint position: {Q1}")
        movej(Q1, vel=VELOCITY, acc=ACC)
        
        print("펜을 쥐어주세요 (5초 대기)")
        wait(5.0)
        grip_close()
        
        print("글쓰기 시작: 감사합니다")
        
        # 글자 간격 설정 (로봇 설정에 따라 X 또는 Y에 -50.0 등 부여)
        NEXT_LETTER_X = 0.0
        NEXT_LETTER_Y = -50.0 
        
        draw_letter(gam, NEXT_LETTER_X, NEXT_LETTER_Y)
        draw_letter(sa,  NEXT_LETTER_X, NEXT_LETTER_Y)
        draw_letter(hab, NEXT_LETTER_X, NEXT_LETTER_Y)
        draw_letter(ni,  NEXT_LETTER_X, NEXT_LETTER_Y)
        draw_letter(da,  NEXT_LETTER_X, NEXT_LETTER_Y)
        
        print("작업 완료")
        
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        movej(Q1, vel=VELOCITY, acc=ACC)
        grip_open()
        rclpy.shutdown()

if __name__ == "__main__":
    main()