import rclpy
from rclpy.node import Node
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1" #[cite: 5]

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 테스트용 점자 데이터 (1: 타각, 0: 통과)
# 각 배열이 점자 한 글자를 의미합니다. 원하시는 대로 수정하여 테스트하세요!
BRAILLE_DATA = [
    [1, 0, 0, 1, 0, 0], # 첫 번째 점자[cite: 6]
    [0, 1, 0, 0, 1, 0], # 두 번째 점자[cite: 6]
    [0, 0, 1, 0, 0, 1], # 세 번째 점자[cite: 6]
    [1, 0, 0, 0, 1, 0],
    [0, 0 ,0 ,0 ,0 ,0],
    [1, 0, 0, 0, 0, 1],
    [0, 1, 0 ,1 ,0 ,0],
    [0, 1, 0, 0, 0, 1]
]

def initialize_robot():
    """로봇의 Tool과 TCP 및 초기 모드를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, release_force, release_compliance_ctrl, set_robot_mode, ROBOT_MODE_AUTONOMOUS #[cite: 5]
    try:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS) #[cite: 5]
        release_force(time=0.0) #[cite: 5]
        release_compliance_ctrl() #[cite: 5]
    except Exception:
        pass
    
    set_tool(ROBOT_TOOL) #[cite: 5]
    set_tcp(ROBOT_TCP) #[cite: 5]
    print("로봇 Tool/TCP 초기화 완료") #[cite: 5]

def calculate_braille_start_position(braille_len, braille_offset=5.5):
    """종이의 중앙 좌표를 계산하여 점자 시작 위치를 반환합니다."""
    x_center = (281.0 + 566.0) / 2.0  # 종이 X 중심: 423.5[cite: 5]
    y_max = 97.0                      # 상단 기준 Y 좌표[cite: 5]
    
    braille_cell_width = 2.5 #[cite: 5]
    braille_width = (braille_len - 1) * braille_offset + braille_cell_width if braille_len > 0 else 0.0 #[cite: 5]
    braille_start_x = x_center - (braille_width / 2.0) #[cite: 5]
    
    # 글자가 없으므로 기본 상단 여백을 주고, 거기서 점자 위치(아래로 40mm)를 계산합니다.
    lowest_text_y = y_max - 20.0 #[cite: 5]
    braille_start_y = lowest_text_y - 20.0 #[cite: 5]
    
    return braille_start_x, braille_start_y

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("braille_test_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import (wait, movej, movel, DR_TOOL, DR_BASE, set_digital_output,
                            set_ref_coord, task_compliance_ctrl, set_desired_force, DR_FC_MOD_REL,
                            release_force, release_compliance_ctrl, check_force_condition, DR_AXIS_Z,
                            get_current_posx, DR_MV_MOD_REL) #[cite: 5, 6]
    from DR_common2 import posx, posj #[cite: 6]

    # 속도 및 가속도 설정[cite: 5]
    MOVEJ_VEL, MOVEJ_ACC = 200.0, 200.0 #[cite: 5]
    MOVE_VEL, MOVE_ACC = 100.0, 100.0 #[cite: 5]
    Z_VEL, Z_ACC = 20.0, 20.0 #[cite: 5]
    PUNCH_FORCE = 15.0 #[cite: 5]
    CHAR_OFFSET = 5.5 #[cite: 5]
    EPS = 1e-3 #[cite: 5]

    try:
        initialize_robot()
        
        Q1 = posj([0.0, 25.0, 55.0, 0.0, 100.0, 0.0]) #[cite: 5]
        
        # 1. 툴 픽업 좌표[cite: 5]
        pos_tool_above = posx([494.0, -183.5, 242.5, 0.0, 180.0, 0.0]) #[cite: 5]
        pos_tool_pick  = posx([494.0, -183.5, 95.0,  0.0, 180.0, 0.0]) #[cite: 5]
        pos_tool_drop  = posx([494.0, -183.5, 105.0, 0.0, 180.0, 0.0]) #[cite: 5]

        # 2. 이쑤시개 픽업하러 가기
        set_digital_output(1, 0); set_digital_output(2, 1) # 그리퍼 열기[cite: 5]
        node.get_logger().info("점자 툴(이쑤시개) 픽업 중...")
        
        movej(Q1, vel=MOVEJ_VEL, acc=MOVEJ_ACC) # 홈 위치 거침
        movel(pos_tool_above, vel=MOVEJ_VEL, acc=MOVEJ_ACC, ref=DR_BASE) #[cite: 5]
        movel(pos_tool_pick, vel=Z_VEL, acc=Z_ACC, ref=DR_BASE) #[cite: 5]
        
        set_digital_output(1, 1); set_digital_output(2, 0) # 그리퍼 닫기[cite: 5]
        wait(1.0) #[cite: 5]
        
        # 안전 높이로 복귀[cite: 5]
        movel(pos_tool_above, vel=MOVEJ_VEL, acc=MOVEJ_ACC, ref=DR_BASE) #[cite: 5]

        # 3. 점자 시작 좌표 계산 및 이동
        num_chars = len(BRAILLE_DATA)
        target_x, target_y = calculate_braille_start_position(num_chars, braille_offset=CHAR_OFFSET)
        
        # 안전 높이(242.5)를 유지하며 타각 위치로 이동[cite: 5]
        pos_braille_start = posx([target_x, target_y, 242.5, 0.0, 180.0, 0.0]) #[cite: 5]
        node.get_logger().info(f"타각 시작 위치로 이동 완료 (X: {target_x:.2f}, Y: {target_y:.2f})")
        movel(pos_braille_start, vel=MOVEJ_VEL, acc=MOVEJ_ACC, ref=DR_BASE) #[cite: 5]
        movel(posx([0.0, 0.0, 40.5, 0.0, 0.0, 0.0]), vel=Z_VEL, acc=Z_ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL)

        # ==========================================
        # 상대 이동 및 힘 제어 펀치 함수
        # ==========================================
        def move_rel(dx, dy, v=MOVE_VEL, a=MOVE_ACC):
            if abs(dx) < EPS and abs(dy) < EPS: return #[cite: 5]
            # 180도 회전을 위해 dx, dy 부호 반전[cite: 5]
            movel(posx([-dx, -dy, 0.0, 0.0, 0.0, 0.0]), vel=v, acc=a, ref=DR_TOOL, mod=DR_MV_MOD_REL) #[cite: 5]

        def punch_dot():
            safe_pos = get_current_posx(ref=DR_BASE)[0] #[cite: 5]
            try:
                # 확실한 타각을 위해 13mm 여유있게 하강
                movel(posx([0.0, 0.0, 10.0, 0.0, 0.0, 0.0]), vel=Z_VEL, acc=Z_ACC, ref=DR_TOOL)
                set_ref_coord(DR_TOOL) #[cite: 5]
                task_compliance_ctrl([3000.0, 3000.0, 1000.0, 200.0, 200.0, 200.0], time=0.2) #[cite: 5]
                
                # 상대 모드(mod=1) 적용하여 영점 오차 무시 및 힘 제어 시간 0.2초 적용
                set_desired_force([0.0, 0.0, PUNCH_FORCE, 0.0, 0.0, 0.0], dir=[0, 0, 1, 0, 0, 0], time=0.2, mod=1)
                
                if (check_force_condition(DR_AXIS_Z, min=1, ref=DR_TOOL)) == 0:
                    print("찍기 성공!")
                    release_force(time=0.2) #[cite: 5]
                    release_compliance_ctrl() #[cite: 5]
                    set_ref_coord(DR_BASE) #[cite: 5]
                    movel(safe_pos, vel=Z_VEL, acc=Z_ACC, ref=DR_BASE)
                else :
                    print("찍기 실패")
                    
                # 목표 힘에 도달하고 아주 짧게 뜸 들이기
                
            finally:
                 # 242.5 높이로 복귀[cite: 5]
                print("다음 점자")

        # ==========================================
        # 점자 출력 실행
        # ==========================================
        node.get_logger().info(f"점자 테스트 시작! (총 {num_chars}글자)")
        for i, bits in enumerate(BRAILLE_DATA):
            char_cur_x, char_cur_y = 0.0, 0.0 #[cite: 5]
            
            for j, val in enumerate(bits):
                if val == 1: #[cite: 5]
                    # 점자 내부 2.5mm 간격 계산[cite: 5]
                    target_x_char, target_y_char = (j // 3) * 2.5, (j % 3) * 2.5 #[cite: 5]
                    dx, dy = target_x_char - char_cur_x, target_y_char - char_cur_y #[cite: 5]
                    
                    move_rel(dx, dy) # 180도 회전 적용된 상대 이동[cite: 5]
                    punch_dot() # 힘 제어 타각[cite: 5]
                    char_cur_x, char_cur_y = target_x_char, target_y_char #[cite: 5]
                    
            if i != num_chars - 1:
                # 글자와 글자 사이의 간격(CHAR_OFFSET) 이동[cite: 5]
                move_rel(CHAR_OFFSET - char_cur_x, -char_cur_y) #[cite: 5]

        # 4. 완료 후 툴 반납[cite: 5]
        node.get_logger().info("점자 타각 완료, 툴 반납 중...")
        
        # 현재 위치에서 안전하게 242.5 높이로 Z축 상승 확인[cite: 5]
        curr_pos = get_current_posx(ref=DR_BASE)[0] #[cite: 5]
        curr_pos[2] = 242.5 #[cite: 5]
        movel(curr_pos, vel=Z_VEL, acc=Z_ACC, ref=DR_BASE) #[cite: 5]
        
        # 툴 반납 위치로 이동[cite: 5]
        movel(pos_tool_above, vel=MOVEJ_VEL, acc=MOVEJ_ACC, ref=DR_BASE) #[cite: 5]
        movel(pos_tool_drop, vel=Z_VEL, acc=Z_ACC, ref=DR_BASE) #[cite: 5]
        
        set_digital_output(1, 0); set_digital_output(2, 1) # 그리퍼 열기 (내려놓기)[cite: 5]
        wait(1.0) #[cite: 5]
        
        # 안전 높이 복귀 및 홈으로 이동[cite: 5]
        movel(pos_tool_above, vel=MOVEJ_VEL, acc=MOVEJ_ACC, ref=DR_BASE) #[cite: 5]
        movej(Q1, vel=MOVEJ_VEL, acc=MOVEJ_ACC) #[cite: 5]
        
        node.get_logger().info("✅ 점자 단독 테스트가 성공적으로 종료되었습니다.")

    except Exception as e:
        node.get_logger().error(f"테스트 중 에러 발생: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()