import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Bool
import DR_init
import time

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==========================================
# [0] 하드웨어 전역 초기화 함수
# ==========================================
def initialize_robot():
    """로봇의 Tool과 TCP 및 초기 모드를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, release_force, release_compliance_ctrl, set_robot_mode, ROBOT_MODE_AUTONOMOUS
    try:
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        release_force(time=0.0)
        release_compliance_ctrl()
    except Exception:
        pass
    
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    print("로봇 Tool/TCP 초기화 완료")


# ==========================================
# [1] 한글 동적 렌더링 엔진 (동일 유지)
# ==========================================
class HangulEngine:
    # 초성, 중성, 종성 유니코드 리스트
    CHO = ['ㄱ','ㄲ','ㄴ','ㄷ','ㄸ','ㄹ','ㅁ','ㅂ','ㅃ','ㅅ','ㅆ','ㅇ','ㅈ','ㅉ','ㅊ','ㅋ','ㅌ','ㅍ','ㅎ']
    JUNG = ['ㅏ','ㅐ','ㅑ','ㅒ','ㅓ','ㅔ','ㅕ','ㅖ','ㅗ','ㅘ','ㅙ','ㅚ','ㅛ','ㅜ','ㅝ','ㅞ','ㅟ','ㅠ','ㅡ','ㅢ','ㅣ']
    JONG = ['','ㄱ','ㄲ','ㄳ','ㄴ','ㄵ','ㄶ','ㄷ','ㄹ','ㄺ','ㄻ','ㄼ','ㄽ','ㄾ','ㄿ','ㅀ','ㅁ','ㅂ','ㅄ','ㅅ','ㅆ','ㅇ','ㅈ','ㅊ','ㅋ','ㅌ','ㅍ','ㅎ']

    # 모음 형태 분류 (V: 세로형, H: 가로형, M: 복합형)
    V_MOEUM = [0, 1, 2, 3, 4, 5, 6, 7, 20]  # ㅏ, ㅐ, ㅑ, ㅓ, ㅣ 등
    H_MOEUM = [8, 12, 13, 17, 18]           # ㅗ, ㅛ, ㅜ, ㅠ, ㅡ
    M_MOEUM = [9, 10, 11, 14, 15, 16, 19]   # ㅘ, ㅝ, ㅢ 등

    # 0~1 사이로 정규화된 자모 기본 획 (디자인 딕셔너리)
    # *여기에 없는 겹자음/겹모음은 필요시 추가하시면 됩니다.*
    JAMO_STROKES = {
        # 기본 자음
        'ㄱ': [[(0.1, 0.1), (0.9, 0.1), (0.9, 0.9)]],
        'ㄴ': [[(0.1, 0.1), (0.1, 0.9), (0.9, 0.9)]],
        'ㄷ': [[(0.1, 0.1), (0.9, 0.1)], [(0.1, 0.1), (0.1, 0.9), (0.9, 0.9)]],
        'ㄹ': [[(0.1, 0.1), (0.9, 0.1), (0.9, 0.5), (0.1, 0.5), (0.1, 0.9), (0.9, 0.9)]],
        'ㅁ': [[(0.1, 0.1), (0.1, 0.9)], [(0.1, 0.1), (0.9, 0.1), (0.9, 0.9), (0.1, 0.9)]],
        'ㅂ': [[(0.2, 0.1), (0.2, 0.9)], [(0.8, 0.1), (0.8, 0.9)], [(0.2, 0.5), (0.8, 0.5)], [(0.2, 0.9), (0.8, 0.9)]],
        'ㅅ': [[(0.5, 0.1), (0.1, 0.9)], [(0.5, 0.5), (0.9, 0.9)]],
        'ㅇ': [[(0.5, 0.1), (0.2, 0.3), (0.2, 0.7), (0.5, 0.9), (0.8, 0.7), (0.8, 0.3), (0.5, 0.1)]], # 팔각형 근사
        'ㅈ': [[(0.1, 0.1), (0.9, 0.1)], [(0.5, 0.1), (0.1, 0.9)], [(0.5, 0.5), (0.9, 0.9)]],
        'ㅊ': [[(0.4, 0.0), (0.6, 0.0)], [(0.1, 0.2), (0.9, 0.2)], [(0.5, 0.2), (0.1, 0.9)], [(0.5, 0.5), (0.9, 0.9)]],
        'ㅋ': [[(0.1, 0.1), (0.9, 0.1), (0.9, 0.9)], [(0.1, 0.5), (0.9, 0.5)]],
        'ㅌ': [[(0.1, 0.1), (0.9, 0.1)], [(0.1, 0.5), (0.9, 0.5)], [(0.1, 0.1), (0.1, 0.9), (0.9, 0.9)]],
        'ㅍ': [[(0.1, 0.1), (0.9, 0.1)], [(0.3, 0.1), (0.3, 0.9)], [(0.7, 0.1), (0.7, 0.9)], [(0.1, 0.9), (0.9, 0.9)]],
        'ㅎ': [[(0.4, 0.0), (0.6, 0.0)], [(0.2, 0.2), (0.8, 0.2)], [(0.5, 0.3), (0.3, 0.5), (0.5, 0.7), (0.7, 0.5), (0.5, 0.3)]],

        # 쌍자음 (초성/종성 공통) - 왼쪽(0.0~0.4)과 오른쪽(0.5~0.9)으로 분할 배치
        'ㄲ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.9)], 
              [(0.5, 0.1), (0.9, 0.1), (0.9, 0.9)]],
        'ㄸ': [[(0.0, 0.1), (0.4, 0.1)], [(0.0, 0.1), (0.0, 0.9), (0.4, 0.9)],
              [(0.5, 0.1), (0.9, 0.1)], [(0.5, 0.1), (0.5, 0.9), (0.9, 0.9)]],
        'ㅃ': [[(0.1, 0.1), (0.1, 0.9)], [(0.4, 0.1), (0.4, 0.9)], [(0.1, 0.5), (0.4, 0.5)], [(0.1, 0.9), (0.4, 0.9)],
              [(0.6, 0.1), (0.6, 0.9)], [(0.9, 0.1), (0.9, 0.9)], [(0.6, 0.5), (0.9, 0.5)], [(0.6, 0.9), (0.9, 0.9)]],
        'ㅆ': [[(0.2, 0.1), (0.0, 0.9)], [(0.2, 0.5), (0.4, 0.9)],
              [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]],
        'ㅉ': [[(0.0, 0.1), (0.4, 0.1)], [(0.2, 0.1), (0.0, 0.9)], [(0.2, 0.5), (0.4, 0.9)],
              [(0.5, 0.1), (0.9, 0.1)], [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]],

        # 겹받침 (종성 전용) - 왼쪽/오른쪽 자음을 작게 구성
        'ㄳ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.9)],               # ㄱ
              [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]], # ㅅ
        'ㄵ': [[(0.0, 0.1), (0.0, 0.9), (0.4, 0.9)],               # ㄴ
              [(0.5, 0.1), (0.9, 0.1)], [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]], # ㅈ
        'ㄶ': [[(0.0, 0.1), (0.0, 0.9), (0.4, 0.9)],               # ㄴ
              [(0.6, 0.0), (0.8, 0.0)], [(0.5, 0.2), (0.9, 0.2)], [(0.7, 0.4), (0.5, 0.6), (0.7, 0.8), (0.9, 0.6), (0.7, 0.4)]], # ㅎ
        'ㄺ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], # ㄹ
              [(0.5, 0.1), (0.9, 0.1), (0.9, 0.9)]],               # ㄱ
        'ㄻ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], # ㄹ
              [(0.5, 0.1), (0.5, 0.9)], [(0.5, 0.1), (0.9, 0.1), (0.9, 0.9), (0.5, 0.9)]], # ㅁ
        'ㄼ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], # ㄹ
              [(0.6, 0.1), (0.6, 0.9)], [(0.9, 0.1), (0.9, 0.9)], [(0.6, 0.5), (0.9, 0.5)], [(0.6, 0.9), (0.9, 0.9)]], # ㅂ
        'ㄽ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], # ㄹ
              [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]], # ㅅ
        'ㄾ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], # ㄹ
              [(0.5, 0.1), (0.9, 0.1)], [(0.5, 0.5), (0.9, 0.5)], [(0.5, 0.1), (0.5, 0.9), (0.9, 0.9)]], # ㅌ
        'ㄿ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], # ㄹ
              [(0.5, 0.1), (0.9, 0.1)], [(0.6, 0.1), (0.6, 0.9)], [(0.8, 0.1), (0.8, 0.9)], [(0.5, 0.9), (0.9, 0.9)]], # ㅍ
        'ㅀ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], # ㄹ
              [(0.6, 0.0), (0.8, 0.0)], [(0.5, 0.2), (0.9, 0.2)], [(0.7, 0.4), (0.5, 0.6), (0.7, 0.8), (0.9, 0.6), (0.7, 0.4)]], # ㅎ
        'ㅄ': [[(0.0, 0.1), (0.0, 0.9)], [(0.4, 0.1), (0.4, 0.9)], [(0.0, 0.5), (0.4, 0.5)], [(0.0, 0.9), (0.4, 0.9)], # ㅂ
              [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]], # ㅅ
        
        # 기본 모음
        'ㅏ': [[(0.3, 0.1), (0.3, 0.9)], [(0.3, 0.5), (0.8, 0.5)]],
        'ㅑ': [[(0.3, 0.1), (0.3, 0.9)], [(0.3, 0.4), (0.8, 0.4)], [(0.3, 0.6), (0.8, 0.6)]],
        'ㅓ': [[(0.8, 0.1), (0.8, 0.9)], [(0.3, 0.5), (0.8, 0.5)]],
        'ㅕ': [[(0.8, 0.1), (0.8, 0.9)], [(0.3, 0.4), (0.8, 0.4)], [(0.3, 0.6), (0.8, 0.6)]],
        'ㅗ': [[(0.1, 0.7), (0.9, 0.7)], [(0.5, 0.2), (0.5, 0.7)]],
        'ㅛ': [[(0.1, 0.7), (0.9, 0.7)], [(0.3, 0.2), (0.3, 0.7)], [(0.7, 0.2), (0.7, 0.7)]],
        'ㅜ': [[(0.1, 0.3), (0.9, 0.3)], [(0.5, 0.3), (0.5, 0.8)]],
        'ㅠ': [[(0.1, 0.3), (0.9, 0.3)], [(0.3, 0.3), (0.3, 0.8)], [(0.7, 0.3), (0.7, 0.8)]],
        'ㅡ': [[(0.1, 0.5), (0.9, 0.5)]],
        'ㅣ': [[(0.5, 0.1), (0.5, 0.9)]],

        # 겹모음
        'ㅐ': [[(0.3, 0.1), (0.3, 0.9)], [(0.3, 0.5), (0.8, 0.5)], [(0.8, 0.1), (0.8, 0.9)]],
        'ㅔ': [[(0.1, 0.5), (0.4, 0.5)], [(0.4, 0.1), (0.4, 0.9)], [(0.8, 0.1), (0.8, 0.9)]],
        'ㅒ': [[(0.3, 0.1), (0.3, 0.9)], [(0.3, 0.4), (0.8, 0.4)], [(0.3, 0.6), (0.8, 0.6)], [(0.8, 0.1), (0.8, 0.9)]],
        'ㅖ': [[(0.1, 0.4), (0.4, 0.4)], [(0.1, 0.6), (0.4, 0.6)], [(0.4, 0.1), (0.4, 0.9)], [(0.8, 0.1), (0.8, 0.9)]],

        # 복합 모음 (초성 자리를 피해 좌측 하단과 우측에 배치)
        'ㅘ': [[(0.3, 0.5), (0.3, 0.8)], [(0.1, 0.8), (0.8, 0.8)], [(0.8, 0.1), (0.8, 0.9)], [(0.8, 0.5), (0.9, 0.5)]],
        'ㅙ': [[(0.3, 0.5), (0.3, 0.8)], [(0.1, 0.8), (0.7, 0.8)], [(0.7, 0.1), (0.7, 0.9)], [(0.7, 0.5), (0.9, 0.5)], [(0.9, 0.1), (0.9, 0.9)]],
        'ㅚ': [[(0.4, 0.5), (0.4, 0.8)], [(0.1, 0.8), (0.8, 0.8)], [(0.8, 0.1), (0.8, 0.9)]],
        'ㅝ': [[(0.1, 0.6), (0.7, 0.6)], [(0.4, 0.6), (0.4, 0.9)], [(0.7, 0.5), (0.9, 0.5)], [(0.9, 0.1), (0.9, 0.9)]],
        'ㅞ': [[(0.1, 0.6), (0.6, 0.6)], [(0.3, 0.6), (0.3, 0.9)], [(0.6, 0.4), (0.75, 0.4)], [(0.75, 0.1), (0.75, 0.9)], [(0.9, 0.1), (0.9, 0.9)]],
        'ㅟ': [[(0.1, 0.6), (0.7, 0.6)], [(0.4, 0.6), (0.4, 0.9)], [(0.8, 0.1), (0.8, 0.9)]],
        'ㅢ': [[(0.1, 0.7), (0.7, 0.7)], [(0.8, 0.1), (0.8, 0.9)]],
    }

    @staticmethod
    def scale_strokes(char_jamo, bx, by, bw, bh):
        """0~1 기준 좌표를 Bounding Box에 맞춰 크기/위치 변환"""
        if char_jamo not in HangulEngine.JAMO_STROKES:
            return []
        
        strokes = HangulEngine.JAMO_STROKES[char_jamo]
        transformed = []
        for stroke in strokes:
            new_stroke = []
            for (x, y) in stroke:
                real_x = bx + (x * bw)
                real_y = by + (y * bh)
                new_stroke.append((real_x, real_y))
            transformed.append(new_stroke)
        return transformed

    @classmethod
    def get_char_strokes(cls, char, box_width=40.0, box_height=40.0):
        """한 글자를 받아 해당 글자의 로봇 궤적(획 리스트)을 반환"""
        # 띄어쓰기 처리
        if char == " ": return []
        
        code = ord(char)
        # 한글이 아닌 경우 빈 배열 반환 (필요시 영어/기호 추가 가능)
        if code < 0xAC00 or code > 0xD7A3:
            return []

        # 한글 유니코드 분리
        offset = code - 0xAC00
        jong_idx = offset % 28
        jung_idx = (offset // 28) % 21
        cho_idx = (offset // 28) // 21

        cho_char = cls.CHO[cho_idx]
        jung_char = cls.JUNG[jung_idx]
        jong_char = cls.JONG[jong_idx]

        strokes = []
        has_jong = jong_idx != 0

        # 종성 유무에 따른 상/하단 높이 배분
        top_h = box_height * 0.6 if has_jong else box_height
        bot_h = box_height * 0.4 if has_jong else 0
        bot_y = box_height * 0.6

        # 모음 형태에 따른 초/중성 레이아웃 분할
        if jung_idx in cls.V_MOEUM:   # 세로형 (가, 강)
            strokes.extend(cls.scale_strokes(cho_char, 0, 0, box_width * 0.5, top_h))
            strokes.extend(cls.scale_strokes(jung_char, box_width * 0.5, 0, box_width * 0.5, top_h))
        elif jung_idx in cls.H_MOEUM: # 가로형 (고, 공)
            strokes.extend(cls.scale_strokes(cho_char, 0, 0, box_width, top_h * 0.5))
            strokes.extend(cls.scale_strokes(jung_char, 0, top_h * 0.5, box_width, top_h * 0.5))
        else:                         # 복합형 (과, 광)
            strokes.extend(cls.scale_strokes(cho_char, 0, 0, box_width * 0.5, top_h * 0.5))
            strokes.extend(cls.scale_strokes(jung_char, 0, 0, box_width, top_h))

        # 종성 렌더링
        if has_jong:
            strokes.extend(cls.scale_strokes(jong_char, 0, bot_y, box_width, bot_h))

        return strokes

# ==========================================
# [2] 글쓰기 작업 클래스
# ==========================================
class WriteTask:
    def __init__(self, movej_vel=200.0, movej_acc=200.0, draw_vel=50.0, draw_acc=50.0, z_vel=100.0, z_acc=100.0, letter_size=20.0, letter_space=10.0):
        self.MOVEJ_VEL = movej_vel
        self.MOVEJ_ACC = movej_acc
        self.DRAW_VEL = draw_vel
        self.DRAW_ACC = draw_acc
        self.Z_VEL = z_vel
        self.Z_ACC = z_acc
        self.LETTER_SIZE = letter_size
        self.LETTER_SPACE = letter_space
        self.EPS = 1e-3
        self.pen_state = "down"

    def execute(self, text, logger):
        from DSR_ROBOT2 import movej, movel, set_digital_output, wait, DR_TOOL, DR_MV_MOD_REL
        from DR_common2 import posx, posj
        
        success = False
        try:
            def move_rel(dx, dy, dz=0.0, v=self.DRAW_VEL, a=self.DRAW_ACC):
                if abs(dx) < self.EPS and abs(dy) < self.EPS and abs(dz) < self.EPS: return
                movel(posx([dx, dy, dz, 0.0, 0.0, 0.0]), vel=v, acc=a, ref=DR_TOOL)
                        
            def pen_up():
                if self.pen_state == "down": 
                    move_rel(0.0, 0.0, -10.0, v=self.Z_VEL, a=self.Z_ACC)
                    self.pen_state = "up"
                                
            def pen_down():
                if self.pen_state == "up": 
                    move_rel(0.0, 0.0, 10.0, v=self.Z_VEL, a=self.Z_ACC)
                    self.pen_state = "down"
            
            Q1 = posj([0.0, 25.0, 60.0, 0.0, 94.5, 0.0]) 
            '''
            W1 = posx([0, 0, 0, 0, 0, 0]) # 글쓰기 시작위치 조정 필요!!!!
            P1 = posx([0, 0, 0, 0, 0, 0]) # 팬위치 위쪽
            P_z = 0 # 팬잡으로 내려오기까지 필요한 거리

            '''
            set_digital_output(1, 0); set_digital_output(2, 1) # 오픈
            
            logger.info("글쓰기 초기 위치로 이동 중...")
            movej(Q1, vel=self.MOVEJ_VEL, acc=self.MOVEJ_ACC)
            '''
            movel(P1, vel=self.MOVEJ_VEL, acc=self.MOVEJ_ACC)
            move_rel(0.0, 0.0, P_z, v=self.Z_VEL, a=self.Z_ACC)

            '''
            print("펜 집 기")
            
            set_digital_output(1, 1); set_digital_output(2, 0) # 클로즈
            wait(1.0)
            
            self.pen_state = "down"
            '''
            move_rel(0.0, 0.0, -P_z, v=self.Z_VEL, a=self.Z_ACC)
            movej(Q1, vel=self.MOVEJ_VEL, acc=self.MOVEJ_ACC)

            '''
            
            logger.info("글쓰기 타각 시작!")
            engine = HangulEngine()
            
            for i, char in enumerate(text):
                if char == " ":
                    move_rel(self.LETTER_SPACE, 0.0, v=self.Z_VEL, a=self.Z_ACC)
                    continue
                    
                strokes = engine.get_char_strokes(char, box_width=self.LETTER_SIZE, box_height=self.LETTER_SIZE)
                cur_x, cur_y = 0.0, 0.0
                first = True
                
                for stroke in strokes:
                    sx, sy = stroke[0]
                    if first and self.pen_state == "down": 
                        move_rel(sx - cur_x, sy - cur_y)
                    else: 
                        pen_up()
                        move_rel(sx - cur_x, sy - cur_y, v=self.Z_VEL, a=self.Z_ACC) 
                        pen_down()
                    
                    cur_x, cur_y = sx, sy
                    first = False
                    for x, y in stroke[1:]:
                        move_rel(x - cur_x, y - cur_y)
                        cur_x, cur_y = x, y
                        
                pen_up()
                if i != len(text) - 1: 
                    move_rel(self.LETTER_SPACE - cur_x, -cur_y, v=self.Z_VEL, a=self.Z_ACC)

            success = True
            
            # 종료 후 뱉기
            '''
            movel(W1, vel=self.MOVEJ_VEL, acc=self.MOVEJ_ACC)
            move_rel

            '''

            movej(Q1, vel=self.MOVEJ_VEL, acc=self.MOVEJ_ACC)
            set_digital_output(1, 0); set_digital_output(2, 1)
            
        except Exception as e:
            logger.error(f"글쓰기 중 에러: {e}")
            
        return success


# ==========================================
# [3] 종이 뒤집기 작업 클래스 
# ==========================================
class FlipTask:
    def __init__(self, movej_vel=150.0, movej_acc=150.0, movel_vel=100.0, movel_acc=100.0, slow_vel=50.0, slow_acc=50.0):
        self.MOVEJ_VEL = movej_vel; self.MOVEJ_ACC = movej_acc
        self.MOVEL_VEL = movel_vel; self.MOVEL_ACC = movel_acc
        self.SLOW_VEL = slow_vel; self.SLOW_ACC = slow_acc

    def execute(self, logger):
        from DSR_ROBOT2 import movej, movel, set_digital_output, wait
        from DR_common2 import posx, posj
        
        success = False
        try:
            Q1 = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
            
            # [진입 및 빠져나오는 'ㄷ'자 경로 좌표]
            p1 = posx([422.25, 230.0,   200.0,  164.4, 179.89, 164.24])
            p2 = posx([422.25, 230.0,   99.1,   91.0, -91.0,   -0.1])
            p3 = posx([422.25, 145.53,  99.15,  91.1, -91.0,   -0.1])
            
            # [들고 뒤집고 내리는 위치 좌표]
            pos_lift = posx([422.25, 145.53, 300.0, 91.1, -91.0, -0.1])
            pos_rot  = posx([422.25, 145.53, 300.0, 91.1, -91.0, 180.0])
            pos_down = posx([422.25, 145.53,  99.15, 91.1, -91.0, 180.0])

            # 시작 시 열려있도록 보장
            set_digital_output(1, 0)
            set_digital_output(2, 1)

            logger.info("-> 'ㄷ'자 궤적으로 종이 잡는 위치로 접근")
            movel(p1, vel=self.MOVEL_VEL, acc=self.MOVEL_ACC)
            movel(p2, vel=self.MOVEL_VEL, acc=self.MOVEL_ACC)
            movel(p3, vel=self.SLOW_VEL, acc=self.SLOW_ACC)
            
            logger.info("-> 종이 잡기")
            set_digital_output(1, 1); set_digital_output(2, 0)
            wait(1.0)

            logger.info("-> 종이 들고 180도 회전")
            movel(pos_lift, vel=self.MOVEL_VEL, acc=self.MOVEL_ACC)
            movel(pos_rot, vel=self.MOVEL_VEL, acc=self.MOVEL_ACC)
            wait(0.5)

            logger.info("-> 뒤집은 상태로 내려놓기")
            movel(pos_down, vel=self.SLOW_VEL, acc=self.SLOW_ACC)
            set_digital_output(1, 0); set_digital_output(2, 1)
            wait(1.0)
            
            logger.info("-> 'ㄷ'자 궤적으로 빠져나와 홈 복귀")
            movel(p2, vel=self.MOVEL_VEL, acc=self.MOVEL_ACC)
            movel(p1, vel=self.MOVEL_VEL, acc=self.MOVEL_ACC)
            movej(Q1, vel=self.MOVEJ_VEL, acc=self.MOVEJ_ACC)

            logger.info("종이 뒤집기 완료!")
            success = True
            
        except Exception as e:
            logger.error(f"종이 뒤집기 중 에러: {e}")
            
        return success


# ==========================================
# [4] 점자 타각 작업 클래스
# ==========================================
class BrailleTask:
    def __init__(self, movej_vel=200.0, movej_acc=200.0, move_vel=150.0, move_acc=150.0, z_vel=200.0, z_acc=200.0, punch_force=15.0, char_offset=10.0):
        self.MOVEJ_VEL = movej_vel
        self.MOVEJ_ACC = movej_acc
        self.MOVE_VEL = move_vel
        self.MOVE_ACC = move_acc
        self.Z_VEL = z_vel
        self.Z_ACC = z_acc
        self.PUNCH_FORCE = punch_force
        self.CHAR_OFFSET = char_offset
        self.EPS = 1e-3

    def execute(self, flat_bits, logger):
        from DSR_ROBOT2 import (movej, movel, set_digital_output, wait, 
                                set_ref_coord, task_compliance_ctrl, set_desired_force, 
                                release_force, release_compliance_ctrl, check_force_condition, get_current_posx,
                                DR_TOOL, DR_BASE, DR_AXIS_Z, DR_MV_MOD_REL)
        from DR_common2 import posx, posj
        
        success = False
        try:
            Q1 = posj([0.0, 25.0, 55.0, 0.0, 100.0, 0.0])
            set_digital_output(1, 0); set_digital_output(2, 1)
            
            logger.info("점자 초기 위치로 이동 중...")
            movej(Q1, vel=self.MOVEJ_VEL, acc=self.MOVEJ_ACC)
            
            print("점필을 쥐어주세요 (5초 대기)")
            wait(5.0)
            set_digital_output(1, 1); set_digital_output(2, 0)
            wait(1.0)
            
            
            # 핀 들기
            movel(posx([0.0, 0.0, 20.0, 0.0, 0.0, 0.0]), vel=self.Z_VEL, acc=self.Z_ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            logger.info("점자 타각 시작!")

            def punch_dot():
                safe_pos = get_current_posx(ref=DR_BASE)[0]
                try:
                    movel(posx([0.0, 0.0, 30.5, 0.0, 0.0, 0.0]), vel=self.Z_VEL, acc=self.Z_ACC, ref=DR_TOOL)
                    set_ref_coord(DR_TOOL)
                    print("툴 좌표계 설정 및 순응 제어 켜기")
                    task_compliance_ctrl([3000.0, 3000.0, 1000.0, 200.0, 200.0, 200.0], time=0.2)
                    set_desired_force([0.0, 0.0, self.PUNCH_FORCE, 0.0, 0.0, 0.0], dir=[0, 0, 1, 0, 0, 0], time=0.2, mod=0)
                    check_force_condition(DR_AXIS_Z, min=1, ref=DR_TOOL)
                finally:
                    release_force(time=0.2)
                    release_compliance_ctrl()
                    set_ref_coord(DR_BASE)
                    movel(safe_pos, vel=self.Z_VEL, acc=self.Z_ACC, ref=DR_BASE)

            num_chars = len(flat_bits) // 6
            for i in range(num_chars):
                bits = flat_bits[i*6 : (i+1)*6]
                char_cur_x, char_cur_y = 0.0, 0.0
                
                for j, val in enumerate(bits):
                    if val == 1:
                        target_x, target_y = (j // 3) * 5, (j % 3) * 5
                        dx, dy = target_x - char_cur_x, target_y - char_cur_y
                        if abs(dx) > self.EPS or abs(dy) > self.EPS:
                            movel(posx([dx, dy, 0.0, 0.0, 0.0, 0.0]), vel=self.MOVE_VEL, acc=self.MOVE_ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                        punch_dot()
                        char_cur_x, char_cur_y = target_x, target_y
                        
                if i != num_chars - 1:
                    movel(posx([self.CHAR_OFFSET - char_cur_x, -char_cur_y, 0.0, 0.0, 0.0, 0.0]), vel=self.MOVE_VEL, acc=self.MOVE_ACC, ref=DR_TOOL)

            success = True
            
            # 종료 후 뱉기
            movel(posx([0.0, 0.0, -35.0, 0.0, 0.0, 0.0]), vel=self.Z_VEL, acc=self.Z_ACC, ref=DR_TOOL)
            movej(Q1, vel=self.MOVEJ_VEL, acc=self.MOVEJ_ACC)
            set_digital_output(1, 0); set_digital_output(2, 1)

        except Exception as e:
            logger.error(f"점자 에러: {e}")
            
        return success


# ==========================================
# [5] 통신 담당 메인 노드 
# ==========================================
class RobotControlNode(Node):
    def __init__(self, writer_obj, braille_obj, flipper_obj):
        super().__init__('robot_control_node', namespace=ROBOT_ID)
        
        self.writer = writer_obj
        self.braille_printer = braille_obj
        self.flipper = flipper_obj
        
        self.sub_write = self.create_subscription(String, '/write_cmd', self.write_cmd_cb, 10)
        self.sub_braille = self.create_subscription(Int32MultiArray, '/braille_cmd', self.braille_cmd_cb, 10)
        self.pub_write_done = self.create_publisher(Bool, '/write_done', 10)
        self.pub_braille_done = self.create_publisher(Bool, '/braille_done', 10)
        
        self.task_queue = [] 

    def write_cmd_cb(self, msg):
        self.get_logger().info(f"[명령 수신] 글쓰기: {msg.data}")
        self.task_queue.append(('write', msg.data))

    def braille_cmd_cb(self, msg):
        self.get_logger().info("[명령 수신] 점자 타각")
        self.task_queue.append(('braille', list(msg.data)))

    def process_queue(self):
        """큐에 담긴 작업을 하나씩 꺼내어 실행"""
        if self.task_queue:
            task_type, data = self.task_queue.pop(0)
            
            if task_type == 'write':
                # 1. 글쓰기 먼저 실행
                is_success = self.writer.execute(data, self.get_logger())
                
                # 2. 글쓰기가 성공했다면, "종이 뒤집기"를 통신 없이 내부적으로 바로 실행!
                if is_success:
                    self.get_logger().info("글쓰기 완료! 이어서 자동으로 종이 뒤집기를 시작합니다.")
                    is_success = self.flipper.execute(self.get_logger())
                
                # 3. 글쓰기 + 종이 뒤집기의 최종 결과를 MasterNode에게 보고
                res = Bool()
                res.data = is_success
                self.pub_write_done.publish(res)
                
            elif task_type == 'braille':
                # 4. MasterNode가 점자 명령을 쏘면 실행
                is_success = self.braille_printer.execute(data, self.get_logger())
                res = Bool()
                res.data = is_success
                self.pub_braille_done.publish(res)

# ==========================================
# [6] 메인 실행 함수
# ==========================================
def main(args=None):
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    # 객체 생성 (속도/가속도 파라미터 세팅)
    my_writer = WriteTask(
        movej_vel=200.0, movej_acc=200.0,
        draw_vel=200.0,  draw_acc=200.0,
        z_vel=200.0,     z_acc=200.0,
        letter_size=20.0, letter_space=25.0
    )
    
    my_flipper = FlipTask(
        movej_vel=50.0, movej_acc=50.0, 
        movel_vel=50.0, movel_acc=50.0,
        slow_vel=50.0,   slow_acc=50.0
    )
    
    my_braille = BrailleTask(
        movej_vel=200.0, movej_acc=200.0,
        move_vel=100.0,  move_acc=100.0,
        z_vel=20.0,      z_acc=20.0,
        punch_force=15.0, char_offset=10.0
    )

    # 3. 노드 생성 및 주입
    node = RobotControlNode(my_writer, my_braille, my_flipper)
    DR_init.__dsr__node = node

    # 4. 초기화!
    initialize_robot()
    node.get_logger().info(" 로봇 통합 제어 노드 가동 완료")

    try:
        while rclpy.ok():
            node.process_queue()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()