import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

VELOCITY, ACC = 200, 200
PEN_Z_VELOCITY, PEN_Z_ACC = 200, 200
EPS = 1e-3


# ==========================================
# [1] 한글 동적 렌더링 엔진
# ==========================================
class HangulEngine:
    CHO = ['ㄱ','ㄲ','ㄴ','ㄷ','ㄸ','ㄹ','ㅁ','ㅂ','ㅃ','ㅅ','ㅆ','ㅇ','ㅈ','ㅉ','ㅊ','ㅋ','ㅌ','ㅍ','ㅎ']
    JUNG = ['ㅏ','ㅐ','ㅑ','ㅒ','ㅓ','ㅔ','ㅕ','ㅖ','ㅗ','ㅘ','ㅙ','ㅚ','ㅛ','ㅜ','ㅝ','ㅞ','ㅟ','ㅠ','ㅡ','ㅢ','ㅣ']
    JONG = ['','ㄱ','ㄲ','ㄳ','ㄴ','ㄵ','ㄶ','ㄷ','ㄹ','ㄺ','ㄻ','ㄼ','ㄽ','ㄾ','ㄿ','ㅀ','ㅁ','ㅂ','ㅄ','ㅅ','ㅆ','ㅇ','ㅈ','ㅊ','ㅋ','ㅌ','ㅍ','ㅎ']

    V_MOEUM = [0, 1, 2, 3, 4, 5, 6, 7, 20]
    H_MOEUM = [8, 12, 13, 17, 18]
    M_MOEUM = [9, 10, 11, 14, 15, 16, 19]

    JAMO_STROKES = {
        'ㄱ': [[(0.1, 0.1), (0.9, 0.1), (0.9, 0.9)]],
        'ㄴ': [[(0.1, 0.1), (0.1, 0.9), (0.9, 0.9)]],
        'ㄷ': [[(0.1, 0.1), (0.9, 0.1)], [(0.1, 0.1), (0.1, 0.9), (0.9, 0.9)]],
        'ㄹ': [[(0.1, 0.1), (0.9, 0.1), (0.9, 0.5), (0.1, 0.5), (0.1, 0.9), (0.9, 0.9)]],
        'ㅁ': [[(0.1, 0.1), (0.1, 0.9)], [(0.1, 0.1), (0.9, 0.1), (0.9, 0.9), (0.1, 0.9)]],
        'ㅂ': [[(0.2, 0.1), (0.2, 0.9)], [(0.8, 0.1), (0.8, 0.9)], [(0.2, 0.5), (0.8, 0.5)], [(0.2, 0.9), (0.8, 0.9)]],
        'ㅅ': [[(0.5, 0.1), (0.1, 0.9)], [(0.5, 0.5), (0.9, 0.9)]],
        'ㅇ': [[(0.5, 0.1), (0.2, 0.3), (0.2, 0.7), (0.5, 0.9), (0.8, 0.7), (0.8, 0.3), (0.5, 0.1)]],
        'ㅈ': [[(0.1, 0.1), (0.9, 0.1)], [(0.5, 0.1), (0.1, 0.9)], [(0.5, 0.5), (0.9, 0.9)]],
        'ㅊ': [[(0.4, 0.0), (0.6, 0.0)], [(0.1, 0.2), (0.9, 0.2)], [(0.5, 0.2), (0.1, 0.9)], [(0.5, 0.5), (0.9, 0.9)]],
        'ㅋ': [[(0.1, 0.1), (0.9, 0.1), (0.9, 0.9)], [(0.1, 0.5), (0.9, 0.5)]],
        'ㅌ': [[(0.1, 0.1), (0.9, 0.1)], [(0.1, 0.5), (0.9, 0.5)], [(0.1, 0.1), (0.1, 0.9), (0.9, 0.9)]],
        'ㅍ': [[(0.1, 0.1), (0.9, 0.1)], [(0.3, 0.1), (0.3, 0.9)], [(0.7, 0.1), (0.7, 0.9)], [(0.1, 0.9), (0.9, 0.9)]],
        'ㅎ': [[(0.4, 0.0), (0.6, 0.0)], [(0.2, 0.2), (0.8, 0.2)], [(0.5, 0.3), (0.3, 0.5), (0.5, 0.7), (0.7, 0.5), (0.5, 0.3)]],
        'ㄲ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.9)], [(0.5, 0.1), (0.9, 0.1), (0.9, 0.9)]],
        'ㄸ': [[(0.0, 0.1), (0.4, 0.1)], [(0.0, 0.1), (0.0, 0.9), (0.4, 0.9)], [(0.5, 0.1), (0.9, 0.1)], [(0.5, 0.1), (0.5, 0.9), (0.9, 0.9)]],
        'ㅃ': [[(0.1, 0.1), (0.1, 0.9)], [(0.4, 0.1), (0.4, 0.9)], [(0.1, 0.5), (0.4, 0.5)], [(0.1, 0.9), (0.4, 0.9)], [(0.6, 0.1), (0.6, 0.9)], [(0.9, 0.1), (0.9, 0.9)], [(0.6, 0.5), (0.9, 0.5)], [(0.6, 0.9), (0.9, 0.9)]],
        'ㅆ': [[(0.2, 0.1), (0.0, 0.9)], [(0.2, 0.5), (0.4, 0.9)], [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]],
        'ㅉ': [[(0.0, 0.1), (0.4, 0.1)], [(0.2, 0.1), (0.0, 0.9)], [(0.2, 0.5), (0.4, 0.9)], [(0.5, 0.1), (0.9, 0.1)], [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]],
        'ㄳ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.9)], [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]],
        'ㄵ': [[(0.0, 0.1), (0.0, 0.9), (0.4, 0.9)], [(0.5, 0.1), (0.9, 0.1)], [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]],
        'ㄶ': [[(0.0, 0.1), (0.0, 0.9), (0.4, 0.9)], [(0.6, 0.0), (0.8, 0.0)], [(0.5, 0.2), (0.9, 0.2)], [(0.7, 0.4), (0.5, 0.6), (0.7, 0.8), (0.9, 0.6), (0.7, 0.4)]],
        'ㄺ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], [(0.5, 0.1), (0.9, 0.1), (0.9, 0.9)]],
        'ㄻ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], [(0.5, 0.1), (0.5, 0.9)], [(0.5, 0.1), (0.9, 0.1), (0.9, 0.9), (0.5, 0.9)]],
        'ㄼ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], [(0.6, 0.1), (0.6, 0.9)], [(0.9, 0.1), (0.9, 0.9)], [(0.6, 0.5), (0.9, 0.5)], [(0.6, 0.9), (0.9, 0.9)]],
        'ㄽ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]],
        'ㄾ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], [(0.5, 0.1), (0.9, 0.1)], [(0.5, 0.5), (0.9, 0.5)], [(0.5, 0.1), (0.5, 0.9), (0.9, 0.9)]],
        'ㄿ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], [(0.5, 0.1), (0.9, 0.1)], [(0.6, 0.1), (0.6, 0.9)], [(0.8, 0.1), (0.8, 0.9)], [(0.5, 0.9), (0.9, 0.9)]],
        'ㅀ': [[(0.0, 0.1), (0.4, 0.1), (0.4, 0.5), (0.0, 0.5), (0.0, 0.9), (0.4, 0.9)], [(0.6, 0.0), (0.8, 0.0)], [(0.5, 0.2), (0.9, 0.2)], [(0.7, 0.4), (0.5, 0.6), (0.7, 0.8), (0.9, 0.6), (0.7, 0.4)]],
        'ㅄ': [[(0.0, 0.1), (0.0, 0.9)], [(0.4, 0.1), (0.4, 0.9)], [(0.0, 0.5), (0.4, 0.5)], [(0.0, 0.9), (0.4, 0.9)], [(0.7, 0.1), (0.5, 0.9)], [(0.7, 0.5), (0.9, 0.9)]],
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
        'ㅐ': [[(0.3, 0.1), (0.3, 0.9)], [(0.3, 0.5), (0.8, 0.5)], [(0.8, 0.1), (0.8, 0.9)]],
        'ㅔ': [[(0.1, 0.5), (0.4, 0.5)], [(0.4, 0.1), (0.4, 0.9)], [(0.8, 0.1), (0.8, 0.9)]],
        'ㅒ': [[(0.3, 0.1), (0.3, 0.9)], [(0.3, 0.4), (0.8, 0.4)], [(0.3, 0.6), (0.8, 0.6)], [(0.8, 0.1), (0.8, 0.9)]],
        'ㅖ': [[(0.1, 0.4), (0.4, 0.4)], [(0.1, 0.6), (0.4, 0.6)], [(0.4, 0.1), (0.4, 0.9)], [(0.8, 0.1), (0.8, 0.9)]],
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
        if char_jamo not in HangulEngine.JAMO_STROKES:
            return []
        strokes = HangulEngine.JAMO_STROKES[char_jamo]
        transformed = []
        for stroke in strokes:
            new_stroke = []
            for (x, y) in stroke:
                new_stroke.append((bx + (x * bw), by + (y * bh)))
            transformed.append(new_stroke)
        return transformed

    @classmethod
    def get_char_strokes(cls, char, box_width=40.0, box_height=40.0):
        if char == " ":
            return []
        code = ord(char)
        if code < 0xAC00 or code > 0xD7A3:
            return []

        offset = code - 0xAC00
        jong_idx = offset % 28
        jung_idx = (offset // 28) % 21
        cho_idx = (offset // 28) // 21

        cho_char = cls.CHO[cho_idx]
        jung_char = cls.JUNG[jung_idx]
        jong_char = cls.JONG[jong_idx]

        strokes = []
        has_jong = jong_idx != 0

        top_h = box_height * 0.6 if has_jong else box_height
        bot_h = box_height * 0.4 if has_jong else 0
        bot_y = box_height * 0.6

        if jung_idx in cls.V_MOEUM:
            strokes.extend(cls.scale_strokes(cho_char, 0, 0, box_width * 0.5, top_h))
            strokes.extend(cls.scale_strokes(jung_char, box_width * 0.5, 0, box_width * 0.5, top_h))
        elif jung_idx in cls.H_MOEUM:
            strokes.extend(cls.scale_strokes(cho_char, 0, 0, box_width, top_h * 0.5))
            strokes.extend(cls.scale_strokes(jung_char, 0, top_h * 0.5, box_width, top_h * 0.5))
        else:
            strokes.extend(cls.scale_strokes(cho_char, 0, 0, box_width * 0.5, top_h * 0.5))
            strokes.extend(cls.scale_strokes(jung_char, 0, 0, box_width, top_h))

        if has_jong:
            strokes.extend(cls.scale_strokes(jong_char, 0, bot_y, box_width, bot_h))

        return strokes


# ==========================================
# [2] 글쓰기 ROS 2 노드
# ==========================================
class WriteNode(Node):
    def __init__(self):
        super().__init__('write_node', namespace=ROBOT_ID)
        self.sub = self.create_subscription(String, '/write_cmd', self.cmd_callback, 10)
        self.pub = self.create_publisher(Bool, '/write_done', 10)
        
        self.received_text = None
        self.pen_state = "down"

    def initialize_robot_hardware(self):
        from DSR_ROBOT2 import set_tool, set_tcp, movej, set_digital_output, wait
        from DR_common2 import posj
        
        self.Q1 = posj(0.0, 25.0, 60.0, 0.0, 94.5, 0)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        
        set_digital_output(1, 0)
        set_digital_output(2, 1)  # open
        self.get_logger().info("초기 위치 이동 중...")
        movej(self.Q1, vel=VELOCITY, acc=ACC)
        
        print("펜을 쥐어주세요 (5초 대기)")
        wait(5.0)
        set_digital_output(1, 1)
        set_digital_output(2, 0)  # close
        wait(1.0)
        
        self.get_logger().info("글쓰기 대기 모드 진입 완료.")

    def cmd_callback(self, msg):
        self.received_text = msg.data
        self.get_logger().info(f"명령 수신: {self.received_text}")

    def do_writing_task(self, text):
        from DSR_ROBOT2 import movel, DR_TOOL
        from DR_common2 import posx
        
        success = False
        
        def move_rel(dx, dy, dz=0.0):
            if abs(dx) < EPS and abs(dy) < EPS and abs(dz) < EPS:
                return
            movel(posx(dx, dy, dz, 0.0, 0.0, 0.0), vel=VELOCITY, acc=ACC, ref=DR_TOOL)

        def pen_up():
            if self.pen_state == "down":
                move_rel(0.0, 0.0, -10.0)
                self.pen_state = "up"

        def pen_down():
            if self.pen_state == "up":
                move_rel(0.0, 0.0, 10.0)
                self.pen_state = "down"

        def draw_letter(strokes, offset_x=0.0, offset_y=0.0, advance=True):
            cur_x, cur_y = 0.0, 0.0
            first_stroke = True
            for stroke in strokes:
                start_x, start_y = stroke[0]
                if first_stroke and self.pen_state == "down":
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

        try:
            self.pen_state = "down"
            engine = HangulEngine()
            LETTER_SIZE, LETTER_SPACE = 20.0, 10.0
            
            for i, char in enumerate(text):
                if char == " ":
                    move_rel(LETTER_SPACE, 0.0)
                    continue
                strokes = engine.get_char_strokes(char, box_width=LETTER_SIZE, box_height=LETTER_SIZE)
                is_last = (i == len(text) - 1)
                draw_letter(strokes, offset_x=LETTER_SPACE, offset_y=0.0, advance=not is_last)
                
            success = True
        except Exception as e:
            self.get_logger().error(f"글쓰기 중 오류: {e}")
        finally:
            res_msg = Bool()
            res_msg.data = success
            self.pub.publish(res_msg)


def main(args=None):
    rclpy.init(args=args)

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    node = WriteNode()
    DR_init.__dsr__node = node

    from DSR_ROBOT2 import release_force, release_compliance_ctrl
    release_force(time=0.0)
    release_compliance_ctrl()

    node.initialize_robot_hardware()

    # 이벤트 루프: 대기 중일 때만 spin_once 실행, 명령 수신 시 메인 스레드에서 직접 모션 수행
    try:
        while rclpy.ok():
            if node.received_text is not None:
                text = node.received_text
                node.received_text = None
                node.do_writing_task(text)
            
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()