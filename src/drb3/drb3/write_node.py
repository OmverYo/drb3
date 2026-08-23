import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

VELOCITY, ACC = 200, 200
PEN_Z_VELOCITY, PEN_Z_ACC = 200, 200
EPS = 1e-3

# (여기에 기존 HangulEngine 클래스를 그대로 복사해 넣으세요!)

class WriteNode(Node):
    def __init__(self):
        super().__init__('write_node', namespace=ROBOT_ID)
        DR_init.__dsr__node = self
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL

        self.sub = self.create_subscription(String, '/write_cmd', self.cmd_callback, 10)
        self.pub = self.create_publisher(Bool, '/write_done', 10)
        
        self.is_working = False
        self.pen_state = "down"

        # 로봇 초기 셋업
        from DSR_ROBOT2 import set_tool, set_tcp, movej, set_digital_output, wait
        from DR_common2 import posj
        
        self.Q1 = posj(0.0, 25.0, 60.0, 0.0, 94.5, 0)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        
        # 그리퍼 초기화
        set_digital_output(1, 0); set_digital_output(2, 1) # open
        self.get_logger().info("초기 위치 이동 중...")
        movej(self.Q1, vel=VELOCITY, acc=ACC)
        
        print("펜을 쥐어주세요 (5초 대기)")
        wait(5.0)
        set_digital_output(1, 1); set_digital_output(2, 0) # close
        wait(1.0)
        
        self.get_logger().info("글쓰기 대기 모드 진입 완료.")

    def cmd_callback(self, msg):
        if self.is_working:
            self.get_logger().warn("현재 작업 중이라 새 명령을 무시합니다.")
            return
            
        text = msg.data
        self.get_logger().info(f"명령 수신: {text}")
        
        # 스레드에서 로봇 이동 시작 (spin 블로킹 방지)
        threading.Thread(target=self.do_writing_task, args=(text,)).start()

    def do_writing_task(self, text):
        from DSR_ROBOT2 import movel, DR_TOOL
        from DR_common2 import posx
        
        self.is_working = True
        success = False
        
        def move_rel(dx, dy, dz=0.0):
            if abs(dx) < EPS and abs(dy) < EPS and abs(dz) < EPS: return
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
            engine = HangulEngine() # 위에 선언된 엔진 인스턴스화
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
            self.is_working = False
            res_msg = Bool()
            res_msg.data = success
            self.pub.publish(res_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WriteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()