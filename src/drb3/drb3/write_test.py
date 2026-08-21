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

# robot star setting 
def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp


    # 설정된 상수 출력
    print("-" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("-" * 50)

    # Tool과 TCP 설정
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

    # DR_init에 노드 설정
    DR_init.__dsr__node = node
    from DSR_ROBOT2 import wait, movej, movel, DR_SSTOP, movesx, DR_MVS_VEL_CONST, DR_BASE, DR_TOOL
    from DR_common2 import posx, posj
    Q1 = posj(0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
    p0=posx(0.0, 0.0, -10.0, 0.0, 0.0, 0.0) #넘어가기전 올라가기
    p1=posx(0.0, 0.0, 10.0, 0.0, 0.0, 0.0)  #넘어가서 내려가기
    p2=posx(-12.5, 0.0, 0.0, 0.0, 0.0, 0.0)
    p3=posx(0, -10.0, 0.0, 0.0, 0.0, 0.0)
    p4=posx(-40, 10, 0 , 0 ,0 ,0) #ㄱ 에서 ㅏ로 이동
    p5=posx(0.0, -7.5, 0.0, 0.0, 0.0, 0.0)
    p6=posx(-5.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    p7=posx(5.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    p8 = posx(40, -2.5, 0 ,0 ,0 ,0)# ㅏ 에서 ㅁ으로 이동
    p9=posx(-40.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    p10=posx(0.0, -10.0, 0.0, 0.0, 0.0, 0.0)
    p11=posx(40.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    p12=posx(0.0, 10.0, 0.0, 0.0, 0.0, 0.0)
    


    plist1 = [p2, p3]
    plist2 = [p5, p6, p7, p8]
    plist3 = [p9, p10, p11, p12]


    try:
            
        initialize_robot()
        grip_open()
        node.get_logger().info(f"Moving to joint position: {Q1}")
        movej(Q1, vel=VELOCITY, acc=ACC)
        
        print("pen please")
        wait(5.0)


        grip_close()
        node.get_logger().info(f"griper open")
        
        print("ㄱ 시작")
        movesx(plist1, vel=20, acc = 20, ref=DR_TOOL, vel_opt = DR_MVS_VEL_CONST)
        movel(p0, vel =VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(p4, vel =VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(p1, vel =VELOCITY, acc=ACC, ref=DR_TOOL)
        print("ㅏ 시작")
        movesx(plist2, vel=20, acc = 20, ref=DR_TOOL, vel_opt = DR_MVS_VEL_CONST)
        movel(p0, vel =VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(p8, vel =VELOCITY, acc=ACC, ref=DR_TOOL)
        movel(p1, vel =VELOCITY, acc=ACC, ref=DR_TOOL)
        print("ㅁ 시작")
        movesx(plist3, vel=20, acc = 20, ref=DR_TOOL, vel_opt = DR_MVS_VEL_CONST)
        movel(p0, vel =VELOCITY, acc=ACC, ref=DR_TOOL)
        


        #grip_open()
        node.get_logger().info(f"griper close")

        
        node.get_logger().info(f"STOP")
        
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        movej(Q1, vel=VELOCITY, acc=ACC)
        
        rclpy.shutdown()
if __name__ == "__main__":
    main()
