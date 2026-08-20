#완성 코드
def grasp():
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(1)

def release():
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(1)

Q1 = posj(0,0,90,0,90,0)
release()
movej(Q1)

movel(Global_p1) #위
down_z =  85
pos_down = posx(0, 0, down_z, 0, 0, 0)
movel(pos_down, ref=DR_TOOL)
grasp()
movel(Global_p1)

movel(Global_p4) #옮긴 위
movel(pos_down, ref=DR_TOOL)
release()
#### 1 cycle

movel(Global_p4)
movel(Global_p2)
movel(pos_down, ref=DR_TOOL)
grasp()
movel(Global_p2)

movel(Global_p5)
movel(pos_down, ref=DR_TOOL)
release()
### 2 cycle

movel(Global_p5)
movel(Global_p3)
movel(pos_down, ref=DR_TOOL)
grasp()
movel(Global_p3)

movel(Global_p6)
movel(pos_down, ref=DR_TOOL)
release()
### 3 cycle

movel(Global_p6)
movel(Global_c1)
movel(pos_down, ref=DR_TOOL)
grasp()
movel(Global_c1)

# 1. 조립 구멍 바로 위(어프로치 위치)로 먼저 이동합니다. 
# (Global_c2가 톱니바퀴 조립 목표 바닥 위치라면, 조립되기 전 공중에 있는 좌표로 먼저 가야합니다.)
# 아래는 현재 위치에서 조립부 상단으로 갔다고 가정합니다.
movel(Global_c2) # 조립 전 구멍 위에 정렬된 위치

# 2. 순응-힘 제어 켜기
stx = [500, 500, 500, 200, 200, 200]
task_compliance_ctrl(stx, time=0)

# Z축 방향으로 -15N의 힘을 주어 누르기 시작 
# (이때부터 로봇이 스스로 저항을 만날 때까지 아래로 내려가려 합니다)
fd = [0, 0, -15, 0, 0, 0]
fctrl_dir = [0, 0, 1, 0, 0, 0]
set_desired_force(fd, dir=fctrl_dir)

# 3. 비동기 회전 모션 시작 (반드시 while 루프 밖에서 1번만 실행!)
amp = [0, 0, 0, 0, 0, 10]    # Rz 방향 진폭 10도
period = [0, 0, 0, 0, 0, 1.0] # 1초 주기
amove_periodic(amp, period, repeat=100, ref=DR_TOOL) # 톱니가 맞을 때까지 넉넉히 반복

# 4. 삽입 완료 감시 루프
while True:
    # 루프 안에서 실시간으로 Z축 조건이 만족되었는지 계속 확인합니다.
    pcon1 = check_position_condition(DR_AXIS_Z, max=70, ref=DR_BASE) 
    
    if pcon1 == 1: # 조건 만족 시 (DRL에서는 1이 True를 의미)
        stop(DR_SSTOP) # 내려가는 힘과 주기적 회전 모션을 부드럽게 정지
        break
        
    wait(0.1) # 루프가 너무 빨리 돌아 시스템 부하가 생기는 것을 방지

# 그리퍼 열기 및 제어 해제
release()
release_force(time=0)
release_compliance_ctrl()

# 조립 후 위로 빠져나오기
# pos_up = posx(0, 0, -85, 0, 0, 0)
# movel(pos_up, ref=DR_TOOL)

movej(Q1)
### final cycle