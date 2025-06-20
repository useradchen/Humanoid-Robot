import pygame
import serial
import time
from collections import deque

# 初始化 pygame 和搖桿
pygame.init()
pygame.joystick.init()
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"成功初始化手柄: {joystick.get_name()}")
except pygame.error as e:
    print(f"手柄初始化失敗: {e}")
    pygame.quit()
    exit()

# 檢測搖桿漂移
print("檢測搖桿漂移，請保持手柄靜止...")
time.sleep(1)
pygame.event.pump()
x_axis_right_samples = deque(maxlen=30)
y_axis_samples = deque(maxlen=30)
for _ in range(30):
    x_axis_right_samples.append(joystick.get_axis(3))
    y_axis_samples.append(joystick.get_axis(1))
    time.sleep(0.05)
x_axis_right_offset = sum(x_axis_right_samples) / len(x_axis_right_samples)
y_axis_offset = sum(y_axis_samples) / len(y_axis_samples)
dead_zone = max(0.25, abs(x_axis_right_offset) + 0.05, abs(y_axis_offset) + 0.05)
print(f"右搖桿漂移: {x_axis_right_offset:.3f}, 左搖桿漂移: {y_axis_offset:.3f}, 死區設為: {dead_zone:.3f}")

# 設置串口
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)
    time.sleep(2)
except serial.SerialException as e:
    print(f"串口錯誤: {e}")
    pygame.quit()
    exit()

print("=== PS4 手柄準備就緒 ===")

try:
    last_command_time = 0.0
    stop_duration = 0.0
    stopped = False
    waiting_for_input = True
    y_axis_filter = deque(maxlen=7)  # 增加濾波窗口
    x_axis_right_filter = deque(maxlen=7)
    last_stepLength = 0.0
    last_rotationAngle = 0.0

    while True:
        pygame.event.pump()
        current_time = time.time()

        # 檢查串口
        if not ser.is_open:
            print("串口斷開，嘗試重新連接...")
            try:
                ser.open()
                time.sleep(2)
            except serial.SerialException as e:
                print(f"重新連接失敗: {e}")
                break

        # 讀取輸入並濾波
        y_axis_raw = joystick.get_axis(1) - y_axis_offset
        x_axis_right_raw = joystick.get_axis(3) - x_axis_right_offset
        y_axis_filter.append(y_axis_raw)
        x_axis_right_filter.append(x_axis_right_raw)
        y_axis = sum(y_axis_filter) / len(y_axis_filter)
        x_axis_right = sum(x_axis_right_filter) / len(x_axis_right_filter)

        # 調試搖桿值
        print(f"校正後右搖桿: {x_axis_right:.3f}, 左搖桿: {y_axis:.3f}")

        l1_button = joystick.get_button(4)
        r1_button = joystick.get_button(5)
        square_button = joystick.get_button(3)
        circle_button = joystick.get_button(1)

        # 調試按鍵
        if l1_button:
            print("檢測到 L1 按下")
        if square_button:
            print("檢測到正方形按下")

        # 計算參數
        stepLength = 0.0
        if abs(y_axis) > dead_zone:
            stepLength = -y_axis * 70
            stepLength = max(min(stepLength, 70), -50)
        # 平滑 stepLength
        stepLength = 0.8 * stepLength + 0.2 * last_stepLength
        last_stepLength = stepLength

        rotationAngle = 0.0
        if abs(x_axis_right) > dead_zone:
            rotationAngle = 10.0 * (x_axis_right / max(abs(x_axis_right), 0.5))  # 線性縮放
        # 平滑 rotationAngle
        rotationAngle = 0.8 * rotationAngle + 0.2 * last_rotationAngle
        last_rotationAngle = rotationAngle

        sideStepLength = 0.0

        # 顯著操作
        significant_input = (l1_button or r1_button or circle_button or
                            abs(y_axis) > 0.5 or abs(x_axis_right) > 0.5)

        # 指令處理
        cmd = ""
        if stop_duration > 0 and current_time - last_command_time < stop_duration:
            time.sleep(1.0)
            continue

        if waiting_for_input and not significant_input:
            time.sleep(1.0)
            continue
        elif l1_button:
            cmd = "initialize\n"
            stopped = False
            waiting_for_input = True  # 初始化後等待輸入
            stop_duration = 0.5
            last_stepLength = 0.0
            last_rotationAngle = 0.0
        elif square_button:
            cmd = "q\n"
            stopped = True
            waiting_for_input = False
            stop_duration = 3.0
            last_stepLength = 0.0
            last_rotationAngle = 0.0
        elif r1_button:
            cmd = "run,0.000,0.000,0.000\n"
            stopped = False
            waiting_for_input = False
            stop_duration = 0.2
        elif circle_button:
            cmd = "kick,30.000,0.000,0.000\n"
            stopped = False
            waiting_for_input = False
            stop_duration = 1.5
        elif stopped and not significant_input:
            cmd = "q\n"
            stop_duration = 0.5
        else:
            cmd = f"run,{stepLength:.3f},{rotationAngle:.3f},{sideStepLength:.3f}\n"
            stopped = False
            waiting_for_input = False
            stop_duration = 0.0

        # 發送指令
        if cmd:
            print(f"[指令] → {cmd.strip()}")
            try:
                ser.write(cmd.encode())
                ser.flush()
                last_command_time = current_time
            except serial.SerialException as e:
                print(f"串口發送錯誤: {e}")

        time.sleep(1.0)  # 1Hz

except KeyboardInterrupt:
    print("程式終止")
    ser.close()
    pygame.quit()
except Exception as e:
    print(f"程式錯誤: {e}")
    ser.close()
    pygame.quit()