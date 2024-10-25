from pyfirmata import Arduino, util
import time

# 設定 Arduino 板的裝置
board = Arduino('/dev/ttyACM0')

# 控制 LED 亮滅
while True:
    board.digital[13].write(1)  # 開 LED
    time.sleep(1)
    board.digital[13].write(0)  # 關 LED
    time.sleep(1)

