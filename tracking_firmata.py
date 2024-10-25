import cv2
from ultralytics import YOLO
import pafy
import argparse
import numpy as np
from pyfirmata import Arduino, util

# 初始化 YOLOv8 模型
model = YOLO('yolov8s.pt')

# 設定 Arduino 與初始化引腳
board = Arduino('/dev/ttyACM0')  # 根據實際情況設置你的 Arduino 連接埠
pin3 = board.get_pin('d:3:o')  # P3 作為輸出（高/低電位）
pin4 = board.get_pin('d:4:o')  # P4 作為輸出（高/低電位）
pin7 = board.get_pin('d:7:i')  # P7 作為輸入（按鈕）

# 初始化引腳狀態
pin3.write(0)  # 預設低電位
pin4.write(0)  # 預設低電位

# 初始化全域變數
entering_count = 0
leaving_count = 0
object_track_history = {}
my_conf = 0.25
my_iou = 0.45

# 設定要追蹤的類別群組（人流 or 車流）
def get_target_classes(flow_type):
    if (flow_type == 'person'):
        return ['person']
    elif (flow_type == 'vehicle'):
        return ['car', 'truck', 'motorcycle']
    else:
        raise ValueError("不支援的流量類型")

# 輸入來源設定
def get_input_source(source_type, source):
    if (source_type == 'file'):
        return cv2.VideoCapture(source)
    elif (source_type == 'webcam'):
        return cv2.VideoCapture(0)
    elif (source_type == 'youtube'):
        video = pafy.new(source)
        best = video.getbest(preftype="mp4")
        return cv2.VideoCapture(best.url)
    elif (source_type == 'rtsp'):
        return cv2.VideoCapture(source)
    else:
        raise ValueError("不支援的輸入來源類型")

# 計算直線的參數 A, B, C
def calculate_line_params(x1, y1, x2, y2):
    A = y1 - y2
    B = x2 - x1
    C = x1 * y2 - x2 * y1
    return A, B, C

# 判斷點 (x, y) 是否在直線的同一側
def point_side(x, y, A, B, C):
    return A * x + B * y + C

# 主程序：讀取來源、追蹤並計算進出流量
def main(flow_type, source_type, source, x1, y1, x2, y2):
    target_classes = get_target_classes(flow_type)

    it = util.Iterator(board)
    it.start()

    # 開啟輸入來源
    cap = get_input_source(source_type, source)

    if not cap.isOpened():
        print("Error: 無法開啟輸入來源")
        return

    global entering_count, leaving_count, object_track_history

    # 計算直線的參數 A, B, C
    A, B, C = calculate_line_params(x1, y1, x2, y2)

    # 迴圈讀取影片的每一幀
    while cap.isOpened():
        ret, frame = cap.read()

        # 如果無法讀取更多幀，結束迴圈
        if not ret:
            break

        # 使用 YOLOv8 模型進行物件追蹤
        results = model.track(frame, conf=my_conf, iou=my_iou, persist=True)

        detected_count = 0  # 用於計算當前畫面中偵測到的目標數量

        # 處理每個物件
        for result in results[0].boxes:
            cls_id = int(result.cls[0])
            cls_name = model.names[cls_id]

            # 只追蹤特定類別的物件
            if cls_name in target_classes:
                detected_count += 1  # 計算畫面中目標物件的總數

                # 取得物件邊界框的中心點位置
                x1_bbox, y1_bbox, x2_bbox, y2_bbox = result.xyxy[0]
                center_x = int((x1_bbox + x2_bbox) / 2)
                center_y = int((y1_bbox + y2_bbox) / 2)

                # 取得物件的追蹤 ID
                obj_id = int(result.id[0])

                # 畫出物件的邊界框和類別
                cv2.rectangle(frame, (int(x1_bbox), int(y1_bbox)), (int(x2_bbox), int(y2_bbox)), (0, 255, 0), 2)
                cv2.putText(frame, f'{cls_name} {obj_id}', (int(x1_bbox), int(y1_bbox) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # 追蹤物件的運動
                if obj_id not in object_track_history:
                    object_track_history[obj_id] = point_side(center_x, center_y, A, B, C)  # 記錄物件初始位置（相對邊界的側）
                else:
                    # 如果物件已經存在，檢查它的移動方向
                    previous_side = object_track_history[obj_id]
                    current_side = point_side(center_x, center_y, A, B, C)

                    if previous_side < 0 and current_side >= 0:
                        entering_count += 1  # 進入邊界區域
                    elif previous_side >= 0 and current_side < 0:
                        leaving_count += 1  # 離開邊界區域

                    # 更新物件的最新位置
                    object_track_history[obj_id] = current_side

        # 檢查統計數據是否超過 10 並控制 Arduino 引腳
        if leaving_count > 10:
            pin3.write(1)  # 設定 P3 高電位
        if entering_count > 10:
            pin4.write(1)  # 設定 P4 高電位

        # 檢查按鈕狀態，若按下則清除統計
        if pin7.read() == 0:
            print("reset")
            entering_count = 0
            leaving_count = 0
            pin3.write(0)  # 重設 P3 為低電位
            pin4.write(0)  # 重設 P4 為低電位

        # 畫出邊界線
        cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # 顯示進入和離開的統計數據
        cv2.putText(frame, f'Entering: {entering_count}', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f'Leaving: {leaving_count}', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 顯示當前畫面中偵測到的目標物件總數
        cv2.putText(frame, f'Total {flow_type.capitalize()}: {detected_count}', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # 顯示影像
        cv2.imshow('YOLOv8 Object Tracking', frame)

        # 按下 'q' 鍵結束
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 釋放影片與關閉視窗
    cap.release()
    cv2.destroyAllWindows()

# 設定命令列參數解析
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="YOLOv8 物件追蹤與車/人流量計算")

    # 流量類型參數 (人或車)
    parser.add_argument('--type', type=str, default='vehicle',
                        choices=['vehicle', 'person'],
                        help="選擇偵測對象: 'vehicle' 或 'person'")

    # 輸入來源參數 (file, webcam, youtube, rtsp)
    parser.add_argument('--input', type=str, default='file',
                        choices=['file', 'webcam', 'youtube', 'rtsp'],
                        help="選擇輸入來源類型: 'file', 'webcam', 'youtube', 'rtsp'")

    # 輸入來源位置（檔案名、URL等）
    parser.add_argument('--source', type=str, default='demo.mp4',
                        help="輸入來源，若使用 'file'，則提供檔案名稱；若使用 'youtube'，則提供 YouTube 影片 URL")

    # 邊界線的起點和終點
    parser.add_argument('--x1', type=int, default=0, help="邊界線起點 x1，預設 0")
    parser.add_argument('--y1', type=int, default=200, help="邊界線起點 y1，預設 200")
    parser.add_argument('--x2', type=int, default=640, help="邊界線終點 x2，預設 640")
    parser.add_argument('--y2', type=int, default=200, help="邊界線終點 y2，預設 200")

    args = parser.parse_args()

    # 執行主程式
    main(args.type, args.input, args.source, args.x1, args.y1, args.x2, args.y2)


