import cv2
from datetime import datetime

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # رزولوشن پایین!
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 10)           # FPS پایین‌تر برای lag کمتر تست - بعداً زیاد کن
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # یا کامنت کن و YUYV تست کن

if not cap.isOpened():
    print("دوربین وصل نیست!")
    exit()

while True:
    for _ in range(2):  # کمتر grab = lag کمتر!
        cap.grab()
    ret, frame = cap.read()
    if not ret or frame is None:
        print("خطا در دریافت تصویر!")
        break

    cv2.imshow('USB Camera', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        now = datetime.now()
        filename = now.strftime("photo_%Y%m%d_%H%M%S.jpg")
        cv2.imwrite(filename, frame)
        print(f'عکس {filename} ذخیره شد.')
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()