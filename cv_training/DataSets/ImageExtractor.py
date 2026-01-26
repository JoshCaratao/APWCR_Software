import cv2
import os

video_path = "input_video.mp4"
out_dir = "frames"
fps_extract = 2  # frames per second

os.makedirs(out_dir, exist_ok=True)

cap = cv2.VideoCapture(video_path)
video_fps = cap.get(cv2.CAP_PROP_FPS)

frame_interval = int(video_fps / fps_extract)
frame_idx = 0
saved_idx = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if frame_idx % frame_interval == 0:
        out_path = os.path.join(out_dir, f"frame_{saved_idx:05d}.jpg")
        cv2.imwrite(out_path, frame)
        saved_idx += 1

    frame_idx += 1

cap.release()
print(f"Saved {saved_idx} frames")
