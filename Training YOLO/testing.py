# test_yolo_pc.py
from ultralytics import YOLO
import cv2
import time
import argparse
import os

# ── Config ────────────────────────────────────────────────────────────────────
MODEL_PATH  = "runs/detect/drone_xml_model_full/weights/best.pt"
CONF_THRESH = 0.5   # lower = more detections, more false positives
IOU_THRESH  = 0.5  # NMS threshold

def draw_detections(frame, results, fps=None):
    for result in results:
        boxes = result.boxes
        for box in boxes:
            # Bounding box
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf  = float(box.conf[0])
            cls   = int(box.cls[0])
            label = f"{result.names[cls]} {conf:.2f}"

            # Box color based on confidence
            color = (0, int(255 * conf), int(255 * (1 - conf)))

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label,
                (x1, y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Center dot — this is what you'd send to Arduino
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"center: ({cx},{cy})",
                (cx + 8, cy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # FPS overlay
    if fps:
        cv2.putText(frame, f"FPS: {fps:.1f}",
            (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
            1.0, (0, 255, 0), 2)

    # Frame center crosshair (where you want the drone to be)
    h, w = frame.shape[:2]
    cx_frame, cy_frame = w // 2, h // 2
    cv2.line(frame, (cx_frame - 20, cy_frame), (cx_frame + 20, cy_frame), (255, 255, 255), 1)
    cv2.line(frame, (cx_frame, cy_frame - 20), (cx_frame, cy_frame + 20), (255, 255, 255), 1)

    return frame


def test_on_image(model, path):
    """Run on a single image or folder of images"""
    if os.path.isdir(path):
        files = [os.path.join(path, f) for f in os.listdir(path)
                 if f.lower().endswith(('.jpg', '.png', '.jpeg'))]
    else:
        files = [path]

    for f in files:
        frame = cv2.imread(f)
        if frame is None:
            print(f"Could not read {f}")
            continue

        results = model(frame, conf=CONF_THRESH, iou=IOU_THRESH, verbose=False)
        frame   = draw_detections(frame, results)

        # Print detections to terminal
        for result in results:
            for box in result.boxes:
                x1,y1,x2,y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cx = (x1+x2)//2
                cy = (y1+y2)//2
                print(f"  Drone detected | conf: {conf:.2f} | "
                      f"bbox: ({x1},{y1}) -> ({x2},{y2}) | "
                      f"center: ({cx},{cy})")

        cv2.imshow(f"YOLO Test - {os.path.basename(f)}", frame)
        print(f"\n{os.path.basename(f)}: {len(results[0].boxes)} drone(s) found")
        print("Press any key for next image, Q to quit")

        key = cv2.waitKey(0)
        cv2.destroyAllWindows()
        if key == ord('q'):
            break


def test_on_video(model, path):
    """Run on a video file"""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"ERROR: could not open {path}")
        return

    fps_timer  = time.time()
    frame_count = 0
    fps = 0.0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, conf=CONF_THRESH, iou=IOU_THRESH, verbose=False)

        # FPS
        frame_count += 1
        elapsed = time.time() - fps_timer
        if elapsed >= 0.5:
            fps = frame_count / elapsed
            frame_count = 0
            fps_timer = time.time()

        frame = draw_detections(frame, results, fps)

        cv2.imshow("YOLO Test - Video", frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def test_on_webcam(model, cam_index=0):
    """Run on live webcam"""
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not cap.isOpened():
        print("ERROR: could not open webcam")
        return

    fps_timer   = time.time()
    frame_count = 0
    fps = 0.0
    run_yolo = True  # toggle with spacebar

    print("Controls: Q = quit | SPACE = pause YOLO | S = screenshot")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if run_yolo:
            results = model(frame, conf=CONF_THRESH, iou=IOU_THRESH, verbose=False)

            frame_count += 1
            elapsed = time.time() - fps_timer
            if elapsed >= 0.5:
                fps = frame_count / elapsed
                frame_count = 0
                fps_timer = time.time()

            frame = draw_detections(frame, results, fps)

            # Print to terminal if drone found
            for result in results:
                for box in result.boxes:
                    x1,y1,x2,y2 = map(int, box.xyxy[0])
                    cx = (x1+x2)//2
                    cy = (y1+y2)//2
                    h, w = frame.shape[:2]
                    # Error from center (what PID controller will use)
                    err_x = cx - w//2
                    err_y = cy - h//2
                    print(f"Drone | center: ({cx},{cy}) | "
                          f"error from frame center: ({err_x:+d}, {err_y:+d})")
        else:
            cv2.putText(frame, "YOLO PAUSED",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (0, 0, 255), 2)

        cv2.imshow("YOLO Test - Webcam", frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord(' '):
            run_yolo = not run_yolo
            print(f"YOLO {'resumed' if run_yolo else 'paused'}")
        elif key == ord('s'):
            fname = f"screenshot_{int(time.time())}.jpg"
            cv2.imwrite(fname, frame)
            print(f"Saved {fname}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLO Drone Detection Tester")
    parser.add_argument("--mode",    default="webcam",
                        choices=["image", "video", "webcam"],
                        help="Test mode")
    parser.add_argument("--source",  default="0",
                        help="Image path, video path, or webcam index")
    parser.add_argument("--model",   default=MODEL_PATH,
                        help="Path to .pt model file")
    parser.add_argument("--conf",    type=float, default=CONF_THRESH,
                        help="Confidence threshold")
    args = parser.parse_args()

    CONF_THRESH = args.conf

    print(f"Loading model: {args.model}")
    model = YOLO(args.model)
    print("Model loaded\n")

    if args.mode == "image":
        test_on_image(model, args.source)
    elif args.mode == "video":
        test_on_video(model, args.source)
    elif args.mode == "webcam":
        test_on_webcam(model, int(args.source))