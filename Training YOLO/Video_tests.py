from ultralytics import YOLO
import cv2
import time
import os
import csv
import torch

# ── Config ────────────────────────────────────────────────────────────────────
MODEL_PATH   = "runs/detect/drone_xml_model_full/weights/best.pt"
TEST_DIR     = "../Test_videos"
CONF_THRESH  = 0.3
IOU_THRESH   = 0.45
SAVE_RESULTS = True   # save annotated images/videos to TEST_DIR/results/
FRAME_SKIP   = 1      # process every Nth frame (1 = every frame, 2 = half, 3 = third...)
IMGSZ        = 640    # inference resolution — try 416 for extra speed, 640 is YOLO default

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}
VIDEO_EXTS = {".mp4", ".avi", ".mov", ".mkv", ".wmv"}


# ── Device check ──────────────────────────────────────────────────────────────

def print_device_info():
    if torch.cuda.is_available():
        name = torch.cuda.get_device_name(0)
        mem  = torch.cuda.get_device_properties(0).total_memory / 1e9
        print(f"  Device : GPU — {name} ({mem:.1f} GB VRAM)")
        print(f"  Tip    : CUDA detected, YOLO will use GPU automatically")
    else:
        print("  Device : CPU only (no CUDA GPU detected)")
        print("  Tip    : CPU inference is slow. For real speed gains,")
        print("           use a machine with an NVIDIA GPU + CUDA.")
    print()


# ── Drawing ───────────────────────────────────────────────────────────────────

def draw_detections(frame, results, fps=None):
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf  = float(box.conf[0])
            cls   = int(box.cls[0])
            label = f"{result.names[cls]} {conf:.2f}"
            color = (0, int(255 * conf), int(255 * (1 - conf)))

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label,
                (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"({cx},{cy})",
                (cx + 8, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

    if fps is not None:
        cv2.putText(frame, f"FPS: {fps:.1f}",
            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

    h, w = frame.shape[:2]
    fcx, fcy = w // 2, h // 2
    cv2.line(frame, (fcx - 20, fcy), (fcx + 20, fcy), (255, 255, 255), 1)
    cv2.line(frame, (fcx, fcy - 20), (fcx, fcy + 20), (255, 255, 255), 1)
    return frame


# ── File discovery ────────────────────────────────────────────────────────────

def collect_files(directory):
    images, videos = [], []
    for fname in sorted(os.listdir(directory)):
        fpath = os.path.join(directory, fname)
        if not os.path.isfile(fpath):
            continue
        ext = os.path.splitext(fname)[1].lower()
        if ext in IMAGE_EXTS:
            images.append(fpath)
        elif ext in VIDEO_EXTS:
            videos.append(fpath)
    return images, videos


# ── Image batch ───────────────────────────────────────────────────────────────

def process_images(model, image_paths, save_dir=None):
    print(f"\n{'='*60}")
    print(f"  IMAGES  ({len(image_paths)} file(s))")
    print(f"{'='*60}")

    records = []

    for fpath in image_paths:
        frame = cv2.imread(fpath)
        if frame is None:
            print(f"[SKIP] Could not read {fpath}")
            continue

        t0      = time.perf_counter()
        results = model(frame, conf=CONF_THRESH, iou=IOU_THRESH,
                        imgsz=IMGSZ, verbose=False)
        inf_ms  = (time.perf_counter() - t0) * 1000

        boxes = results[0].boxes
        confs = [float(b.conf[0]) for b in boxes]
        n     = len(confs)

        print(f"\n[IMAGE] {os.path.basename(fpath)}")
        print(f"        {n} detection(s)  |  inference: {inf_ms:.1f} ms")
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            print(f"        conf: {conf:.2f} | "
                  f"bbox: ({x1},{y1})->({x2},{y2}) | center: ({cx},{cy})")

        records.append({
            "file":             os.path.basename(fpath),
            "type":             "image",
            "detections":       n,
            "avg_conf":         f"{sum(confs)/n:.3f}" if n else "N/A",
            "max_conf":         f"{max(confs):.3f}"   if n else "N/A",
            "min_conf":         f"{min(confs):.3f}"   if n else "N/A",
            "inference_ms":     f"{inf_ms:.1f}",
            "frames_processed": 1,
            "total_frames":     1,
            "avg_fps":          "N/A",
        })

        if save_dir:
            annotated = draw_detections(frame.copy(), results)
            out_path  = os.path.join(save_dir, "img_" + os.path.basename(fpath))
            cv2.imwrite(out_path, annotated)
            print(f"        saved → {out_path}")

    return records


# ── Video batch ───────────────────────────────────────────────────────────────

def process_videos(model, video_paths, save_dir=None):
    print(f"\n{'='*60}")
    print(f"  VIDEOS  ({len(video_paths)} file(s))")
    print(f"{'='*60}")

    records = []

    for fpath in video_paths:
        cap = cv2.VideoCapture(fpath)
        if not cap.isOpened():
            print(f"[SKIP] Could not open {fpath}")
            continue

        width        = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height       = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        src_fps      = cap.get(cv2.CAP_PROP_FPS) or 30.0
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

        print(f"\n[VIDEO] {os.path.basename(fpath)}")
        print(f"        {width}x{height} @ {src_fps:.1f} fps  |  "
              f"~{total_frames} frames  |  frame_skip={FRAME_SKIP}")

        writer  = None
        eff_fps = src_fps / FRAME_SKIP
        if save_dir:
            out_name = "vid_" + os.path.splitext(os.path.basename(fpath))[0] + ".mp4"
            out_path = os.path.join(save_dir, out_name)
            fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
            writer   = cv2.VideoWriter(out_path, fourcc, eff_fps, (width, height))

        fps_timer     = time.perf_counter()
        fps_count     = 0
        live_fps      = 0.0
        proc_frames   = 0
        total_frame_i = 0
        total_det     = 0
        all_confs     = []
        inf_times     = []

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            total_frame_i += 1

            # Skip frames — advance the capture but skip inference
            if total_frame_i % FRAME_SKIP != 0:
                continue

            t0      = time.perf_counter()
            results = model(frame, conf=CONF_THRESH, iou=IOU_THRESH,
                            imgsz=IMGSZ, verbose=False)
            inf_ms  = (time.perf_counter() - t0) * 1000
            inf_times.append(inf_ms)

            proc_frames += 1
            fps_count   += 1
            elapsed = time.perf_counter() - fps_timer
            if elapsed >= 1.0:
                live_fps  = fps_count / elapsed
                fps_count = 0
                fps_timer = time.perf_counter()

            boxes = results[0].boxes
            n     = len(boxes)
            total_det += n
            all_confs.extend([float(b.conf[0]) for b in boxes])

            if writer:
                annotated = draw_detections(frame.copy(), results, live_fps)
                writer.write(annotated)

            if proc_frames % 100 == 0:
                pct     = (total_frame_i / total_frames * 100) if total_frames else 0
                avg_inf = sum(inf_times[-100:]) / len(inf_times[-100:])
                print(f"        frame {total_frame_i}/{total_frames} ({pct:.0f}%)  |  "
                      f"{live_fps:.1f} fps  |  avg inf: {avg_inf:.1f} ms  |  "
                      f"{total_det} detections so far")

        cap.release()
        if writer:
            writer.release()
            print(f"        saved → {out_path}")

        avg_inf  = sum(inf_times) / len(inf_times) if inf_times else 0
        avg_fps  = proc_frames / (sum(inf_times) / 1000) if inf_times else 0
        avg_det  = total_det / proc_frames if proc_frames else 0
        avg_conf = sum(all_confs) / len(all_confs) if all_confs else None
        max_conf = max(all_confs) if all_confs else None
        min_conf = min(all_confs) if all_confs else None

        print(f"        ── Summary ──────────────────────────────────")
        print(f"        Frames processed : {proc_frames} / {total_frames} "
              f"(every {FRAME_SKIP} frame(s))")
        print(f"        Avg inference    : {avg_inf:.1f} ms/frame")
        print(f"        Throughput       : {avg_fps:.1f} fps")
        print(f"        Total detections : {total_det}")
        print(f"        Avg det/frame    : {avg_det:.2f}")
        if all_confs:
            print(f"        Avg confidence   : {avg_conf:.3f}")
            print(f"        Conf range       : {min_conf:.3f} – {max_conf:.3f}")

        records.append({
            "file":             os.path.basename(fpath),
            "type":             "video",
            "detections":       total_det,
            "avg_conf":         f"{avg_conf:.3f}" if avg_conf else "N/A",
            "max_conf":         f"{max_conf:.3f}" if max_conf else "N/A",
            "min_conf":         f"{min_conf:.3f}" if min_conf else "N/A",
            "inference_ms":     f"{avg_inf:.1f}",
            "frames_processed": proc_frames,
            "total_frames":     total_frames,
            "avg_fps":          f"{avg_fps:.1f}",
        })

    return records


# ── CSV report ────────────────────────────────────────────────────────────────

def save_csv(records, save_dir):
    path   = os.path.join(save_dir, "performance_report.csv")
    fields = ["file", "type", "detections", "avg_conf", "max_conf", "min_conf",
              "inference_ms", "frames_processed", "total_frames", "avg_fps"]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)
    print(f"\n  CSV report saved → {path}")


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    test_dir = os.path.abspath(TEST_DIR)
    if not os.path.isdir(test_dir):
        raise FileNotFoundError(f"Test directory not found: {test_dir}")

    save_dir = None
    if SAVE_RESULTS:
        save_dir = os.path.join(test_dir, "results")
        os.makedirs(save_dir, exist_ok=True)

    images, videos = collect_files(test_dir)
    print(f"\nTest directory : {test_dir}")
    print(f"Found          : {len(images)} image(s), {len(videos)} video(s)")
    if save_dir:
        print(f"Results dir    : {save_dir}")

    print_device_info()

    if not images and not videos:
        print("No supported files found. Exiting.")
        exit(0)

    print(f"Loading model  : {MODEL_PATH}")
    model = YOLO(MODEL_PATH)
    print("Model loaded\n")

    all_records = []
    if images:
        all_records += process_images(model, images, save_dir)
    if videos:
        all_records += process_videos(model, videos, save_dir)

    if save_dir and all_records:
        save_csv(all_records, save_dir)

    print(f"\n{'='*60}")
    print("  All done!")
    print(f"{'='*60}\n")