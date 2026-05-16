from ultralytics import YOLO
import torch

if __name__ == '__main__':
    model = YOLO("yolov8n.pt")

    print(torch.cuda.is_available())
    print(torch.cuda.get_device_name(0))

    model.train(
        data="drone_xml_converted/data.yaml",

        # ── Epochs & convergence ────────────────────────────────
        epochs=100,             # ↑ from 50 — mAP50 was still climbing
                                #   at ep 33, hasn't plateaued yet
        patience=20,            # ↑ from 15 — oscillation is wider on
                                #   this harder dataset, needs more room
        fraction=1.0,           # ↑ from 0.5 — use full dataset since
                                #   we're ignoring time constraints

        # ── Image size ─────────────────────────────────────────
        imgsz=640,              # ↑ from 416 — this dataset has small
                                #   drones in cluttered backgrounds,
                                #   higher resolution helps significantly

        batch=16,               # ↓ from 32 — needed because 640 uses
                                #   more VRAM than 416; drop to 8 if OOM

        # ── Augmentation ───────────────────────────────────────
        mosaic=1.0,             # ↑ from 0.5 — this dataset has varied
                                #   backgrounds, mosaic helps more here
                                #   than on the sky-heavy merged set
        mixup=0.1,              # ↑ from 0.0 — small amount helps with
                                #   the harder background variation
        degrees=15.0,           # ↑ from 0.0 — drones appear at varied
                                #   angles in real footage
        scale=0.8,              # add — dataset has drones at many sizes,
                                #   scale jitter improves size robustness
        flipud=0.1,             # add — small benefit for aerial shots
        fliplr=0.5,             # keep
        hsv_h=0.015,            # keep
        hsv_s=0.7,              # keep
        hsv_v=0.4,              # keep
        translate=0.1,          # keep
        close_mosaic=15,        # ↑ from 10 — proportional to 100 epochs

        # ── Optimizer ──────────────────────────────────────────
        optimizer="AdamW",
        lr0=0.001,              # ↓ from 0.002 — lower LR works better
                                #   with full dataset at 640, more stable
        lrf=0.005,              # ↓ from 0.01 — decay further for finer
                                #   convergence in later epochs
        warmup_epochs=5,        # ↑ from default 3 — larger imgsz and
                                #   full dataset benefits from longer warmup
        cos_lr=True,            # keep
        weight_decay=0.0005,    # keep

        # ── Loss weights ───────────────────────────────────────
        box=9.0,                # keep — already tuned for tight boxes
        cls=0.5,                # keep

        # ── Practical ──────────────────────────────────────────
        rect=True,              # keep — still useful at 640
        cache="disk",           # keep
        workers=8,              # keep
        device=0,
        plots=True,
        val=True,
        save_period=10,
        name="drone_xml_model_full"
    )

    model.export(
        format="onnx",
        imgsz=640,
        opset=12,
        simplify=True,
        dynamic=False,
    )