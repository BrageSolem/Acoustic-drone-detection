import os
import xml.etree.ElementTree as ET
from pathlib import Path

def voc_to_yolo(xml_path, out_label_dir, class_map):
    """Convert a single Pascal VOC XML file to YOLO .txt format."""
    tree = ET.parse(xml_path)
    root = tree.getroot()

    size  = root.find("size")
    img_w = int(size.find("width").text)
    img_h = int(size.find("height").text)

    lines = []
    for obj in root.findall("object"):
        name = obj.find("name").text.strip().lower()
        if name not in class_map:
            continue  # skip unknown classes
        class_id = class_map[name]

        bbox  = obj.find("bndbox")
        xmin  = float(bbox.find("xmin").text)
        ymin  = float(bbox.find("ymin").text)
        xmax  = float(bbox.find("xmax").text)
        ymax  = float(bbox.find("ymax").text)

        # Convert to YOLO normalised format
        cx = (xmin + xmax) / 2 / img_w
        cy = (ymin + ymax) / 2 / img_h
        bw = (xmax - xmin) / img_w
        bh = (ymax - ymin) / img_h

        lines.append(f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

    # Write label file (empty if no valid objects — acts as hard negative)
    stem     = Path(xml_path).stem
    out_path = os.path.join(out_label_dir, stem + ".txt")
    os.makedirs(out_label_dir, exist_ok=True)
    with open(out_path, "w") as f:
        f.write("\n".join(lines))

def convert_dataset(xml_dir, out_label_dir, class_map):
    xml_files = list(Path(xml_dir).glob("*.xml"))
    print(f"Converting {len(xml_files)} XML files...")
    for i, xml_path in enumerate(xml_files):
        voc_to_yolo(xml_path, out_label_dir, class_map)
        if i % 1000 == 0:
            print(f"  {i}/{len(xml_files)}")
    print("Done.")

# ── Run it ──────────────────────────────────────────────────────
CLASS_MAP = {"drone": 0}   # maps class name → YOLO class ID

# Convert train labels
convert_dataset(
    xml_dir       = "DroneDetectDataset/DroneTrainDataset/Drone_TrainSet_XMLs",
    out_label_dir = "drone_xml_converted/train/labels",
    class_map     = CLASS_MAP
)

# Convert test labels
convert_dataset(
    xml_dir       = "DroneDetectDataset/DroneTestDataset/Drone_TestSet_XMLs",
    out_label_dir = "drone_xml_converted/test/labels",
    class_map     = CLASS_MAP
)