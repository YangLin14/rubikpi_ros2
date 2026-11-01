#!/usr/bin/env python3
"""Fine-tune YOLO11n on the landmark dataset. Run on the Mac.

    python train.py ../../dataset/yolo_round1/data.yaml --name round1

imgsz matches the deployed ONNX model, so validation numbers reflect the rover.
"""
import argparse
import os

from ultralytics import YOLO


def main():
    p = argparse.ArgumentParser()
    p.add_argument('data')
    p.add_argument('--name', default='round1')
    p.add_argument('--model', default='yolo11n.pt')
    p.add_argument('--imgsz', type=int, default=416)
    p.add_argument('--epochs', type=int, default=120)
    p.add_argument('--batch', type=int, default=16)
    p.add_argument('--device', default='mps')
    args = p.parse_args()

    project = os.path.abspath(os.path.join(os.path.dirname(args.data), '..', 'runs'))
    YOLO(args.model).train(
        data=os.path.abspath(args.data), imgsz=args.imgsz, epochs=args.epochs, batch=args.batch,
        device=args.device, project=project, name=args.name, exist_ok=True, patience=40,
        workers=2, flipud=0.0, plots=True, seed=0, verbose=False)


if __name__ == '__main__':
    main()
