import argparse
import glob
import json
import os
import random
import time

from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", required=True, choices=["pt", "onnx", "engine"])
    ap.add_argument("--model", required=True)
    ap.add_argument("--images-dir", required=True)
    ap.add_argument("--n", type=int, default=150)
    ap.add_argument("--warmup", type=int, default=20)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    device = "cpu" if args.backend == "onnx" else 0
    half = args.backend == "engine"

    all_images = sorted(glob.glob(os.path.join(args.images_dir, "*.jpg")))
    rng = random.Random(args.seed)
    sample = rng.sample(all_images, min(args.n, len(all_images)))

    torch = None
    if device != "cpu":
        import torch
        torch.cuda.reset_peak_memory_stats()

    model = YOLO(args.model, task="detect")

    warmup_imgs = (sample * ((args.warmup // len(sample)) + 1))[: args.warmup]
    for img_path in warmup_imgs:
        model.predict(img_path, imgsz=args.imgsz, device=device, half=half,
                       verbose=False, save=False)

    records = []
    wall_start = time.perf_counter()
    for img_path in sample:
        t0 = time.perf_counter()
        results = model.predict(img_path, imgsz=args.imgsz, device=device, half=half,
                                 verbose=False, save=False)
        t1 = time.perf_counter()
        r = results[0]
        records.append({
            "image": os.path.basename(img_path),
            "preprocess_ms": r.speed["preprocess"],
            "inference_ms": r.speed["inference"],
            "postprocess_ms": r.speed["postprocess"],
            "wall_ms": (t1 - t0) * 1000.0,
            "n_detections": len(r.boxes) if r.boxes is not None else 0,
        })
    wall_total_s = time.perf_counter() - wall_start

    peak_gpu_mem_mb = None
    if device != "cpu":
        peak_gpu_mem_mb = torch.cuda.max_memory_allocated() / (1024 * 1024)

    out = {
        "backend": args.backend,
        "model_path": args.model,
        "device": "cpu" if device == "cpu" else "cuda:0",
        "half": half,
        "imgsz": args.imgsz,
        "n_images": len(sample),
        "warmup": args.warmup,
        "file_size_mb": os.path.getsize(args.model) / (1024 * 1024),
        "peak_gpu_mem_mb": peak_gpu_mem_mb,
        "wall_total_s": wall_total_s,
        "records": records,
    }

    with open(args.output, "w") as f:
        json.dump(out, f, indent=2)

    print(f"[{args.backend}] done: {len(sample)} images, "
          f"{len(sample) / wall_total_s:.2f} FPS (wall), "
          f"file_size={out['file_size_mb']:.1f}MB, "
          f"peak_gpu_mem={peak_gpu_mem_mb}")


if __name__ == "__main__":
    main()
