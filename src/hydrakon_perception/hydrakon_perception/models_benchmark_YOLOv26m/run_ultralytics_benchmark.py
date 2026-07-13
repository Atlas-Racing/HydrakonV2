import argparse
import json

from ultralytics.utils.benchmarks import benchmark


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--format", required=True)  # "-" for pytorch, "onnx", "engine"
    ap.add_argument("--device", required=True)  # "0" or "cpu"
    ap.add_argument("--half", action="store_true")
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    df = benchmark(
        model=args.model,
        data=args.data,
        imgsz=args.imgsz,
        half=args.half,
        device=args.device,
        format=args.format,
        verbose=False,
    )

    print(df)
    with open(args.output, "w") as f:
        json.dump(df.to_dicts(), f, indent=2)


if __name__ == "__main__":
    main()
