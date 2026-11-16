"""Fetch the YOLOX-Nano person detector (Apache-2.0) and verify its bytes.

Used by th_autonomy's person_follower. Run from the workspace root:
  python3 tools/fetch_person_model.py
"""

import argparse
import hashlib
from pathlib import Path
from urllib.request import urlopen

MODEL_SHA256 = "c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d"
MODEL_URL = (
    "https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_nano.onnx"
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("models/yolox_nano.onnx"))
    args = parser.parse_args()
    if args.output.exists():
        if hashlib.sha256(args.output.read_bytes()).hexdigest() != MODEL_SHA256:
            parser.error(f"Refusing to overwrite a different model: {args.output}")
        print(f"Verified existing person model: {args.output.resolve()}")
        return
    with urlopen(MODEL_URL, timeout=60) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != MODEL_SHA256:
        raise RuntimeError("Downloaded model checksum does not match the pinned YOLOX-Nano release.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as output:
        output.write(data)
    print(f"Downloaded and verified person model: {args.output.resolve()}")


if __name__ == "__main__":
    main()
