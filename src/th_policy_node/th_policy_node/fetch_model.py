"""Fetch the team's Berkeley checkpoint at a fixed revision and verify its bytes."""

import argparse
import hashlib
from pathlib import Path
from urllib.request import urlopen


REVISION = "984741a3623c93b0583ccfdc479f1f8b1c4d900e"
MODEL_SHA256 = "d3f096eda1b2444afce193fed82b9e1fa6d0421cc9ec1d450c549c30163c3a36"
MODEL_URL = (
    "https://raw.githubusercontent.com/Texas-Humanoids/"
    f"berkeley-humanoid-lite/{REVISION}/checkpoints/policy_humanoid.onnx"
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("policies/policy_humanoid.onnx"))
    args = parser.parse_args()
    if args.output.exists():
        if hashlib.sha256(args.output.read_bytes()).hexdigest() != MODEL_SHA256:
            parser.error(f"Refusing to overwrite a different model: {args.output}")
        print(f"Verified existing Berkeley model: {args.output.resolve()}")
        return
    with urlopen(MODEL_URL, timeout=30) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != MODEL_SHA256:
        raise RuntimeError("Downloaded model checksum does not match the pinned checkpoint.")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("xb") as output:
        output.write(data)
    print(f"Downloaded and verified Berkeley model: {args.output.resolve()}")


if __name__ == "__main__":
    main()
