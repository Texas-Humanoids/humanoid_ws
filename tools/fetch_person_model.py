"""Fetch the RTMO pose model (OpenMMLab, Apache-2.0) used by th_autonomy's person_follower.

Downloads OpenMMLab's ONNX export, checks it against a pinned checksum and
saves it as models/rtmo-<size>.onnx. Run from the workspace root:
  python3 tools/fetch_person_model.py            # rtmo-l, the default (23 ms on the Orin Nano GPU)
  python3 tools/fetch_person_model.py --size m   # 16 ms; s: 9 ms
"""

import argparse
import hashlib
import io
from pathlib import Path
from urllib.request import urlopen
import zipfile

BASE = 'https://download.openmmlab.com/mmpose/v1/projects/rtmo/onnx_sdk/'
MODELS = {  # size: (zip, sha256 of the zip), all 640x640 trained on body7
    's': ('rtmo-s_8xb32-600e_body7-640x640-dac2bf74_20231211.zip',
          '3da7ad88b209f9da8be87ba5c325610639af04de3b5c8a96649b98cd9e2848a2'),
    'm': ('rtmo-m_16xb16-600e_body7-640x640-39e78cc4_20231211.zip',
          '6b9d3be1323cc030444f731bbe593ea61f59b9481e23da2f8b60cb4b72136e16'),
    'l': ('rtmo-l_16xb16-600e_body7-640x640-b37118ce_20231211.zip',
          '17b361174d759d974879f9fb46d564ae658d004bfa070e6f1c9ad275d3fd6b87'),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--size', choices=sorted(MODELS), default='l')
    parser.add_argument('--output-dir', type=Path, default=Path('models'))
    args = parser.parse_args()
    output = args.output_dir / f'rtmo-{args.size}.onnx'
    if output.exists():
        print(f'Already have {output.resolve()}')
        return
    name, sha256 = MODELS[args.size]
    print(f'Downloading {name} ...')
    with urlopen(BASE + name, timeout=120) as response:
        data = response.read()
    if hashlib.sha256(data).hexdigest() != sha256:
        raise RuntimeError(f'{name} does not match its pinned checksum')
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        model = next(n for n in archive.namelist() if n.endswith('end2end.onnx'))
        onnx = archive.read(model)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as f:
        f.write(onnx)
    print(f'Saved {output.resolve()}')


if __name__ == '__main__':
    main()
