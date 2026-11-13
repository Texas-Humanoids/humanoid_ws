#!/usr/bin/env python3
"""Show th_sim's robot in the MuJoCo viewer on the Mac while the stack runs in Docker.

The sim node streams qpos over UDP when launched with
`sim_stream_to:=host.docker.internal:9870`. On the Mac (outside Docker):

    pip install mujoco
    mjpython tools/walker/sim_viewer.py   # mjpython is required for the viewer on macOS

Display only: the physics runs in the container.
"""

import argparse
import socket

import mujoco
import mujoco.viewer
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument(
        '--model', default='src/th_description/robots/walker_v2/mjcf/scene.xml')
    parser.add_argument('--port', type=int, default=9870)
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(0.05)
    print(f'Listening for sim state on UDP {args.port}')
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            try:
                packet = sock.recv(65536)
            except socket.timeout:
                continue
            qpos = np.frombuffer(packet, dtype=np.float64)
            if qpos.size != model.nq:
                continue
            data.qpos[:] = qpos
            mujoco.mj_forward(model, data)
            viewer.sync()


if __name__ == '__main__':
    main()
