"""Shared helpers for the WalkerV2 hardware tools: config loading and motor topics."""

import json

import yaml


def load_robot(path):
    with open(path) as f:
        robot = yaml.safe_load(f) or {}
    return robot, robot.get('motors') or []


def load_defaults(spec_path):
    with open(spec_path) as f:
        spec = json.load(f)
    return dict(zip(spec['joint_order'], spec['default_joint_pos']))


def namespace(joint):
    return '/' + joint.removesuffix('_joint')
