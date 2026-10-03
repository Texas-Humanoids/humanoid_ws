from setuptools import find_packages, setup


package_name = "th_policy_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/policy_humanoid.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Texas Humanoids Controls",
    maintainer_email="noreply@github.com",
    description="Runs an ONNX walking policy on policy-ready ROS observations.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "policy_node = th_policy_node.policy_node:main",
        ],
    },
)
