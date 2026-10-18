from setuptools import setup


package_name = "th_policy_integration"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Texas Humanoids Controls",
    maintainer_email="noreply@github.com",
    description="Assemble Berkeley policy observations from ROS robot state.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "observation_builder = th_policy_integration.observation_builder:main",
        ],
    },
)
