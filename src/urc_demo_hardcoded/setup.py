from setuptools import setup

package_name = "urc_demo_hardcoded"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/hardcoded_pick_place.launch.py",
                "launch/complete_gazebo_demo.launch.py",
                "launch/arm_motion_gazebo_demo.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                "config/hardcoded_pick_place.yaml",
                "config/arm_wave_motion.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Suryaansh",
    maintainer_email="suryaansh2112@gmail.com",
    description="Hard-coded pick and place demo scaffolding for Gazebo/MoveIt pipelines.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hardcoded_pick_place = urc_demo_hardcoded.hardcoded_pick_place:main",
            "arm_wave_motion = urc_demo_hardcoded.arm_wave_motion:main",
        ],
    },
)
