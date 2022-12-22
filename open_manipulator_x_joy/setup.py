from setuptools import setup

package_name = "open_manipulator_x_joy"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="husarion",
    maintainer_email="maciej.stepien@husarion.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_servo = open_manipulator_x_joy.joy_servo:main",
        ],
    },
)
