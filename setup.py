from setuptools import setup
from pathlib import Path
from glob import glob

# from farmbot_controller import pkg_name

package_name = "farmbot_controller"
installation_pkg_path = Path("share") / Path(package_name)

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # (installation_pkg_path / Path("launch"), glob("launch/*.py")),
        (str(installation_pkg_path), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gregoire",
    maintainer_email="greg.pichereau@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        # "console_scripts": ["controller = farmbot_controller.controller:main"],
        "console_scripts": []
    },
)
