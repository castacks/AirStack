from glob import glob

from setuptools import find_packages, setup

package_name = 'search_baselines'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # The offboard-compute container's start script. Lives in the package
        # because it starts THIS package's servers; the compose service runs it
        # from the mounted source tree so it can be edited without a rebuild.
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Semantic-search baselines: one shared planner, one launch file per method',
    license='TODO',
    entry_points={
        'console_scripts': [
            # The shared BLIP-2 ITM scoring service. One instance serves every
            # robot and every method that needs image-text matching.
            # The planner every baseline runs. One node, one map, one
            # actuation path; the method is a parameter.
            # The planner runs under SYSTEM python: it is an ordinary ROS node
            # and needs rclpy and cv_bridge, which the venv does not have.
            'search_planner = search_baselines.planner_node:main',
            # The shared BLIP-2 ITM scoring service (VLFM and anything else
            # needing image-text matching).
            # The two SERVERS go through _venv_exec: `ros2 run` uses system
            # python, which lacks fastapi/uvicorn/bitsandbytes/accelerate, so
            # these hand off to /opt/lvlm-venv rather than failing on import.
            'itm_server = search_baselines._venv_exec:itm_server',
            # The generative OpenAI-compatible VLM server (Co-NavGPT2).
            'vlm_server = search_baselines._venv_exec:vlm_server',
            # YOLO + MobileSAM as a shared service. One instance for the whole
            # fleet, on the offboard-compute container: detection is stateless,
            # so interleaving robots is safe, and three planners loading their
            # own copy of both models is three times the VRAM for no benefit.
            'detector_server = search_baselines._venv_exec:detector_server',
        ],
    },
)
