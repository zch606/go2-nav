from setuptools import find_packages, setup

package_name = 'dog_nav_step56'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/step56.launch.py',
            'launch/closed_loop_demo.launch.py',
            'launch/step56_test.launch.py',
            'launch/unitree_bridge.launch.py',
            'launch/stereo_terrain.launch.py',
            'launch/simulated_stereo_demo.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/motion_adapter.yaml',
            'config/safety_supervisor.yaml',
            'config/unitree_bridge.yaml',
            'config/stereo_terrain.yaml',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/terrain_view.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GitHub Copilot',
    maintainer_email='copilot@example.com',
    description='Step 5-6 implementation for dog navigation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'local_controller_node = dog_nav_step56.local_controller_node:main',
            'motion_adapter_node = dog_nav_step56.motion_adapter_node:main',
            'safety_supervisor_node = dog_nav_step56.safety_supervisor_node:main',
            'simulated_input_node = dog_nav_step56.simulated_input_node:main',
            'test_trace_recorder_node = dog_nav_step56.test.test_trace_recorder_node:main',
            'unitree_bridge_node = dog_nav_step56.unitree_bridge_node:main',
            'stereo_terrain_node = dog_nav_step56.stereo_terrain_node:main',
            'simulated_camera_node = dog_nav_step56.test.simulated_camera_node:main',
        ],
    },
)
