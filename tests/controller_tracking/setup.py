from glob import glob
from setuptools import setup

setup(name='controller_tracking', version='0.2.0', packages=['controller_tracking'],
      install_requires=['setuptools', 'numpy', 'PyYAML', 'matplotlib'],
      maintainer='zkam', maintainer_email='zkam@example.com', license='Apache-2.0',
      data_files=[('share/ament_index/resource_index/packages', ['resource/controller_tracking']),
                  ('share/controller_tracking', ['package.xml', 'README.md']),
                  ('share/controller_tracking/config', glob('config/*')),
                  ('share/controller_tracking/launch', glob('launch/*.py'))],
      entry_points={'console_scripts': ['tracking_test = controller_tracking.cli:main',
                                        'tracking_feedback = controller_tracking.feedback:main',
                                        'tracking_gate = controller_tracking.gate:main',
                                        'world_calibration = controller_tracking.world_calibration:main']})
