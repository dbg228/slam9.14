from glob import glob
from setuptools import setup

setup(name='mocap_anchored_mapping', version='0.2.0', packages=['mocap_anchored_mapping'],
      install_requires=['setuptools', 'numpy', 'PyYAML', 'scipy'],
      maintainer='zkam', maintainer_email='zkam@example.com', license='Apache-2.0',
      data_files=[('share/ament_index/resource_index/packages', ['resource/mocap_anchored_mapping']),
                  ('share/mocap_anchored_mapping', ['package.xml', 'README.md']),
                  ('share/mocap_anchored_mapping/config', glob('config/*.yaml')),
                  ('share/mocap_anchored_mapping/launch', glob('launch/*.py'))],
      entry_points={'console_scripts': ['anchored_map = mocap_anchored_mapping.cli:main',
                                        'initial_anchor = mocap_anchored_mapping.online:main']})
