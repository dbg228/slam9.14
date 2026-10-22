from setuptools import setup

setup(
    name='localization_ab_test', version='0.1.0',
    packages=['localization_ab_test'],
    # Reuse the project's already tested geometry and trajectory evaluator.
    package_dir={'localization_ab_test': 'localization_ab_test', '': '../..'},
    py_modules=['ndt_match_evaluation_core', 'fastlio_mocap_trajectory_evaluation_core'],
    data_files=[('share/localization_ab_test/config', ['config/ndt_initialization.rviz','config/fastdds_udp_only.xml']),
                ('share/ament_index/resource_index/packages', ['resource/localization_ab_test']),
                ('share/localization_ab_test', ['package.xml', 'README.md'])],
    install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=False,
    maintainer='zkam', maintainer_email='maintainer@example.com', description='Distance gated localization experiments',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'motion_gate = localization_ab_test.nodes:motion_main',
        'velocity_guard = localization_ab_test.nodes:guard_main',
        'pose_sources = localization_ab_test.sources:main',
        'run_trial = localization_ab_test.run:main',
        'run_three_trials = localization_ab_test.series:main',
        'compare_trials = localization_ab_test.comparison:main',
        'evaluate = localization_ab_test.evaluate:main',
        'prepare_chassis = localization_ab_test.prepare_chassis:main',
    ]},
)
