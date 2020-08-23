from setuptools import setup, find_packages

setup(
    name='ros_np_tools',
    version='0.0.1',
    description='Various convenience functions for converting from ros types to other ros types, ros types to'
                'numpy types, and holding ros types in convenience classes. Requires a ros installation for access'
                'to typical ros tools (rospy, tf2_ros, etc.).',
    author='Trevor Ablett',
    author_email='trevor.ablett@robotics.utias.utoronto.ca',
    license='MIT',
    packages=find_packages(),
    install_requires=['numpy'],  # in case we're installing in a virtualenv
    include_package_data=True
)
