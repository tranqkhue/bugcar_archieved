## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
import setuptools
# fetch values from package.xml
setup_args = generate_distutils_setup(
	package_dir={'': 'src'},
	install_requires=[
	'polyline==1.4.0',
	'utm == 0.5.0',
	'request>=2.18.4',
	'numpy'
	]
)
setup(**setup_args)
