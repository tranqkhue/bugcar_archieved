# How to create a ROS Python package?

## Initialize a package

Run `catkin_create_pkg <pkg_name> <depend_1, depend_2...>`

It will initialize a package with a name, dependencies and a `src` folder

## Add Python scripts

Add Python scripts inside `src` folder

Caution: add Shebang at the top of Python scripts
```
#!/usr/bin/env python2
# -*- coding: utf-8 -*-
```

## Create an empty `__init__.py` in `src` folder

## Uncomment `catkin_python_setup()` in `CMakeLists.txt`

## Add `setup.py`

The `setup.py` should be in the package's root directory (not in package `src` folder, for example)

```
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
# fetch values from package.xml
setup_args = generate_distutils_setup(
    package_dir={'': 'src'},
    install_requires=[
		<pip_pkg1>,
		<pip_pkg2>
	]
)
setup(**setup_args)
```

## Run catkin_make to make sure everything works
## Make the src Python scripts executable
