#cython: language_level=3


import os
from setuptools import setup
from Cython.Build import cythonize


setup(
    name='mapping_c_util',
    ext_modules=cythonize("_mapping.pyx",force=True,language_level=3),
    
)