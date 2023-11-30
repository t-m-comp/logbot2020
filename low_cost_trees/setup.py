import os

import numpy
from setuptools import setup, find_packages
from setuptools.extension import Extension
from Cython.Build import cythonize

libraries = []
if os.name == 'posix':
    libraries.append('m')

extensions = [
    Extension("low_cost_trees._tree",
              ["low_cost_trees/_tree.pyx"],
              include_dirs=[numpy.get_include()],
              libraries=libraries),
    Extension("low_cost_trees._splitter",
              ["low_cost_trees/_splitter.pyx"],
              include_dirs=[numpy.get_include()],
              libraries=libraries),
    Extension("low_cost_trees._criterion",
              ["low_cost_trees/_criterion.pyx"],
              include_dirs=[numpy.get_include()],
              libraries=libraries),
    Extension("low_cost_trees._utils",
              ["low_cost_trees/_utils.pyx"],
              include_dirs=[numpy.get_include()],
              libraries=libraries),
    Extension("low_cost_trees.quad_tree",
              ["low_cost_trees/quad_tree.pyx"],
              include_dirs=[numpy.get_include()],
              libraries=libraries),
]

LICENSE = 'new BSD'
INSTALL_REQUIRES = ['numpy', 'scipy', 'scikit-learn']
CLASSIFIERS = ['Intended Audience :: Science/Research',
               'Intended Audience :: Developers',
               'License :: OSI Approved',
               'Programming Language :: Python',
               'Topic :: Software Development',
               'Topic :: Scientific/Engineering',
               'Operating System :: Unix',
               'Programming Language :: Python :: 3.7']

setup(
    name="low_cost_trees",
    version="0.0.1",
    license=LICENSE,
    classifiers=CLASSIFIERS,
    install_requires=INSTALL_REQUIRES,
    packages = find_packages(),
    ext_modules = cythonize(extensions)
)