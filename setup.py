import argparse
import os
import shutil
import subprocess
import sys

from setuptools import Extension
from setuptools import setup, find_packages
from setuptools.command.build_ext import build_ext

with open("README.md", "r") as f:
    long_description = f.read()


_project_dir = os.path.abspath(os.path.dirname(__file__))
_src_python_dir = os.path.join(_project_dir, 'bc_exploration')

parser = argparse.ArgumentParser()
parser.add_argument('--debug', action='store_true')
parser.add_argument('--clean-before-build', action='store_true')
args, unknown = parser.parse_known_args()
sys.argv = [sys.argv[0]] + unknown


class CMakeExtension(Extension):
    def __init__(self, name, source_dir=_project_dir):
        super(CMakeExtension, self).__init__(name, sources=[])
        self.source_dir = os.path.abspath(source_dir)
        self.build_type = 'Debug' if args.debug else 'Release'


class CMakeBuild(build_ext):

    def run(self):
        try:
            subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: "
                f"{', '.join(e.name for e in self.extensions)}"
            )

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext) -> None:
        original_full_path = self.get_ext_fullpath(ext.name)
        ext_dir = os.path.abspath(os.path.dirname(original_full_path))
        ext_dir = os.path.join(ext_dir, self.distribution.get_name())
        cmake_args = [
            f'-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={ext_dir}',
            f'-DPython3_ROOT_DIR={os.path.dirname(sys.executable)}',
            f'-DCMAKE_BUILD_TYPE={ext.build_type}'
        ]
        build_temp = os.path.join(self.build_temp, ext.build_type)
        if args.clean_before_build and os.path.exists(build_temp):
            shutil.rmtree(build_temp)
        os.makedirs(build_temp, exist_ok=True)
        subprocess.check_call(
            ['cmake', ext.source_dir] + cmake_args, cwd=build_temp, env=os.environ
        )
        subprocess.check_call(
            ['cmake', '--build', '.', '--target', ext.name, '--', '-j', f'{os.cpu_count()}'], cwd=build_temp
        )
        subprocess.check_call(
            ['cmake', '--build', '.', '--target', f'{ext.name}', '--', '-j', f'{os.cpu_count()}'], cwd=build_temp
        )


with open('requirements.txt', 'r') as f:
    requires = f.readlines()
dependency_links = []
for i in range(len(requires)):
    require = requires[i]
    if require.strip().startswith('git'):
        # dependency_links.append(require)
        name = require.split('=')[-1].strip()
        version = require.split('#egg')[0].split('@')[1]
        requires[i] = f'{name} @ {require.strip()}'


setup(
    name='bc_exploration',
    version='0.0.1',
    description='Brain Corp Exploration Methods',
    long_description=long_description,
    author='Alexander Khoury',
    author_email='akhoury727@gmail.com',
    url='https://github.com/daizhirui/diff_info_gathering.git',
    download_url='',
    license='Braincorp',
    install_requires=requires,
    package_data={'': ['input', 'params/*'], 'bc_exploration.maps': ['test/*']},
    include_package_data=True,
    extras_require={
        'tests': ['pytest==4.3.0',
                  'pytest-pep8==1.0.6',
                  'pytest-xdist==1.26.1',
                  'pylint==1.9.2',
                  'astroid==1.6.5'
                  ],
    },
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Topic :: Software Development :: Libraries',
        'Topic :: Software Development :: Libraries :: Python Modules'
    ],
    packages=find_packages(os.path.join(_src_python_dir, '..')),
    cmdclass=dict(build_ext=CMakeBuild),
    ext_modules=[CMakeExtension('_exploration_cpp')]
)
