# Copyright 2018 Apex.AI, Inc.
# Licensed under the Apache License, Version 2.0

import os
import shutil
import subprocess
import sys

from pathlib import Path

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.package_selection import get_package_descriptors
from colcon_core.package_selection import select_package_decorators
from colcon_core.topological_order import topological_order_packages
from colcon_core.verb import VerbExtensionPoint

logger = colcon_logger.getChild(__name__)


class CoverageResultVerb(VerbExtensionPoint):
    """
    Collect the lcov results generated from running tests
    """

    def __init__(self):
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):
        parser.add_argument(
            '--build-base',
            default='build',
            help='The base path for all build directories (default: build)'
        )
        parser.add_argument(
            '--lcov-base',
            default='lcov',
            help='The path for lcov artifacts (default: lcov)'
        )
        parser.add_argument(
            '--lcov-config-file',
            default=Path(__file__).parent / 'configuration' / 'lcovrc',
            help='The path to the lcov configuration (default: configuration/lcovrc)'
        )
        add_packages_arguments(parser)

    def main(self, *, context):
        # TODO(jpsamper) Assert that all require programs exist lcov, gcov, c++filt, genhtml,
        # geninfo
        gcc_pkgs = self._get_gcc_packages(context, additional_argument_names=['*'])

        # Define base paths
        build_base_path = Path(os.path.abspath(context.args.build_base))
        if not build_base_path.exists():
            # TODO(jpsamper) Use logger
            print('ERROR: {} does not exist.'.format(str(build_base_path)))
            return 1

        lcov_base_path = Path(os.path.abspath(context.args.lcov_base))
        if not lcov_base_path.exists():
            lcov_base_path.mkdir()

        if isinstance(context.args.base_paths, list):
            base_path = context.args.base_paths[0]
            if len(context.args.base_paths) > 1:
                # TODO(jpsamper) Use logger
                print('WARNING: colcon coverage-result only supports one base path. '
                      'Using only the first element in the list: [{}]'.format(base_path))
        else:
            base_path = context.args.base_paths
        base_path = Path(os.path.abspath(base_path))

        # TODO(jpsamper) Create baseline zero-coverage
        # TODO(jpsamper) Zero coverage counters

        # Iterate over gcc packages - capturing coverage
        # TODO(jpsamper) Parallelize
        coverage_pkgs = []
        for pkg in gcc_pkgs:
            pkg_build_folder = build_base_path / pkg
            cmd = ['lcov',
                   '--quiet',
                   '--base-directory', str(base_path),
                   '--capture',
                   '--directory', str(pkg_build_folder),
                   '--output-file', str(pkg_build_folder / 'coverage.info'),
                   '--config-file', str(context.args.lcov_config_file)
            ]
            rc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if not "geninfo: WARNING: no .gcda files found" in rc.stderr.decode('utf_8'):
                coverage_pkgs.append(pkg_build_folder)
            else:
                print(rc.stderr.decode('utf_8'), file=sys.stderr)

        if len(coverage_pkgs) == 0:
            # TODO(jpsamper) Use logger
            print("ERROR: No coverage files found. Did you build with the correct flags? Did "
                  "you run tests?")
            return 1

        # Aggregate lcov outputs
        output_file = str(lcov_base_path / 'total_coverage.info')
        if len(coverage_pkgs) == 1:
            shutil.copyfile(str(coverage_pkgs[0] / 'coverage.info'), output_file)
        else:
            cmd = ['lcov',
                   '--quiet',
                   '--config-file', str(context.args.lcov_config_file)]
            for pkg_coverage in coverage_pkgs:
                cmd.extend(['--add-tracefile', str(pkg_coverage / 'coverage.info')])
            cmd.extend(['--output-file', output_file])
            subprocess.run(cmd)

        # TODO(jpsamper) Add step to filter unwanted files

        # Generate html
        cmd = ['genhtml',
               '--output-directory', str(lcov_base_path),
               output_file,
               '--demangle-cpp',
               '--config-file', str(context.args.lcov_config_file),
               '--prefix', str(base_path)]
        return subprocess.run(cmd).returncode


    def _get_gcc_packages(self, context, additional_argument_names=None):
        descriptors = get_package_descriptors(
            context.args, additional_argument_names=additional_argument_names
        )
        descriptors = get_package_descriptors(context.args)

        # always perform topological order for the select package extensions
        decorators = topological_order_packages(
            descriptors, recursive_categories=('run', ))

        select_package_decorators(context.args, decorators)

        gcc_pkgs = []
        for decorator in decorators:
            if not decorator.selected:
                continue
            pkg = decorator.descriptor
            if pkg.type in ['ros.ament_cmake', 'ros.cmake', 'cmake']:
                gcc_pkgs.append(pkg.name)
            else:
                print("WARNING: Specified package {} is not a gcc package. Not collecting "
                      "coverage".format(pkg.name))  # TODO(jpsamper) Use logger
        return gcc_pkgs
