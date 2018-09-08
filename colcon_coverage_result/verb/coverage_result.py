# Copyright 2018 Apex.AI, Inc.
# Licensed under the Apache License, Version 2.0

import os
import shutil
import subprocess

from pathlib import Path

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.package_selection import get_package_descriptors
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
        add_packages_arguments(parser)

    def main(self, *, context):
        descriptors = get_package_descriptors(
            context.args, additional_argument_names=['*']
        )
        lcov_base_path = Path(os.path.abspath(context.args.lcov_base))
        coverage_pkgs = []
        for pkg in descriptors:
            if pkg.type in ['ros.ament_cmake', 'ros.cmake', 'cmake']:
                build_base_path = Path(os.path.abspath(context.args.build_base))
                pkg_build_folder = build_base_path / pkg.name
                cmd = ['lcov',
                       '--base-directory', str(os.path.abspath(context.args.base_paths)),
                       '--capture',
                       '--directory', str(pkg_build_folder),
                       '--output-file', str(pkg_build_folder / 'coverage'),
                       '--no-external',
                       '--rc', 'lcov_branch_coverage=1'
                ]
                rc = subprocess.run(cmd)
                if rc.returncode == 0:
                    coverage_pkgs.append(pkg_build_folder)
        output_file = str(lcov_base_path / 'total_coverage')
        if len(coverage_pkgs) == 0:
            print("No coverage files found. Did you build with the correct flags?")
        elif len(coverage_pkgs) == 1:
            shutil.copyfile(str(coverage_pkgs[0] / 'coverage'), output_file)
        else:
            cmd = ['lcov',
                   '--rc', 'lcov_branch_coverage=1']
            for pkg_coverage in coverage_pkgs:
                cmd.extend(['--add-tracefile', str(pkg_coverage / 'coverage')])
            cmd.extend(['--output-file', output_file])
            rc = subprocess.run(cmd)
            if rc.returncode != 0:
                return rc.returncode
        cmd = ['genhtml',
               '--output-directory', str(lcov_base_path),
               output_file,
               '--legend',
               '--demangle-cpp',
               '--rc', 'genhtml_branch_coverage=1',
               '--prefix', os.path.abspath(context.args.base_paths)]
        return subprocess.run(cmd)
