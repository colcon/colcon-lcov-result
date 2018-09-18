# Copyright 2018 Apex.AI, Inc.
# Licensed under the Apache License, Version 2.0

import os
import re
import subprocess

from pathlib import Path

from colcon_core.command import add_log_level_argument
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.package_selection import get_package_descriptors
from colcon_core.package_selection import select_package_decorators
from colcon_core.topological_order import topological_order_packages
from colcon_core.verb import VerbExtensionPoint
from . import CPP_FILT_EXECUTABLE
from . import GCOV_EXECUTABLE
from . import GENHTML_EXECUTABLE
from . import LCOV_EXECUTABLE

logger = colcon_logger.getChild(__name__)


class LcovResultVerb(VerbExtensionPoint):
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
        parser.add_argument(
            '--verbose',
            action='store_true',
            help='Show coverage rates for individual packages'
        )
        add_packages_arguments(parser)
        add_log_level_argument(parser)

    def main(self, *, context):
        # Check that all required programs exist
        if GCOV_EXECUTABLE is None:
            raise RuntimeError("Could not find 'gcov' executable")
        if LCOV_EXECUTABLE is None:
            raise RuntimeError("Could not find 'lcov' executable")
        if GENHTML_EXECUTABLE is None:
            raise RuntimeError("Could not find 'genhtml' executable")

        # Gather absolute paths
        base_abspaths = {
            'build': Path(os.path.abspath(context.args.build_base)),
            'lcov': Path(os.path.abspath(context.args.lcov_base))}

        assert base_abspaths['build'].exists()
        base_abspaths['lcov'].mkdir(exist_ok=True)

        gcc_pkgs = _get_gcc_packages(context, additional_argument_names=['*'])

        # TODO(jpsamper) Create baseline zero-coverage
        # TODO(jpsamper) Zero coverage counters

        # Iterate over gcc packages - capturing coverage
        # TODO(jpsamper) Parallelize
        output_files = []
        for pkg in gcc_pkgs:
            output_file = lcov_capture(
                context,
                base_abspaths,
                pkg
                )
            if output_file:
                output_files.append(output_file)

        if len(output_files) == 0:
            logger.error("No coverage files found. Did you build with the correct flags? "
                         "Did you run tests?")
            return 1

        # Aggregate lcov outputs
        total_output_file = str(base_abspaths['lcov'] / 'total_coverage.info')
        lcov_add(context, output_files, total_output_file)

        # TODO(jpsamper) Add step to filter unwanted files

        # Generate html
        cmd = [GENHTML_EXECUTABLE,
               '--quiet',
               '--output-directory', str(base_abspaths['lcov']),
               total_output_file,
               '--config-file', str(context.args.lcov_config_file)]
        if CPP_FILT_EXECUTABLE is not None:
            cmd.extend(['--demangle-cpp'])
        # Strip paths to packages
        for path in context.args.base_paths:
            cmd.extend(['--prefix', str(os.path.abspath(path))])
        return subprocess.run(cmd).returncode


def _get_gcc_packages(context, additional_argument_names=None):
    descriptors = get_package_descriptors(
        context.args, additional_argument_names=additional_argument_names
    )

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
            gcc_pkgs.append(pkg)
        else:
            logger.info("Specified package {} is not a gcc package. Not "
                        "collecting coverage".format(pkg.name))
    return gcc_pkgs


def lcov_capture(context, base_abspaths, package):
    pkg_build_folder = base_abspaths['build'] / package.name
    output_file = pkg_build_folder / 'coverage.info'
    cmd = [LCOV_EXECUTABLE,
           '--quiet',
           '--gcov-tool', GCOV_EXECUTABLE,
           '--base-directory', str(package.path),
           '--capture',
           '--directory', str(pkg_build_folder),
           '--output-file', str(output_file),
           '--config-file', str(context.args.lcov_config_file)
           ]
    rc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    # No stdout because of --quiet flag
    logger.info(rc.stderr.decode('utf_8'))
    if "geninfo: WARNING: no .gcda files found" not in rc.stderr.decode('utf_8'):
        if context.args.verbose:
            lcov_add(context, [output_file], os.devnull, descriptor=package.name)
        return output_file
    else:
        return None


def lcov_add(context, tracefiles, output_file, descriptor='Total'):
    cmd = ['lcov',
           '--gcov-tool', GCOV_EXECUTABLE,
           '--config-file', str(context.args.lcov_config_file)]
    for tracefile in tracefiles:
        cmd.extend(['--add-tracefile', str(tracefile)])
    cmd.extend(['--output-file', str(output_file)])
    rc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if rc.returncode == 0:
        # Parse output for:
        #   <type>...: <pct>% (<x> of <total> <type>)
        matches = re.findall('\s*([a-z]+)\.*: ([0-9.]+)% \((\d+) of (\d+) .*\)',
                             rc.stdout.decode('utf-8'))
        coverage_results = []
        for match in matches:
            coverage_results.append(CoverageResult(cov_type=match[0],
                                                   percent=match[1],
                                                   covered=match[2],
                                                   total=match[3]))
        print("{} coverage rates:".format(descriptor))
        for result in coverage_results:
            print("  {}".format(result))
    else:
        logger.error(rc.stderr.decode('utf-8'))


class CoverageResult:
    def __init__(self, cov_type='None', percent=0.0, covered=0, total=0):
        self.cov_type = cov_type
        self.padded_type = cov_type.ljust(10, '.')
        self.percent = percent
        self.covered = covered
        self.total = total

    def __repr__(self):
        return \
            '{0.padded_type}: {0.percent}% ({0.covered} of {0.total} {0.cov_type})'.format(self)
