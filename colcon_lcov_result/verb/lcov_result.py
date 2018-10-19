# Copyright 2018 Apex.AI, Inc.
# Licensed under the Apache License, Version 2.0

import os
import subprocess

from collections import OrderedDict
from pathlib import Path

from colcon_core.command import add_log_level_argument
from colcon_core.event_handler import add_event_handler_arguments
from colcon_core.executor import add_executor_arguments
from colcon_core.executor import execute_jobs
from colcon_core.executor import Job
from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
from colcon_core.package_selection import get_package_descriptors
from colcon_core.package_selection import select_package_decorators
from colcon_core.task import TaskContext
from colcon_core.topological_order import topological_order_packages
from colcon_core.verb import VerbExtensionPoint
from colcon_core.verb import check_and_mark_build_tool
from ..task.lcov import LcovCaptureTask
from ..task.lcov import LcovZeroCountersTask
from ..task.lcov import lcov_add
from ..task.lcov import lcov_remove
from . import CPP_FILT_EXECUTABLE
from . import GENHTML_EXECUTABLE

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
        parser.add_argument(
            '--initial',
            action='store_true',
            help='Generate baseline (zero) coverage. This option is meant to be used before'
                 'running `colcon test`'
        )
        parser.add_argument(
            '--zero-counters',
            action='store_true',
            help='Zero the coverage counters'
        )
        parser.add_argument(
            '--filter',
            nargs='*',
            help='Remove files matching FILTER from total coverage (e.g. "*/test/*")'
        )
        add_packages_arguments(parser)
        add_log_level_argument(parser)
        add_executor_arguments(parser)
        add_event_handler_arguments(parser)

    def main(self, *, context):
        check_and_mark_build_tool(context.args.build_base)

        lcov_base_abspath = Path(os.path.abspath(context.args.lcov_base))
        lcov_base_abspath.mkdir(exist_ok=True)

        gcc_pkgs = self._get_gcc_packages(context, additional_argument_names=['*'])

        jobs = OrderedDict()
        for pkg in gcc_pkgs:
            task_context = TaskContext(
                pkg=pkg,
                args=context.args,
                dependencies=OrderedDict()
            )

            if context.args.zero_counters:
                extension = LcovZeroCountersTask()
            else:
                extension = LcovCaptureTask()
            extension.PACKAGE_TYPE = pkg.type

            job = Job(
                identifier=pkg.name,
                dependencies=set(),  # Can be generated in any order
                task=extension,
                task_context=task_context
            )
            jobs[pkg.name] = job

        rc = execute_jobs(context, jobs)

        if context.args.initial or context.args.zero_counters:
            return rc

        print("\nCalculating total coverage... ")
        total_output_file = str(lcov_base_abspath / 'total_coverage.info')
        if rc == 0:
            output_files = []
            for pkg in gcc_pkgs:
                output_file = os.path.abspath(
                    os.path.join(context.args.build_base, pkg.name, 'coverage.info'))
                if os.stat(output_file).st_size != 0:
                    output_files.append(output_file)
            if len(output_files) == 0:
                logger.error('No valid coverage.info files found. Did you run tests?')
                return 1
            rc = lcov_add(context, output_files, total_output_file, verbose=context.args.verbose)

        if rc != 0:
            return rc

        if context.args.filter:
            print("\nApplying filters... ")
            rc = lcov_remove(context, total_output_file)

        if rc != 0:
            return rc

        print("\nGenerating HTML: ", end='')
        # Check that genhtml exists
        if GENHTML_EXECUTABLE is None:
            raise RuntimeError("Could not find 'genhtml' executable")

        # Generate html
        cmd = [GENHTML_EXECUTABLE,
               '--quiet',
               '--output-directory', str(lcov_base_abspath),
               total_output_file,
               '--config-file', str(context.args.lcov_config_file)]
        if CPP_FILT_EXECUTABLE is not None:
            cmd.extend(['--demangle-cpp'])
        # Strip paths to packages
        for path in context.args.base_paths:
            cmd.extend(['--prefix', str(os.path.abspath(path))])
        rc = subprocess.run(cmd).returncode
        print("Done")
        return rc

    @staticmethod
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
