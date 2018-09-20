# Copyright 2018 Apex.AI, Inc.
# Licensed under the Apache License, Version 2.0

import os
import subprocess

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.task import check_call
from colcon_core.task import TaskExtensionPoint
from . import GCOV_EXECUTABLE
from . import LCOV_EXECUTABLE

logger = colcon_logger.getChild(__name__)


class LcovCaptureTask(TaskExtensionPoint):
    """Run lcov on a given package"""
    TASK_NAME='lcov_capture'

    def __init__(self):
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def lcov_capture(self, *, additional_hooks=None):
        pkg = self.context.pkg
        args = self.context.args

        logger.info(
            "Running lcov -c on {pkg} in '{args.build_base}'".format_map(locals())
        )

        if GCOV_EXECUTABLE is None:
            raise RuntimeError("Could not find 'gcov' executable")
        if LCOV_EXECUTABLE is None:
            raise RuntimeError("Could not find 'lcov' executable")

        assert os.path.exists(args.build_base)

        pkg_build_folder = os.path.abspath(os.path.join(args.build_base, pkg.name))
        baseline_file = os.path.join(pkg_build_folder, 'coverage_base.info')
        if args.initial:
            output_file = baseline_file
            additional_args = ['--initial',
                               '--rc', 'lcov_branch_coverage=0']
        else:
            output_file = os.path.join(pkg_build_folder, 'coverage.info')
            additional_args = []

        cmd = [LCOV_EXECUTABLE,
               '--gcov-tool', GCOV_EXECUTABLE,
               '--base-directory', str(pkg.path),
               '--capture',
               '--directory', str(pkg_build_folder),
               '--output-file', str(output_file),
               '--config-file', str(self.context.args.lcov_config_file)]
        cmd.extend(additional_args)

        rc = await check_call(
            self.context,
            cmd,
            cwd=str(pkg.path)
        )

        if rc.returncode == 0 and args.verbose:
            lcov_summary(self.context, output_file)

        if not args.initial and \
           os.path.exists(baseline_file) and \
           os.stat(baseline_file).st_size != 0:
            rc = lcov_add(self.context,
                          [baseline_file, output_file],
                          output_file,
                          verbose=args.verbose)
            # If the output file is empty, then no tests were run, so use baseline
            if rc != 0:
                lcov_add(self.context, [baseline_file], output_file, verbose=args.verbose)
        return


class LcovZeroCountersTask(TaskExtensionPoint):
    """Zero coverage counters"""
    TASK_NAME='lcov_zero'

    def __init__(self):
        super().__init__()
        satisfies_version(TaskExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    async def lcov_zero(self, *, additional_hooks=None):
        pkg = self.context.pkg
        args = self.context.args

        logger.info(
            "Running lcov -z on {pkg} in '{args.build_base}'".format_map(locals())
        )

        if GCOV_EXECUTABLE is None:
            raise RuntimeError("Could not find 'gcov' executable")
        if LCOV_EXECUTABLE is None:
            raise RuntimeError("Could not find 'lcov' executable")

        assert os.path.exists(args.build_base)

        pkg_build_folder = os.path.abspath(os.path.join(args.build_base, pkg.name))
        baseline_file = os.path.join(pkg_build_folder, 'coverage_base.info')

        cmd = [LCOV_EXECUTABLE,
               '--gcov-tool', GCOV_EXECUTABLE,
               '--base-directory', str(pkg.path),
               '--zerocounters',
               '--quiet',
               '--directory', str(pkg_build_folder)]

        await check_call(
            self.context,
            cmd,
            cwd=str(pkg.path)
        )


def lcov_summary(context, tracefile):
    cmd = ['lcov',
           '--gcov-tool', GCOV_EXECUTABLE,
           '--config-file', str(context.args.lcov_config_file),
           '--summary', tracefile]
    rc = subprocess.run(cmd, stderr=subprocess.STDOUT)  # lcov prints to stderr for some reason
    return rc


def lcov_add(context, tracefiles, output_file, verbose=True):
    assert len(tracefiles) > 0
    cmd = ['lcov',
           '--quiet',
           '--gcov-tool', GCOV_EXECUTABLE,
           '--config-file', str(context.args.lcov_config_file)]
    for tracefile in tracefiles:
        cmd.extend(['--add-tracefile', str(tracefile)])
    cmd.extend(['--output-file', str(output_file)])
    rc = subprocess.run(cmd, stderr=subprocess.PIPE)
    if rc.returncode != 0:
        logger.error(rc.stderr.decode('utf-8'))
        return rc.returncode

    if verbose:
        rc = lcov_summary(context, output_file)
        if rc.returncode != 0:
            logger.error(rc.stderr.decode('utf-8'))
        return rc.returncode

    return 0