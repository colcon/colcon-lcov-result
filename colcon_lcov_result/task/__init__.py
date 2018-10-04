# Copyright 2018 Apex.AI, Inc.
# Licensed under the Apache License, Version 2.0

import os
import shutil

from colcon_core.environment_variable import EnvironmentVariable

"""Environment variable to override the gcov executable"""
GCOV_COMMAND_ENVIRONMENT_VARIABLE = EnvironmentVariable(
    'GCOV_COMMAND', 'The full path to the gcov executable')
"""Environment variable to override the lcov executable"""
LCOV_COMMAND_ENVIRONMENT_VARIABLE = EnvironmentVariable(
    'LCOV_COMMAND', 'The full path to the lcov executable')


def which_executable(environment_variable, executable_name):
    """
    Determine the path of an executable.

    An environment variable can be used to override the location instead of
    relying on searching the PATH.
    :param str environment_variable: The name of the environment variable
    :param str executable_name: The name of the executable
    :rtype: str
    """
    value = os.getenv(environment_variable)
    if value:
        return value
    return shutil.which(executable_name)


GCOV_EXECUTABLE = which_executable(GCOV_COMMAND_ENVIRONMENT_VARIABLE.name, 'gcov')
LCOV_EXECUTABLE = which_executable(LCOV_COMMAND_ENVIRONMENT_VARIABLE.name, 'lcov')
