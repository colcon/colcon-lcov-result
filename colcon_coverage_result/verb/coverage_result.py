# Copyright 2018 Apex.AI, Inc.
# Licensed under the Apache License, Version 2.0

from colcon_core.logging import colcon_logger
from colcon_core.plugin_system import satisfies_version
from colcon_core.package_selection import add_arguments \
    as add_packages_arguments
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
        add_packages_arguments(parser)

    def main(self, *, context):
        print("Hello World!")
        return 0
