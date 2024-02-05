# Developing colcon-lcov-result

## Developing

See [*Contributing* in the README](./README.rst#contributing).

## Releasing

1. Bump `__version__` in [`colcon_lcov_result/__init__.py`](./colcon_lcov_result/__init__.py)
1. Create commit with the new version number as the commit message
1. Tag that commit with the new version number
1. Push commit and tag
1. Package and publish using [dirk-thomas/publish-python](https://github.com/dirk-thomas/publish-python) (see instructions) to:
    * PyPI: https://pypi.org/project/colcon-lcov-result/
    * packagecloud: https://packagecloud.io/app/dirk-thomas/colcon/search?q=python3-colcon-lcov-result
1. Open a PR to bump version in [ros-infrastructure/reprepro-updater](https://github.com/ros-infrastructure/reprepro-updater) config files so that the ROS apt repos import packages from packagecloud
    * For Debian: https://github.com/ros-infrastructure/reprepro-updater/blob/master/config/colcon.debian.upstream.yaml
    * For Ubuntu: https://github.com/ros-infrastructure/reprepro-updater/blob/master/config/colcon.ubuntu.upstream.yaml
