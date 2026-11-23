import os
import site
import sys
import sysconfig

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

PACKAGE = 'dexhand_description'
_AMENT_MARKER = os.path.join('share', 'ament_index', 'resource_index', 'packages', PACKAGE)


def description_share() -> str:
    try:
        return get_package_share_directory(PACKAGE)
    except PackageNotFoundError:
        pass
    try:
        import dexhand_description

        return dexhand_description.get_share()
    except ImportError:
        return ''


def _pip_prefixes() -> list:
    return [sysconfig.get_path('data'), sys.prefix, site.getuserbase()]


def description_env() -> dict:
    try:
        get_package_share_directory(PACKAGE)
        return {}
    except PackageNotFoundError:
        pass
    for prefix in _pip_prefixes():
        if os.path.exists(os.path.join(prefix, _AMENT_MARKER)):
            paths = [prefix, os.environ.get('AMENT_PREFIX_PATH', '')]
            return {'AMENT_PREFIX_PATH': os.pathsep.join(p for p in paths if p)}
    return {}
