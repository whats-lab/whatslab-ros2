"""자산 경로 — launch 파일에서도 쓰므로 무거운 import 를 하지 않는다."""

import os
import site
import sys
import sysconfig

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

PACKAGE = 'dexhand_description'
_AMENT_MARKER = os.path.join('share', 'ament_index', 'resource_index', 'packages', PACKAGE)


def description_share() -> str:
    """dexhand_description 자산 루트 — ament 패키지(colcon) 우선, 없으면 pip 패키지."""
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
    """rviz 가 `package://dexhand_description/...` 메쉬를 찾게 하는 추가 환경변수.

    pip 설치는 ament 색인을 `<prefix>/share/ament_index` 에 깔지만 그 prefix 가
    AMENT_PREFIX_PATH 에 없다. ament 로 이미 찾히거나 pip 설치가 없으면 {}.
    """
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
