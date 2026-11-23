import os
import re
import site
import sys
import sysconfig
import xml.etree.ElementTree as ET
from glob import glob

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

PACKAGE = 'dexhand_description'
_AMENT_MARKER = os.path.join('share', 'ament_index', 'resource_index', 'packages', PACKAGE)
_MESH_FILENAME = re.compile(r'(<mesh\b[^>]*?\bfilename=)(["\'])(.*?)\2')


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
    prefixes = [sysconfig.get_path('data'), sys.prefix, site.getuserbase()]
    share = description_share()
    return prefixes + [os.path.dirname(os.path.dirname(share))] if share else prefixes


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


def find_hand_urdf(share: str, model: str, side: str) -> str:
    folder = os.path.join(share, model)
    names = (f'{model}_{side}.urdf', f'{model}.urdf', os.path.join('urdf', f'{side}.urdf'))
    for path in (os.path.join(folder, name) for name in names):
        if os.path.isfile(path):
            return path
    sided = sorted(glob(os.path.join(folder, f'*{side}.urdf')))
    sided += sorted(glob(os.path.join(folder, 'urdf', f'*{side}.urdf')))
    if sided:
        return sided[0] if len(sided) == 1 else ''
    files = sorted(os.path.basename(p) for p in glob(os.path.join(folder, '*.urdf')))
    plain = [n for n in files if 'left' not in n and 'right' not in n]
    plain = [n for n in plain if not n.endswith('_glb.urdf')] or plain
    return os.path.join(folder, plain[0]) if len(plain) == 1 else ''


def resolve_urdf(spec: str, side: str) -> str:
    if not spec:
        return ''
    if spec.endswith('.urdf'):
        path = spec.replace('{side}', side)
        if os.path.isabs(path):
            return path
        share = description_share()
        return os.path.join(share, path) if share else ''
    share = description_share()
    return find_hand_urdf(share, spec, side) if share else ''


def absolute_mesh_uris(xml: str, base_dir: str) -> str:
    def rewrite(m):
        name = m.group(3)
        if '://' in name:
            return m.group(0)
        path = os.path.normpath(os.path.join(base_dir, name))
        return f'{m.group(1)}{m.group(2)}file://{path}{m.group(2)}'

    return _MESH_FILENAME.sub(rewrite, xml)


def root_link(xml: str) -> str:
    robot = ET.fromstring(xml)
    children = {j.find('child').get('link') for j in robot.findall('joint')}
    names = [link.get('name') for link in robot.findall('link')]
    return next(n for n in names if n not in children)
