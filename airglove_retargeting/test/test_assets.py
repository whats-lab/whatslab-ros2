import os

from airglove_retargeting import assets


def test_pip_install_prefix_is_added_for_rviz(tmp_path, monkeypatch):
    marker = tmp_path / 'share' / 'ament_index' / 'resource_index' / 'packages'
    marker.mkdir(parents=True)
    (marker / assets.PACKAGE).touch()

    def not_found(name):
        raise assets.PackageNotFoundError(name)

    monkeypatch.setattr(assets, 'get_package_share_directory', not_found)
    monkeypatch.setattr(assets, '_pip_prefixes', lambda: ['/nonexistent', str(tmp_path)])
    monkeypatch.setenv('AMENT_PREFIX_PATH', '/opt/ros/jazzy')
    env = assets.description_env()
    assert env == {'AMENT_PREFIX_PATH': os.pathsep.join([str(tmp_path), '/opt/ros/jazzy'])}


def test_prefix_of_imported_share_is_a_pip_prefix(tmp_path, monkeypatch):
    share = tmp_path / 'share' / assets.PACKAGE
    monkeypatch.setattr(assets, 'description_share', lambda: str(share))
    assert assets._pip_prefixes()[-1] == str(tmp_path)


def test_no_extra_env_when_ament_finds_it(monkeypatch):
    monkeypatch.setattr(assets, 'get_package_share_directory', lambda name: '/x')
    assert assets.description_env() == {}


def _touch(root, *names):
    for name in names:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()


def test_find_hand_urdf_layouts(tmp_path):
    _touch(
        tmp_path,
        'leap_hand/leap_hand_right.urdf',
        'leap_hand/leap_hand_right_glb.urdf',
        'dclaw/dclaw.urdf',
        'orca_hand/urdf/right.urdf',
        'schunk_hand/schunk_svh_hand_right.urdf',
        'schunk_hand/schunk_svh_hand_right_glb.urdf',
        'robotis_hx5_d20/urdf/hx5_d20_right.urdf',
        'barrett/bhand_model.urdf',
        'barrett/bhand_model_glb.urdf',
        'two/a_right.urdf',
        'two/b_right.urdf',
        'variant/v_right_a.urdf',
    )

    def find(model, side='right'):
        path = assets.find_hand_urdf(str(tmp_path), model, side)
        return os.path.relpath(path, tmp_path) if path else ''

    assert find('leap_hand') == 'leap_hand/leap_hand_right.urdf'
    assert find('dclaw', 'left') == 'dclaw/dclaw.urdf'
    assert find('orca_hand') == 'orca_hand/urdf/right.urdf'
    assert find('schunk_hand') == 'schunk_hand/schunk_svh_hand_right.urdf'
    assert find('robotis_hx5_d20') == 'robotis_hx5_d20/urdf/hx5_d20_right.urdf'
    assert find('barrett') == 'barrett/bhand_model.urdf'
    assert find('leap_hand', 'left') == ''
    assert find('two') == ''
    assert find('variant', 'left') == ''
    assert find('missing') == ''


def test_resolve_urdf_spec_forms(tmp_path, monkeypatch):
    _touch(tmp_path, 'orca_hand/urdf/left.urdf')
    monkeypatch.setattr(assets, 'description_share', lambda: str(tmp_path))
    expected = str(tmp_path / 'orca_hand/urdf/left.urdf')
    assert assets.resolve_urdf('orca_hand', 'left') == expected
    assert assets.resolve_urdf('orca_hand/urdf/{side}.urdf', 'left') == expected
    assert assets.resolve_urdf('/abs/{side}.urdf', 'left') == '/abs/left.urdf'
    assert assets.resolve_urdf('', 'left') == ''


def test_absolute_mesh_uris_rewrites_relative_only():
    xml = (
        '<robot><link name="a"><visual><geometry>'
        '<mesh filename="meshes/a.stl" scale="1 1 1"/>'
        "<mesh filename='../m/b.obj'/>"
        '<mesh filename="package://dexhand_description/x/c.stl"/>'
        '<mesh filename="file:///abs/d.stl"/>'
        '</geometry></visual></link></robot>'
    )
    out = assets.absolute_mesh_uris(xml, '/share/hand')
    assert 'filename="file:///share/hand/meshes/a.stl" scale="1 1 1"' in out
    assert "filename='file:///share/m/b.obj'" in out
    assert 'filename="package://dexhand_description/x/c.stl"' in out
    assert 'filename="file:///abs/d.stl"' in out


def test_root_link():
    xml = (
        '<robot name="r"><link name="tip"/><link name="palm"/><link name="base"/>'
        '<joint name="j1" type="revolute"><parent link="palm"/><child link="tip"/></joint>'
        '<joint name="j0" type="fixed"><parent link="base"/><child link="palm"/></joint>'
        '</robot>'
    )
    assert assets.root_link(xml) == 'base'
