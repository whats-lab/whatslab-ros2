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


def test_no_extra_env_when_ament_finds_it(monkeypatch):
    monkeypatch.setattr(assets, 'get_package_share_directory', lambda name: '/x')
    assert assets.description_env() == {}
