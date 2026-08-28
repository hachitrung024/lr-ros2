from ament_pep257.main import main


def test_pep257():
    assert main(argv=["lr_terrain_geometry"]) == 0
