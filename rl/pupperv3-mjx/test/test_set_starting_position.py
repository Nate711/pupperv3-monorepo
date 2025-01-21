"""
pytest test/test_set_starting_position.py
"""

import pytest
from pathlib import Path
from pupperv3_mjx import utils
import xml.etree.ElementTree as ET


def test_set_starting_position():
    ORIGINAL_MODEL_PATH = Path("test/test_pupper_model.xml")

    xml_str = ORIGINAL_MODEL_PATH.read_text()
    tree = ET.ElementTree(ET.fromstring(xml_str))

    utils.set_robot_starting_position(
        tree, starting_pos=[0.1, 0.2, 0.5], starting_quat=[0.1, 0.2, 0.3, 0.4]
    )

    body = tree.find(".//worldbody/body[@name='base_link']")
    assert body.get("pos").split(" ")[0] == "0.1"
    assert body.get("pos").split(" ")[1] == "0.2"
    assert body.get("pos").split(" ")[2] == "0.5"

    assert body.get("quat").split(" ")[0] == "0.1"
    assert body.get("quat").split(" ")[1] == "0.2"
    assert body.get("quat").split(" ")[2] == "0.3"
    assert body.get("quat").split(" ")[3] == "0.4"

    home_position = tree.find(".//keyframe/key[@name='home']")
    assert list(map(float, home_position.get("qpos").split(" ")))[:7] == [
        0.1,
        0.2,
        0.5,
        0.1,
        0.2,
        0.3,
        0.4,
    ]


if __name__ == "__main__":
    pytest.main()
