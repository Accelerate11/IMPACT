import numpy as np

from xq_autonomy.multisensor_monitor_node import record_image_diagnostics


def test_landing_texture_cannot_replace_research_image_quality():
    research, latest = {}, {}
    textured = np.arange(256, dtype=np.uint8).reshape(16, 16)
    blank = np.zeros((16, 16), dtype=np.uint8)
    record_image_diagnostics(research, latest, 'down', blank, 'HOVER', 10)
    assert not research
    record_image_diagnostics(research, latest, 'down', textured, 'RESEARCH', 20)
    record_image_diagnostics(research, latest, 'down', blank, 'LAND', 30)
    assert research['down']['acquisition_stamp_ns'] == 20
    assert research['down']['distinct_values'] == 256
    assert latest['down']['std'] == 0


def test_each_camera_freezes_its_first_research_frame():
    research, latest = {}, {}
    image = np.arange(100, dtype=np.uint8).reshape(10, 10)
    record_image_diagnostics(research, latest, 'front', image, 'RESEARCH', 10)
    record_image_diagnostics(research, latest, 'front', image+10, 'RESEARCH', 20)
    record_image_diagnostics(research, latest, 'down', image, 'RESEARCH', 21)
    assert research['front']['acquisition_stamp_ns'] == 10
    assert latest['front']['acquisition_stamp_ns'] == 20
    assert research['down']['acquisition_stamp_ns'] == 21
