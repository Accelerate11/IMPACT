import importlib.util
from pathlib import Path
import pytest


spec=importlib.util.spec_from_file_location('joint_runner',Path(__file__).parents[1]/'run_joint_livo_replay.py')
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)


def source():
    streams={'imu':[90,100,200,300,310],'lidar':[100,200,300],
             'front':[50,150,250,350],'down':[50,150,250,350],'range':[50,150,250,360]}
    entries=[{'sensor':key,'stamp_ns':value} for key,values in streams.items() for value in values]
    return {'entries':entries,'published_sensor_counts':{key:len(values) for key,values in streams.items()}}


def test_all_five_eof_contract_uses_global_tail_and_only_covered_images():
    result=runner.completion(source())
    assert result['last_eligible_image_stamp_ns']==250
    assert result['expected_down_messages']==4
    assert result['expected_range_messages']==4
    assert result['last_barrier_stamp_ns']==360
    assert result['last_imu_stamp_ns']==310
    assert result['last_down_stamp_ns']==350


def test_changed_counts_cannot_complete():
    value=source();value['published_sensor_counts']['range']+=1
    with pytest.raises(ValueError,match='count mismatch'):runner.completion(value)


@pytest.mark.parametrize('role',['imu','lidar','front','down','range'])
def test_each_stream_must_be_strictly_increasing(role):
    value=source();row=next(row.copy() for row in value['entries'] if row['sensor']==role)
    value['entries'].append(row)
    with pytest.raises(ValueError,match='non-increasing'):runner.completion(value)


def test_nearest_down_stamp_cannot_replace_exact_epoch():
    value=source();next(row for row in value['entries'] if row['sensor']=='down')['stamp_ns']+=1
    with pytest.raises(ValueError,match='acquisition mismatch'):runner.completion(value)
