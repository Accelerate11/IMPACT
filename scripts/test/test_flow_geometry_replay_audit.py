"""Small independent geometry fixtures and hostile-artifact regression checks."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "audit_flow_geometry_replay.py"
SPEC = importlib.util.spec_from_file_location("independent_geometry_audit", SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)
PROTOCOL = SCRIPT.parent.parent / "config/research_flow_precondition_gate.json"


@pytest.fixture
def window():
    cv2.setNumThreads(1)
    pixels = np.zeros((240,320), np.uint8)
    for y in range(25, 225, 25):
        for x in range(25, 305, 25):
            pixels[y-3:y+4, x-3:x+4] = 180+(x+y)%76
    points = cv2.goodFeaturesToTrack(pixels,maxCorners=150,qualityLevel=.01,minDistance=5.,blockSize=5)
    opts = dict(winSize=(21,21),maxLevel=3,criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,30,.01))
    forward,status,_ = cv2.calcOpticalFlowPyrLK(pixels,pixels,points,None,**opts)
    assert status.all()
    backward,status,_ = cv2.calcOpticalFlowPyrLK(pixels,pixels,forward,None,**opts)
    assert status.all()
    start,end = 1_000_000_000,1_066_000_000
    def ref(name):
        return {"source":name}
    before={"stamp_ns":start,"ref":ref("before"),"camera_info_ref":ref("info-before")}
    after={"stamp_ns":end,"ref":ref("after"),"camera_info_ref":ref("info-after")}
    stamps=list(range(995_000_000,1_075_000_000,5_000_000))
    imus=[{"stamp_ns":t,"angular_velocity_body_rad_s":[0.,0.,0.],"ref":ref("imu"+str(t))} for t in stamps]
    left,right=1,15
    tracks=[]
    for i,(old,new,returned) in enumerate(zip(points[:,0],forward[:,0],backward[:,0])):
        xy=(old.astype(float)-AUDIT.PRINCIPAL)/AUDIT.FOCAL
        target=(new.astype(float)-AUDIT.PRINCIPAL)/AUDIT.FOCAL
        x,y=xy
        j=[[x*y,-(1+x*x),y],[1+y*y,-x*y,-x]]
        tracks.append({"track_id":i,"old_pixel":old.astype(float).tolist(),"new_pixel":new.astype(float).tolist(),
                       "old_normalized_ray":[*xy,1.],"new_normalized_ray":[*target,1.],
                       "raw_normalized_displacement":(target-xy).tolist(),"forward_backward_error_px":float(np.linalg.norm(returned-old)),
                       "jacobian_gamma_at_zero":j,"covariance":None,"uncertainty_known":False,
                       "c_gamma":[0.,0.],"J_gamma":j,"compensated_normalized_displacement":(target-xy).tolist()})
    row={"schema_version":1,"window_kind":"native","window_index":0,"acquisition_start_ns":start,"acquisition_end_ns":end,
         "source_image_stamps_ns":[start,end],"dt_s":.066,"source_images":[before["ref"],after["ref"]],
         "source_camera_info":[before["camera_info_ref"],after["camera_info_ref"]],"frame_id":"impact_down_optical",
         "measurement_space":"normalized_image_plane","track_id_scope":"image_pair","selection_dependencies":["down_camera"],
         "gyro_used":False,"ground_truth_used":False,"covariance":None,"uncertainty_known":False,
         "metric_velocity_valid":False,"metric_velocity_mps":None,"scale_source":"unavailable","depth_m":None,
         "cross_time_covariance":None,"fusion_update_performed":False,"detected_track_count":len(points),"track_count":len(tracks),
         "tracks":tracks,"valid":True,"reason":"OK_RAW_PIXEL_TRACKS_NO_SCALE_OR_COVARIANCE",
         "gyro":{"covered":True,"reason":"OK","source":"livox_imu","bias_subtracted":False,"integral_covariance":None,
                 "state_gyro_cross_covariance":None,"integral_optical_rad":[0.,0.,0.],"sample_index_range_in_source_index":[left,right],
                 "first_sample_ref":imus[left]["ref"],"last_sample_ref":imus[right]["ref"],"bracketing_sample_count":right-left+1},
         "blocked_reasons":list(AUDIT.BLOCKS),"noise_statistics":{"status":"UNAVAILABLE","reason":"COVARIANCE_UNKNOWN",
                 "uncertainty_known":False,"R_effective":None,"C_effective":None},
         "rotation_oracle":{"tested_tracks":1,"method":"cv2.Rodrigues","central_difference_step_rad":2e-6,
                            "maximum_jacobian_absolute_error":0.,"maximum_displacement_absolute_error":0.}}
    return row,before,after,imus,stamps,{start:pixels,end:pixels}


def verify_window(fixture):
    row,before,after,imus,stamps,pixels=fixture
    return AUDIT.check_window(row,"native",0,before,after,imus,stamps,pixels,1e-7)


def test_small_static_pixel_fixture_passes_without_production_import(window):
    result=verify_window(window)
    assert result["valid"] == 1 and result["tracks"] >= 12 and result["oracle_window"] == 1
    assert result["max_jacobian_error"] < 1e-7
    assert "xq_autonomy" not in SCRIPT.read_text().split("import ")[-1]


@pytest.mark.parametrize("field,value",[
    ("acquisition_start_ns",1_000_000_001),("fusion_update_performed",True),
    ("depth_m",1.),("cross_time_covariance",[[0.]]),("covariance",[[0.]]),
    ("metric_velocity_valid",True),("gyro_used",True),("blocked_reasons",[]),
    ("track_count",0),("source_images",[]),("valid",False),
])
def test_window_contract_tampering_rejected(window,field,value):
    window[0][field]=value
    with pytest.raises(ValueError):
        verify_window(window)


@pytest.mark.parametrize("field,value",[
    ("old_normalized_ray",[0.,0.,1.]),("raw_normalized_displacement",[.01,0.]),
    ("jacobian_gamma_at_zero",[[0.]*3]*2),("J_gamma",[[0.]*3]*2),
    ("c_gamma",[.01,0.]),("compensated_normalized_displacement",[.01,0.]),
    ("forward_backward_error_px",1.1),("covariance",[[0.]*2]*2),
])
def test_interior_track_tampering_rejected(window,field,value):
    # Use an interior track, beyond the first point used by the writer's oracle.
    window[0]["tracks"][10][field]=value
    with pytest.raises(ValueError):
        verify_window(window)


def test_pixel_and_ray_joint_forgery_rejected_by_original_image_LK(window):
    track=window[0]["tracks"][10]
    track["old_pixel"][0]+=.1
    track["new_pixel"][0]+=.1
    for field in ("old_normalized_ray","new_normalized_ray"):
        track[field][0]+=.1/AUDIT.FOCAL[0]
    x,y=track["old_normalized_ray"][:2]
    track["jacobian_gamma_at_zero"]=track["J_gamma"]=[[x*y,-(1+x*x),y],[1+y*y,-x*y,-x]]
    with pytest.raises(ValueError,match="original detected pixel"):
        verify_window(window)


def test_omitting_all_tracks_fails_independent_tracking(window):
    row=window[0]
    row.update(tracks=[],track_count=0,valid=False,reason="INSUFFICIENT_TRACKS")
    row["blocked_reasons"].append("INSUFFICIENT_TRACKS")
    row["noise_statistics"]["reason"]="NO_COVERED_TRACK_JACOBIANS"
    row["rotation_oracle"].update(tested_tracks=0,maximum_jacobian_absolute_error=None,maximum_displacement_absolute_error=None)
    with pytest.raises(ValueError,match="accepted point set"):
        verify_window(window)


def test_exact_rotation_zero_matches_closed_form_and_large_rotation_is_nonzero():
    xy=np.array([.2,-.3])
    displacement,j=AUDIT.rotation_oracle(xy,np.zeros(3))
    np.testing.assert_array_equal(displacement,[0.,0.])
    np.testing.assert_allclose(j,[[-.06,-1.04,-.3],[1.09,.06,-.2]],atol=1e-10)
    displacement,_=AUDIT.rotation_oracle(xy,np.array([.05,-.02,.01]))
    assert np.linalg.norm(displacement)>.01


def test_independent_gyro_clips_interpolated_endpoints_and_rejects_gaps():
    records=[{"angular_velocity_body_rad_s":[float(i),0.,0.]} for i in range(3)]
    stamps=[1_000_000_000,1_010_000_000,1_020_000_000]
    value,reason,bounds=AUDIT.integrate_gyro(records,stamps,1_005_000_000,1_015_000_000)
    np.testing.assert_allclose(value,[0.,-.01,0.],atol=1e-15)
    assert reason=="OK" and bounds==(0,2)
    assert AUDIT.integrate_gyro(records,stamps,999_000_000,1_015_000_000)[1]=="GYRO_WINDOW_UNCOVERED"
    assert AUDIT.integrate_gyro(records,[1,30_000_000,60_000_000],2,50_000_000)[1]=="GYRO_GAP_EXCEEDED"


def test_ledger_middle_membership_and_omitted_image_rejected():
    images=[{"stamp_ns":i+1,"ref":{"image":i},"camera_info_ref":{"info":i}} for i in range(7)]
    aligned=images[::3]
    rows=[]
    for i,image in enumerate(images):
        j=i//3 if i%3==0 else None
        rows.append({"schema_version":1,"image_sequence_id":i+1,"header_stamp_ns":i+1,"classification":"ACCOUNTED_ORIGINAL_DOWN_IMAGE",
                     "source_image":image["ref"],"source_camera_info":image["camera_info_ref"],"retained_front_epoch":j is not None,
                     "native_window_indices":[v for v in (i-1,i) if 0<=v<6],
                     "aligned_window_indices":[v for v in (j-1,j) if 0<=v<2] if j is not None else []})
    AUDIT.check_ledger(rows,images,aligned)
    rows[3]["aligned_window_indices"]=[]
    with pytest.raises(ValueError):
        AUDIT.check_ledger(rows,images,aligned)
    with pytest.raises(ValueError):
        AUDIT.check_ledger(rows[:-1],images,aligned)


@pytest.mark.parametrize("text",['{"x":1,"x":2}','{"x":NaN}','{"x":1e999}'])
def test_hostile_json_rejected(text):
    with pytest.raises(ValueError):
        AUDIT.loads(text)


def test_frozen_protocol_rejects_weaker_thresholds():
    protocol=json.loads(PROTOCOL.read_text())
    AUDIT.frozen_protocol(protocol)
    protocol["bag_checks"]["minimum_aligned_valid_track_windows"]=1
    with pytest.raises(ValueError):
        AUDIT.frozen_protocol(protocol)


def test_missing_artifacts_fail_closed_with_machine_readable_error(tmp_path):
    result=AUDIT.audit(tmp_path,PROTOCOL)
    assert result["status"]=="FAIL" and result["errors"] and result["checks"]["execution_completed"] is False


@pytest.fixture
def original_sqlite(tmp_path):
    serialization=pytest.importorskip("rclpy.serialization")
    messages=pytest.importorskip("sensor_msgs.msg")
    standard=pytest.importorskip("std_msgs.msg")
    bag=tmp_path/"bag"; bag.mkdir()
    path=bag/"rosbag_0.db3"
    image=messages.Image(); image.header.stamp.sec=1; image.header.frame_id="impact_down_optical"
    image.height=240; image.width=320; image.encoding="rgb8"; image.step=960; image.data=bytes(230400)
    front=copy.deepcopy(image); front.header.frame_id="impact_front_optical"
    info=messages.CameraInfo(); info.header=copy.deepcopy(image.header); info.height=240; info.width=320
    info.distortion_model="plumb_bob"; info.d=[0.]*5
    info.k=[AUDIT.FOCAL[0],0.,159.5,0.,AUDIT.FOCAL[1],119.5,0.,0.,1.]
    imu=messages.Imu(); imu.header=copy.deepcopy(image.header); imu.header.frame_id="livox_imu"
    imu.angular_velocity.x=.4
    flow=standard.String(); flow.data='{"hello":"original"}'
    originals=[image,info,front,imu,flow]
    index={"messages":{v[0]:[] for v in AUDIT.TOPICS.values()},"topic_counts":{}}
    con=sqlite3.connect(path)
    con.executescript("create table topics(id integer,name text,type text);create table messages(id integer,topic_id integer,timestamp integer,data blob);")
    for i,((topic,(key,kind)),message) in enumerate(zip(AUDIT.TOPICS.items(),originals),1):
        blob=serialization.serialize_message(message)
        con.execute("insert into topics values(?,?,?)",(i,topic,kind))
        con.execute("insert into messages values(?,?,?,?)",(i,i,i,blob))
        decoded=AUDIT.decode_payload(blob,key)
        decoded.pop("pixels",None)
        decoded["ref"]={"file":str(path.resolve()),"rowid":i,"topic":topic,"type":kind,
                        "serialized_size_bytes":len(blob),"sha256":hashlib.sha256(blob).hexdigest()}
        index["messages"][key].append(decoded)
        index["topic_counts"][topic]=1
    # Unsupported truth bytes prove the SQL payload reader only uses whitelist IDs.
    con.execute("insert into topics values(99,'/ground_truth','unsupported/no/decoder')")
    con.execute("insert into messages values(99,99,99,?)",(b"DO NOT DECODE",))
    con.commit(); con.close()
    return index,bag.resolve()


def test_original_CDR_manual_decode_matches_official_ROS_serialization(original_sqlite):
    index,bag=original_sqlite
    pixels,count=AUDIT.check_sources(index,bag)
    assert count==5 and pixels[1_000_000_000].shape==(240,320)


def test_official_camera_info_accepts_only_exact_zero_terminal_alignment(original_sqlite):
    index,bag=original_sqlite
    record=index["messages"]["down_camera_info"][0]
    database=sqlite3.connect((bag/"rosbag_0.db3").as_uri()+"?mode=ro&immutable=1",uri=True)
    database.execute("PRAGMA query_only=ON")
    blob=database.execute("select data from messages where id=?",(record["ref"]["rowid"],)).fetchone()[0]
    database.close()
    # The actual R2A recorded CameraInfo has exactly this 373->376 layout.
    assert len(blob)==373 and (-len(blob))%4==3
    expected=AUDIT.decode_payload(blob,"down_camera_info")
    assert AUDIT.decode_payload(blob+b"\0\0\0","down_camera_info")==expected
    for suffix in (b"\0",b"\0\0",b"\0\0\1",b"\0\0\0\0",b"\0"*7,b"junk"):
        with pytest.raises(ValueError,match="trailing content"):
            AUDIT.decode_payload(blob+suffix,"down_camera_info")


@pytest.mark.parametrize("payload_size",[4,5,6,7])
def test_terminal_alignment_rejects_extra_complete_word_and_nonzero(payload_size):
    # Exercise every possible final alignment, including already aligned data.
    blob=b"\x00\x01\x00\x00"+bytes(payload_size-4)
    reader=AUDIT.CDR(blob);reader.pos=payload_size;reader.done()
    padding=(-payload_size)%4
    reader=AUDIT.CDR(blob+bytes(padding));reader.pos=payload_size;reader.done()
    for suffix in (bytes(padding+4),b"\1"+bytes(max(0,padding-1))):
        reader=AUDIT.CDR(blob+suffix);reader.pos=payload_size
        with pytest.raises(ValueError,match="trailing content"):
            reader.done()


@pytest.mark.parametrize("kind",["payload","header","omission","escape"])
def test_original_CDR_ref_or_header_forgery_rejected(original_sqlite,kind):
    index,bag=original_sqlite
    record=index["messages"]["imu"][0]
    if kind=="payload": record["ref"]["sha256"]="0"*64
    if kind=="header": record["stamp_ns"]+=1
    if kind=="omission": index["messages"]["imu"]=[]
    if kind=="escape": record["ref"]["file"]=str(bag.parent/"other.db3")
    with pytest.raises(ValueError):
        AUDIT.check_sources(index,bag)
