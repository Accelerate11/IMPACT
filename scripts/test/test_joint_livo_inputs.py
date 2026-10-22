"""Five-sensor source whitelist, immutable payload and exact-pair boundaries."""
import copy
import hashlib
import importlib.util
from pathlib import Path
import sqlite3
import tempfile
import unittest


spec = importlib.util.spec_from_file_location('joint_input_api', Path(__file__).resolve().parents[1]/'replay_joint_livo_inputs.py')
api = importlib.util.module_from_spec(spec)
spec.loader.exec_module(api)


def entries(camera=(1_000_000_000,1_066_000_000), ranges=(1_000_000_000,1_020_000_000)):
    rows=[]
    streams={'imu':[990_000_000,1_080_000_000],'lidar':[990_000_000,1_080_000_000],
             'front':camera,'down':camera,'range':ranges}
    for role,stamps in streams.items():
        topic,(_,kind)=next((t,v) for t,v in api.SENSORS.items() if v[0]==role)
        for stamp in stamps:
            rows.append({'sensor':role,'stamp_ns':stamp,'topic':topic,'type':kind,
                'file':'/tmp/joint_fixture/source.db3','rowid':len(rows)+1})
    return sorted(rows,key=lambda r:(r['stamp_ns'],api.PRIORITY[r['sensor']]))


class PairTests(unittest.TestCase):
    def test_exact_seed_uses_next_original_frame_without_nearest_range(self):
        profile=api.pair_profile(entries())
        self.assertEqual(profile['valid_nonoverlapping_pair_count'],1)
        self.assertEqual(profile['pairs'][0]['dt_ns'],66_000_000)
        self.assertFalse(profile['pairs'][0]['t1_requires_new_range'])
        self.assertEqual(profile['other_ranges_input_audit_only_not_fused_by_this_profile'],1)

    def test_one_nanosecond_nearby_range_cannot_initialize(self):
        with self.assertRaises(ValueError):
            api.pair_profile(entries(ranges=(999_999_999,1_020_000_000)))

    def test_exact_seed_opportunities_are_not_all_fused_and_pair_cannot_reuse_t1(self):
        profile=api.pair_profile(entries(camera=(1_000_000_000,1_066_000_000,1_132_000_000),
            ranges=(1_000_000_000,1_066_000_000,1_132_000_000)))
        self.assertEqual(profile['exact_seed_opportunities'],3)
        self.assertEqual(profile['valid_nonoverlapping_pair_count'],1)
        self.assertEqual(len(profile['rejected_seed_opportunities']),2)

    def test_300ms_allowed_but_300ms_plus_1ns_not_allowed(self):
        self.assertEqual(api.pair_profile(entries(camera=(1_000_000_000,1_300_000_000)))['valid_nonoverlapping_pair_count'],1)
        with self.assertRaises(ValueError):
            api.pair_profile(entries(camera=(1_000_000_000,1_300_000_001)))

    def test_front_down_stamp_mismatch_fails_without_relabeling(self):
        rows=entries()
        next(r for r in rows if r['sensor']=='down')['stamp_ns']+=1
        with self.assertRaises(ValueError):api.pair_profile(rows)

    def test_repeated_acquisition_stamp_fails(self):
        rows=entries();rows.append(copy.deepcopy(next(r for r in rows if r['sensor']=='range')))
        rows.sort(key=lambda r:(r['stamp_ns'],api.PRIORITY[r['sensor']]))
        with self.assertRaises(ValueError):api.pair_profile(rows)


class PayloadTests(unittest.TestCase):
    def fixture(self,directory):
        path=Path(directory)/'source.db3'
        writer=sqlite3.connect(path)
        writer.execute('create table topics(id integer primary key,name text,type text)')
        writer.execute('create table messages(id integer primary key,topic_id integer,data blob)')
        writer.execute('insert into topics values(1,?,?)',('/impact/r2b3/down/range','sensor_msgs/msg/Range'))
        writer.execute('insert into topics values(2,?,?)',(api.TRUTH,'nav_msgs/msg/Odometry'))
        writer.execute('insert into messages values(1,1,?)',(b'allowed sensor',))
        writer.execute('insert into messages values(2,2,?)',(b'evaluator only secret',))
        writer.commit();writer.close()
        row={'topic':'/impact/r2b3/down/range','sensor':'range','type':'sensor_msgs/msg/Range',
             'rowid':1,'serialized_size_bytes':14,'payload_sha256':hashlib.sha256(b'allowed sensor').hexdigest()}
        return path,row

    def test_immutable_reader_returns_exact_bytes_and_rejects_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            path,row=self.fixture(directory);before=api.digest(path)
            database=api.connect(path)
            try:
                self.assertEqual(api.fetch_sensor_payload(database,row),b'allowed sensor')
                with self.assertRaises(sqlite3.OperationalError):database.execute('delete from messages')
            finally:database.close()
            self.assertEqual(api.digest(path),before)
            self.assertEqual(sorted(p.name for p in Path(directory).iterdir()),['source.db3'])

    def test_truth_row_cannot_be_loaded_via_forged_sensor_rowid(self):
        with tempfile.TemporaryDirectory() as directory:
            path,row=self.fixture(directory);row['rowid']=2
            database=api.connect(path)
            try:
                with self.assertRaises(ValueError):api.fetch_sensor_payload(database,row)
                row.update(topic=api.TRUTH,type='nav_msgs/msg/Odometry',sensor='truth')
                with self.assertRaises(ValueError):api.fetch_sensor_payload(database,row)
            finally:database.close()

    def test_changed_payload_hash_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path,row=self.fixture(directory);row['payload_sha256']='0'*64
            database=api.connect(path)
            try:
                with self.assertRaises(ValueError):api.fetch_sensor_payload(database,row)
            finally:database.close()

    def test_source_snapshot_rejects_any_existing_wal_before_opening(self):
        with tempfile.TemporaryDirectory() as directory:
            path,row=self.fixture(directory)
            (Path(directory)/'metadata.yaml').write_text('test metadata')
            (Path(directory)/'source.db3-wal').write_bytes(b'original sidecar fixture')
            with self.assertRaises(ValueError):api.snapshot(Path(directory))
            self.assertEqual((Path(directory)/'source.db3-wal').read_bytes(),b'original sidecar fixture')


class BarrierTests(unittest.TestCase):
    def test_payload_order_fixed_independent_of_mapping_iteration(self):
        self.assertEqual(api.barrier_payload(99,{'range':5,'down':4,'front':3,'lidar':2,'imu':1}),[99,1,2,3,4,5])

    def test_zero_count_is_valid_for_startup_but_invalid_uint64_rejected(self):
        zero={role:0 for role in api.ROLES}
        self.assertEqual(api.barrier_payload(1,zero),[1,0,0,0,0,0])
        for value in (-1,True,2**64):
            counts=dict(zero);counts['down']=value
            with self.assertRaises(ValueError):api.barrier_payload(1,counts)


class OutputBoundaryTests(unittest.TestCase):
    def test_output_inside_or_around_source_is_rejected_before_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            bag=Path(directory)/'original';bag.mkdir()
            for output in (bag,bag/'new-index',bag.parent):
                with self.assertRaises(ValueError):api.safe_output(bag,output)
            self.assertEqual(list(bag.iterdir()),[])

    def test_separate_new_output_is_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            bag=Path(directory)/'original';bag.mkdir()
            expected=Path(directory)/'new-index'
            self.assertEqual(api.safe_output(bag,expected),expected)
            self.assertFalse(expected.exists())


if __name__=='__main__':unittest.main()
