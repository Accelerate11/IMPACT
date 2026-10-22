"""Selection changes records, never raw bytes/stamps/identities or source IMU."""
import copy
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


spec=importlib.util.spec_from_file_location('joint_selection_api',Path(__file__).resolve().parents[1]/'prepare_joint_livo_selection.py')
tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)
api=tool.api_module()


def fixture():
    entries=[]
    streams={'imu':range(1_000_000_000,42_000_000_001,5_000_000),
             'lidar':range(1_000_000_000,42_000_000_001,100_000_000),
             'front':range(1_000_000_000,42_000_000_001,66_000_000),
             'down':range(1_000_000_000,42_000_000_001,66_000_000),
             'range':range(1_000_000_000,42_000_000_001,20_000_000)}
    definitions={role:{'source_topic':topic,'type':kind,'published_topic':api.PREFIX+role} for topic,(role,kind) in api.SENSORS.items()}
    for role,stamps in streams.items():
        for stamp in stamps:
            entries.append({'sensor':role,'stamp_ns':stamp,'topic':definitions[role]['source_topic'],
                'type':definitions[role]['type'],'file':'/tmp/selection_original/bag.db3','rowid':len(entries)+1,
                'payload_sha256':'a'*64,'serialized_size_bytes':123,'frame_id':role})
    entries.sort(key=lambda row:(row['stamp_ns'],api.PRIORITY[row['sensor']]))
    counts={role:len(values) for role,values in streams.items()}
    return {'schema_version':1,'input_origin':api.ORIGIN,'bag':'/tmp/selection_original','camera_keep_every_n':1,
        'sensor_definitions':definitions,'entries':entries,'published_sensor_counts':counts,
        'source_counts':{definitions[role]['source_topic']:count for role,count in counts.items()},
        'source_files':{'bag.db3':{'bytes':123,'sha256':'b'*64}},
        'raw_files':[{'file':'/tmp/selection_original/bag.db3','size_bytes':123,'sha256':'b'*64}],
        'pair_profile':api.pair_profile(entries)}


class SelectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.base=fixture()

    def test_every_registered_profile_and_seed_preserves_source_and_is_publishable(self):
        baseline=copy.deepcopy(self.base)
        for name in tool.PROFILES:
            for seed in tool.SEEDS:
                selected,removed=tool.transform(self.base,name,seed,api)
                api.validate_index(selected)
                self.assertEqual(selected['source_files'],self.base['source_files'])
                self.assertEqual(selected['raw_files'],self.base['raw_files'])
                self.assertGreater(len(removed),0)
                self.assertEqual([v for v in selected['entries'] if v['sensor']=='imu'],
                                 [v for v in self.base['entries'] if v['sensor']=='imu'])
        self.assertEqual(self.base,baseline)

    def test_same_schedule_is_bitwise_deterministic(self):
        first=tool.transform(self.base,'range_dropout_2s',501,api)
        second=tool.transform(self.base,'range_dropout_2s',501,api)
        self.assertEqual(first,second)

    def test_range_dropout_uses_half_open_original_time_and_does_not_touch_other_roles(self):
        result,removed=tool.transform(self.base,'range_dropout_2s',501,api)
        start,end=21_000_000_000,23_000_000_000
        self.assertEqual(len(removed),100)
        self.assertTrue(all(v['source']['sensor']=='range' and start<=v['source']['stamp_ns']<end for v in removed))
        self.assertTrue(any(v['sensor']=='range' and v['stamp_ns']==end for v in result['entries']))

    def test_camera_dropout_removes_front_down_together_and_records_bridged_pairs(self):
        result,removed=tool.transform(self.base,'camera_dropout_0p2s',501,api)
        front=[v['stamp_ns'] for v in result['entries'] if v['sensor']=='front']
        down=[v['stamp_ns'] for v in result['entries'] if v['sensor']=='down']
        self.assertEqual(front,down)
        self.assertTrue(all(v['source']['sensor'] in ('front','down') for v in removed))
        self.assertIn('next_camera_semantics',result['selection_transform'])

    def test_stride_uses_source_scan_number_and_declared_phase(self):
        result,_=tool.transform(self.base,'lidar_keep5',503,api)
        original=[v for v in self.base['entries'] if v['sensor']=='lidar']
        actual=[v for v in result['entries'] if v['sensor']=='lidar']
        self.assertEqual(actual,original[2::5])
        self.assertEqual(result['selection_transform']['retained_lidar_phase'],2)

    def test_selected_and_removed_records_partition_original_without_modification(self):
        result,removed=tool.transform(self.base,'camera_dropout_1s',505,api)
        original={(v['file'],v['rowid']):v for v in self.base['entries']}
        selected={(v['file'],v['rowid']):v for v in result['entries']}
        dropped={(v['source']['file'],v['source']['rowid']):v['source'] for v in removed}
        self.assertFalse(set(selected)&set(dropped));self.assertEqual(selected|dropped,original)

    def test_schedule_seeds_are_explicitly_not_independent_flights(self):
        self.assertIn('NOT independent trajectories',tool.TABLE['seed_scope'])
        self.assertEqual(tool.TABLE['seeds'],[501,502,503,504,505])

    def test_unknown_profile_or_seed_is_rejected(self):
        for profile,seed in [('unknown',501),('range_dropout_2s',999)]:
            with self.assertRaises(ValueError):tool.transform(self.base,profile,seed,api)

    def test_existing_large_imu_gap_is_not_hidden_by_selection(self):
        base=copy.deepcopy(self.base)
        base['entries']=[v for v in base['entries'] if not(v['sensor']=='imu' and 10_000_000_000<=v['stamp_ns']<11_000_000_000)]
        base['published_sensor_counts']['imu']=sum(v['sensor']=='imu' for v in base['entries'])
        base['pair_profile']=api.pair_profile(base['entries'])
        with self.assertRaisesRegex(ValueError,'IMU'):tool.transform(base,'range_dropout_2s',501,api)

    def test_chained_selection_is_rejected_instead_of_hiding_multiple_faults(self):
        result,_=tool.transform(self.base,'range_dropout_2s',501,api)
        with self.assertRaisesRegex(ValueError,'Chained'):
            tool.transform(result,'camera_dropout_1s',501,api)

    def test_output_must_be_exclusive_and_outside_base_index_artifact(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);base=root/'frozen';base.mkdir();index=base/'sensor-index.json'
            for output in (base,base/'new',root):
                with self.assertRaises(ValueError):tool.output_path(index,Path('/tmp/selection_original'),output,api)
            self.assertEqual(tool.output_path(index,Path('/tmp/selection_original'),root/'derived',api),root/'derived')

    def test_prepare_writes_a_valid_derived_index_without_opening_raw_database(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);base=root/'frozen';base.mkdir();index=base/'sensor-index.json'
            index.write_text(json.dumps(self.base));before=index.read_bytes()
            args=SimpleNamespace(base_index=index,profile='range_dropout_2s',seed=501,output=root/'derived')
            with patch.object(api.sqlite3,'connect',side_effect=AssertionError('No database read authorized')),contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(tool.run(args),0)
            self.assertEqual(index.read_bytes(),before)
            self.assertEqual({p.name for p in base.iterdir()},{'sensor-index.json'})
            derived=json.loads((args.output/'sensor-index.json').read_text());api.validate_index(derived)
            report=json.loads((args.output/'prepare-result.json').read_text())
            self.assertTrue(report['base_index_and_tool_sources_unchanged'])
            self.assertFalse(report['original_databases_opened_or_rehashed'])
            with self.assertRaises(ValueError):tool.run(args)


if __name__=='__main__':unittest.main()
