#!/usr/bin/env python3
"""Real MAVROS/Gazebo/AP471 unarmed experiment, in a separate installation."""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

from run_extnav_link_probe import Diagnostic, CANONICAL, sha, write


class AP471Diagnostic(Diagnostic):
    PHASE_PREFIX='ap471_extnav'
    PRESERVATION_RELATIVE='runs/verify_r2b3b4_preserved.py'
    PROTOCOL_RELATIVE='config/research_ap471_extnav.json'
    HELPERS=(
        'scripts/run_ap471_extnav_probe.py','scripts/run_ap471_extnav_probe.sh',
        'scripts/run_extnav_link_probe.py','scripts/probe_mavlink_tcp.py','scripts/probe_extnav_ros.py',
        'scripts/query_extnav_parameter_proof.py','scripts/observe_ap471_extnav.py',
        'scripts/analyze_ap471_extnav.py','scripts/analyze_extnav_link_probe.py',
        'scripts/analyze_extnav_guard_probe.py','scripts/prepare_livo_transport.py',
        'config/research_ap471_extnav.json',
        'src/xq_autonomy/xq_autonomy/p4_external_nav_node.py',
        'src/xq_autonomy/xq_autonomy/extnav_clock_guard.py',
        'src/xq_autonomy/xq_autonomy/extnav_transport_clock.py',
        'src/xq_autonomy/xq_autonomy/extnav_startup_gate.py',
        'src/xq_autonomy/xq_autonomy/extnav_runtime_graph.py',
        'src/xq_autonomy/xq_autonomy/fcu_extnav_gate.py',
    )

    def prepare(self):
        print(json.dumps({'event':'run_created','run_dir':str(self.run)}),flush=True)
        ports_text=subprocess.check_output(['ss','-H','-lntup'],text=True)
        (self.run/'ports.before.txt').write_text(ports_text)
        ports={int(p) for p in re.findall(r':(\d+)\s',ports_text)}
        if ports & {6760,6761,6762,6763,9002}:
            raise RuntimeError('AP471 or shared GZ simulation port is occupied')
        domain=next((d for d in range(100+os.getpid()%20,140)
            if not any(7400+250*d<=port<=7400+250*d+139 for port in ports)),None)
        if domain is None:
            raise RuntimeError('No isolated DDS domain available')
        self.env['ROS_DOMAIN_ID']=str(domain)
        self.preserve('before')
        self.install=self.args.install_prefix.resolve(strict=True)
        build_manifest=json.loads((self.install/'.xq_build_manifest.json').read_text())
        if build_manifest.get('status')!='PASS':
            raise ValueError('A successful isolated autonomy build is required')
        modules=build_manifest['installed_modules']
        for name,row in modules.items():
            if sha(Path(row['installed_path']))!=row['sha256']:
                raise ValueError('Installed module changed: '+name)
        self.env['PYTHONPATH']=str(self.install/'xq_autonomy/lib/python3.10/site-packages')+':'+self.env.get('PYTHONPATH','')
        self.env['AMENT_PREFIX_PATH']=str(self.install/'xq_autonomy')+':'+self.env.get('AMENT_PREFIX_PATH','')
        prep=self.args.preparation.resolve(strict=True)
        prepared=json.loads((prep/'status.json').read_text())
        if prepared.get('result')!='PASS' or prepared.get('commit')!='dbe792162d06cab66c3475fd5556bf7a120f119e':
            raise ValueError('Pinned AP471 preparation required')
        self.ap=Path(prepared['source']);self.binary=Path(prepared['binary'])
        if sha(self.binary)!=prepared['binary_sha256']:
            raise ValueError('AP471 binary changed')
        for relative in self.HELPERS:
            source=CANONICAL/relative;target=self.run/'sources'/relative
            target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(source.read_bytes())
            self.source_records[relative]={'bytes':source.stat().st_size,'sha256':sha(source)}
        self.protocol=json.loads((self.run/'sources'/self.PROTOCOL_RELATIVE).read_text())
        self.args.duration=self.protocol['duration_wall_s']
        write(self.run/'source-binding.json',self.source_records)
        for relative in self.HELPERS:
            if relative.startswith('src/xq_autonomy/xq_autonomy/'):
                name='xq_autonomy/'+Path(relative).name
                if modules[name]['sha256']!=self.source_records[relative]['sha256']:
                    raise ValueError('Frozen source differs from installed module: '+name)
        self.world=self.workspace/'xq_install/xq_gz_assets/share/xq_gz_assets/worlds/xq_r2b3_range.sdf'
        defaults=self.run/'configuration/ap471.parm'
        defaults.write_text(''.join(f'{k} {v}\n' for k,v in self.protocol['live_readiness']['required_parameters'].items()))
        dependencies=[self.world,self.binary,self.install/'.xq_build_manifest.json',prep/'status.json',
            self.ap/'Tools/autotest/default_params/copter.parm',self.ap/'Tools/autotest/default_params/gazebo-iris.parm',
            Path('/home/accelerate/ardupilot_gazebo/build/libArduPilotPlugin.so'),defaults]
        dependencies.extend(Path(row['installed_path']) for row in modules.values())
        for relative in ('libraries/GCS_MAVLink/GCS.cpp','libraries/GCS_MAVLink/GCS_Common.cpp',
            'ArduCopter/GCS_Copter.cpp','libraries/AP_VisualOdom/AP_VisualOdom.cpp',
            'libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp','libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp'):
            dependencies.append(self.ap/relative)
        write(self.run/'runtime-dependencies.json',{str(p):{'bytes':p.stat().st_size,'sha256':sha(p)} for p in dependencies})
        write(self.run/'run.json',{'source':self.args.source,'load':'lean','firmware_commit':prepared['commit'],
            'install_prefix':str(self.install),'duration_wall_s':self.args.duration,'armed_trial':False,
            'mission_nodes_started':False,'actual_flow_state_updates':0,
            'synthetic_reference_source_uses_Gazebo_pose':False,
            'synthetic_zero_pose_velocity':self.args.source=='synthetic',
            'source_is_estimator_evidence':self.args.source=='lio',
            'ros_domain_id':domain,'gz_partition':self.env['GZ_PARTITION'],
            'scope':self.protocol['scope']})
        self.command('transport-setup',[sys.executable,CANONICAL/'scripts/prepare_livo_transport.py',
            '--initial-peer-range','64','--output-dir',self.run/'configuration'])
        self.env['FASTRTPS_DEFAULT_PROFILES_FILE']=str(self.run/'configuration/fastdds-replay.xml')
        self.command('renderer-probe',['glxinfo','-B'],timeout=25)
        if 'D3D12 (NVIDIA' not in (self.run/'renderer-probe.log').read_text():
            raise RuntimeError('Requested NVIDIA renderer absent')

    def execute(self):
        self.prepare();self.stage='AP471_Gazebo_startup'
        defaults=','.join(map(str,[self.ap/'Tools/autotest/default_params/copter.parm',
            self.ap/'Tools/autotest/default_params/gazebo-iris.parm',self.run/'configuration/ap471.parm']))
        self.start('sitl',[self.binary,'-S','--model','JSON','--speedup','1','--slave','0','--wipe',
            '--defaults',defaults,'--base-port','6760','-I0','--sim-address=127.0.0.1',
            '--sim-port-in','9003','--sim-port-out','9002'],cwd=self.run/'sitl_runtime')
        self.wait_log('sitl','SERIAL0 on TCP port 6760',30)
        self.start('proxy',[sys.executable,self.run/'sources/scripts/probe_mavlink_tcp.py',
            '--output-dir',self.run/'wire','--target-port','6760','--listen-port','6761',
            '--guard-unarmed','--allow-fcu-startup-ascii','--allow-sitl-irlock-bootstrap-line',
            '--max-runtime-s',str(self.args.duration+180)])
        self.wait_log('proxy','ready',45)
        self.start('mavros',['ros2','launch','mavros','apm.launch',
            'fcu_url:=tcp://127.0.0.1:6761','namespace:=uav1/mavros'])
        self.start('gazebo',['gz','sim','-r','-s','-v','3',self.world])
        self.wait_log('sitl','JSON received',90)
        self.wait_log('mavros','Got HEARTBEAT',60)
        self.command('telemetry-request',['ros2','service','call','/uav1/mavros/set_stream_rate',
            'mavros_msgs/srv/StreamRate','{"stream_id":0,"message_rate":20,"on_off":true}'],timeout=25)
        self.command('parameter-proof',[sys.executable,self.run/'sources/scripts/query_extnav_parameter_proof.py',
            '--output',self.run/'configuration/fcu-parameter-proof.json','--config',self.run/'sources'/self.PROTOCOL_RELATIVE,
            '--timeout-s','60'],timeout=70)
        self.start('bridge',['ros2','run','xq_gz_bridge','xq_gz_bridge_node','--ros-args','--params-file',
            self.workspace/'xq_install/xq_gz_bridge/share/xq_gz_bridge/config/p4_external_nav.yaml','-p','publish_ground_truth:=false'])
        if self.args.source=='lio':
            for child,parent,xyz in (('xq_base_link','livox_imu',('0','0','0')),('livox_frame','xq_base_link',('0.04','0','0.12'))):
                self.start('tf_'+child,['ros2','run','tf2_ros','static_transform_publisher','--x',xyz[0],'--y',xyz[1],'--z',xyz[2],
                    '--roll','0','--pitch','0','--yaw','0','--frame-id',parent,'--child-frame-id',child,'--ros-args','-p','use_sim_time:=true'])
            self.start('lio',['ros2','run','xq_fast_lio','fastlio_mapping','--ros-args','--params-file',
                self.workspace/'xq_install/xq_fast_lio/share/xq_fast_lio/config/xq_p4.yaml','-p','use_sim_time:=true'])
        monitor=self.start('readiness',[sys.executable,self.run/'sources/scripts/observe_ap471_extnav.py',
            '--output-dir',self.run/'readiness','--duration',str(self.args.duration+self.protocol['monitor_extra_s']),
            '--parameter-proof',self.run/'configuration/fcu-parameter-proof.json'])
        arguments=[]
        for name,value in self.protocol['adapter'].items():
            arguments+=['-p',name+':='+str(value).lower()]
        arguments+=['-p','clock_ledger_path:='+str(self.run/'adapter-clock-ledger.jsonl')]
        self.start('adapter',[self.install/'xq_autonomy/lib/xq_autonomy/xq_p4_external_nav','--ros-args',*arguments])
        pause=self.protocol['source_pause_after_s'] if self.args.source=='synthetic' else 0
        observer=self.start('observer',[sys.executable,self.run/'sources/scripts/probe_extnav_ros.py',
            '--output-dir',self.run/'ros-observer','--source',self.args.source,'--duration',str(self.args.duration),
            '--pause-after-s',str(pause),'--pause-duration-s',str(self.protocol['source_pause_duration_s'] if pause else 0)])
        self.stage='unarmed_AP471_observation'
        deadline=time.monotonic()+self.args.duration+self.protocol['monitor_extra_s']+25
        while (observer.poll() is None or monitor.poll() is None) and time.monotonic()<deadline:
            for name,process in self.groups:
                if name not in {'observer','readiness'} and process.poll() is not None:
                    raise RuntimeError(f'Owned {name} exited: {process.returncode}')
            time.sleep(.2)
        if observer.poll()!=0 or monitor.poll()!=0:
            raise RuntimeError('Observation/monitor failed or exceeded deadline')
        self.stop();self.stage='closed_AP471_analysis'
        code=self.command('analysis',[sys.executable,self.run/'sources/scripts/analyze_ap471_extnav.py',
            '--run-dir',self.run,'--protocol',self.run/'sources'/self.PROTOCOL_RELATIVE],check=False)
        self.stage='complete' if code==0 else 'diagnostic_gate_failed'
        return code


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--install-prefix',type=Path,default=Path(os.environ.get('IMPACT_INSTALL_PREFIX','/home/accelerate/impact_research_ws/xq_install')))
    parser.add_argument('--preparation',type=Path,required=True)
    parser.add_argument('--source',choices=('synthetic','lio'),default='synthetic')
    args=parser.parse_args();args.workspace_root=Path('/home/accelerate/impact_research_ws');args.load='lean';args.duration=60
    diagnostic=AP471Diagnostic(args);code=1;error=None
    def interrupt(sig,frame):
        raise KeyboardInterrupt(signal.Signals(sig).name)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,interrupt)
    try:
        code=diagnostic.execute()
    except BaseException as exc:
        error=repr(exc)
    finally:
        diagnostic.stop()
        try:
            diagnostic.preserve('after')
            if (diagnostic.run/'external-assets.before.sha256').read_bytes()!=(diagnostic.run/'external-assets.after.sha256').read_bytes():
                raise RuntimeError('External assets changed')
            for relative,row in diagnostic.source_records.items():
                if sha(CANONICAL/relative)!=row['sha256'] or sha(diagnostic.run/'sources'/relative)!=row['sha256']:
                    raise RuntimeError('Frozen helper changed')
            path=diagnostic.run/'runtime-dependencies.json'
            if path.exists():
                before=json.loads(path.read_text());after={name:{'bytes':Path(name).stat().st_size,'sha256':sha(Path(name))} for name in before}
                write(diagnostic.run/'runtime-dependencies.after.json',after)
                if before!=after:raise RuntimeError('Runtime dependencies changed')
        except BaseException as exc:
            code=14;error=(error+'; ' if error else '')+'preservation: '+repr(exc)
        result={'status':'PASS' if code==0 and diagnostic.stage=='complete' else 'FAIL',
            'last_stage':diagnostic.stage,'exit_code':code,'error':error,'run_dir':str(diagnostic.run),
            'armed_trial':False,'actual_flow_state_updates':0,'flight_regression_repaired':False}
        write(diagnostic.run/'runner-result.json',result);print(json.dumps(result),flush=True)
    return code


if __name__=='__main__':raise SystemExit(main())
