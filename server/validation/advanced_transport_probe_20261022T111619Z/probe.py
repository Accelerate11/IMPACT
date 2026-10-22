import argparse,time,json,hashlib
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2,PointField
p=argparse.ArgumentParser();p.add_argument('role');p.add_argument('output',type=Path);a=p.parse_args()
rclpy.init();n=Node('impact_large_payload_'+a.role);q=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE if a.role=='publisher' else ReliabilityPolicy.BEST_EFFORT)
count=0;payload=b'\0\0\x80\x3f'*4*23040;expected=hashlib.sha256(payload).hexdigest();matches=True;discovered=False
if a.role=='subscriber':
 def cb(m):
  global count,matches
  count+=1;matches=matches and hashlib.sha256(m.data).hexdigest()==expected
 subscription=n.create_subscription(PointCloud2,'/impact/server/payload_probe',cb,q);start=time.monotonic()
 while count<10 and time.monotonic()-start<15:rclpy.spin_once(n,timeout_sec=.05)
else:
 pub=n.create_publisher(PointCloud2,'/impact/server/payload_probe',q);start=time.monotonic()
 while pub.get_subscription_count()==0 and time.monotonic()-start<10:rclpy.spin_once(n,timeout_sec=.05)
 discovered=pub.get_subscription_count()>0
 for i in range(10):
  m=PointCloud2();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='synthetic_probe';m.width=23040;m.height=1;m.point_step=16;m.row_step=len(payload);m.fields=[PointField(name=k,offset=4*j,datatype=7,count=1) for j,k in enumerate(('x','y','z','intensity'))];m.data=payload
  pub.publish(m);count+=1;rclpy.spin_once(n,timeout_sec=.1);time.sleep(.1)
 time.sleep(1)
a.output.write_text(json.dumps({'role':a.role,'count':count,'payload_bytes':len(payload),'all_payload_sha_matched':matches,'subscriber_discovered':discovered}))
n.destroy_node();rclpy.shutdown()
