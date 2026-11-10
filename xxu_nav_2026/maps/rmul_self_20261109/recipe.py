"""Acquire a self-built map from committed ZLIO frames; no simulator truth input."""
from pathlib import Path
import json,math,time,argparse,hashlib
import cv2,yaml
from PIL import Image
from scipy.ndimage import binary_dilation,distance_transform_edt
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from localization_interfaces.msg import FrontendFrame
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
OUT=Path(__file__).resolve().parent
OFFSET=np.array([2.,2.,.06973823713479495])
SENSOR_OFFSET=np.array([.13917788102930553,-.08456666086335882,.16020454766857808])
class Capture(Node):
 def __init__(self):
  super().__init__('self_map_capture',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.odom=None;self.received=0.;self.healthy=False;self.epoch=None;self.count=0;self.last_saved=0
  self.clouds=[];self.origins=[];self.poses=[];self.stamps=[];self.errors=[]
  self.pub=self.create_publisher(TwistStamped,'/mapping/cmd_vel',10)
  self.subs=[self.create_subscription(FrontendFrame,'/frontend/frame',self.frame,qos_profile_sensor_data),self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data),self.create_subscription(Bool,'/cmd_vel_watchdog/healthy',lambda m:setattr(self,'healthy',m.data),10)]
 def on_odom(self,m):self.odom=m;self.received=time.monotonic()
 def frame(self,m):
  stamp=m.odometry.header.stamp.sec*10**9+m.odometry.header.stamp.nanosec
  if self.epoch is not None and self.epoch!=(m.instance_id,m.epoch):self.errors.append('frontend session changed');return
  self.epoch=(m.instance_id,m.epoch);self.count+=1
  if stamp-self.last_saved<500000000:return
  assert m.odometry.header.frame_id=='odom' and m.odometry.child_frame_id=='imu'
  fields={f.name:f for f in m.cloud.fields};assert all(fields[k].datatype==7 for k in ['x','y','z']) and not m.cloud.is_bigendian
  c=m.cloud;xyz=np.stack([np.ndarray((c.height,c.width),dtype='<f4',buffer=c.data,offset=fields[k].offset,strides=(c.row_step,c.point_step)).ravel() for k in ['x','y','z']],axis=1)
  xyz=xyz[np.isfinite(xyz).all(axis=1)];xyz=xyz[np.linalg.norm(xyz,axis=1)<25]
  pose=m.odometry.pose.pose;q=pose.orientation;r=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();p=np.array([pose.position.x,pose.position.y,pose.position.z])
  world=xyz@r.T+p+OFFSET
  _,indices=np.unique(np.floor(world/.05).astype(np.int32),axis=0,return_index=True);world=world[indices].astype(np.float32)
  self.clouds.append(world);self.origins.append(r@SENSOR_OFFSET+p+OFFSET);self.poses.append([*p,q.x,q.y,q.z,q.w]);self.stamps.append(stamp);self.last_saved=stamp
 def send(self,vx=0.,vy=0.,wz=0.):
  m=TwistStamped();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='base_link';m.twist.linear.x=vx;m.twist.linear.y=vy;m.twist.angular.z=wz;self.pub.publish(m)
 def state(self):
  p=self.odom.pose.pose;q=p.orientation;return np.array([p.position.x,p.position.y]),Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')[2]
 def wait(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.01);self.send()
 def go(self,target):
  start=time.monotonic();last_progress=start;previous=None
  while time.monotonic()-start<140:
   rclpy.spin_once(self,timeout_sec=.015)
   assert not self.errors,self.errors
   assert self.odom is not None and time.monotonic()-self.received<.5,'stale base odometry'
   assert self.healthy,'safety gate closed'
   p,yaw=self.state();error=np.array(target)-p;distance=np.linalg.norm(error)
   if distance<.06:self.wait(1.5);return
   if previous is None or distance<previous-.025:previous=distance;last_progress=time.monotonic()
   assert time.monotonic()-last_progress<12,'no mapping progress'
   world=error/max(distance,.001)*min(.12,distance*.6);c,s=math.cos(yaw),math.sin(yaw)
   self.send(c*world[0]+s*world[1],-s*world[0]+c*world[1],float(np.clip(-yaw*.8,-.15,.15)))
  raise RuntimeError('mapping waypoint timeout')
def save(node,summary):
 if not node.clouds:return
 offsets=np.cumsum([0]+[len(x) for x in node.clouds]);points=np.concatenate(node.clouds)
 np.savez_compressed(OUT/'observations.npz',points=points,offsets=offsets,origins=np.array(node.origins),poses=np.array(node.poses),stamp_ns=node.stamps)
 _,index=np.unique(np.floor(points/.05).astype(np.int32),axis=0,return_index=True);points=points[index]
 with (OUT/'map.pcd').open('wb') as stream:
  stream.write((f'# Self-built ZLIO map\nVERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n').encode());stream.write(np.c_[points,np.zeros(len(points),dtype=np.float32)].astype('<f4').tobytes())
 summary.update(frames=node.count,saved_frames=len(node.clouds),points=len(points),map_offset=OFFSET.tolist(),frontend_session=node.epoch,source='FrontendFrame cloud transformed by its atomic ZLIO posterior; no CAD occupancy or Gazebo truth')
 if node.odom is not None:summary['final_base_odom']=node.state()[0].tolist()
def main():
 rclpy.init();n=Capture();summary={'date':'2026-11-09','completed':False,'new_baseline':True};route=[[4.3,0.],[0.,0.],[0.,1.7],[-.9,1.7],[-.9,5.1],[0.,5.1],[-.9,5.1],[-.9,0.],[0.,0.]];summary['relative_odom_targets']=route
 try:
  end=time.monotonic()+90
  while time.monotonic()<end and (n.odom is None or not n.healthy or len(n.clouds)<8):rclpy.spin_once(n,timeout_sec=.02);n.send()
  assert n.odom is not None and n.healthy and len(n.clouds)>=8,'frontend/base/safety not ready'
  summary['initial_base_odom']=n.state()[0].tolist();summary['reached']=[]
  for target in route:
   print('mapping target',target,flush=True);n.go(target);summary['reached'].append(target)
  n.wait(3.);summary['completed']=True
 except BaseException as e:summary['failure']=repr(e)
 finally:
  n.wait(1.);save(n,summary);(OUT/'capture.json').write_text(json.dumps(summary,indent=2)+'\n');n.destroy_node();rclpy.shutdown();print(json.dumps(summary,indent=2),flush=True)
 raise SystemExit(0 if summary['completed'] else 1)



def export_grid():
 out=OUT;capture=json.loads((out/'capture.json').read_text());assert capture['completed'],'Incomplete survey'
 a=np.load(out/'observations.npz');points=a['points'];offsets=a['offsets'];origins=a['origins'];resolution=.05
 lo=np.floor(points[:,:2].min(axis=0)/resolution)*resolution-.5;hi=np.ceil(points[:,:2].max(axis=0)/resolution)*resolution+.5;shape=np.ceil((hi-lo)/resolution).astype(int)+1
 assert shape.max()<1500,'Unexpected map extents'
 free=np.zeros((shape[1],shape[0]),np.uint8);occupied=np.zeros_like(free)
 def pixel(xy):return np.floor((xy-lo)/resolution).astype(int)
 for i,origin in enumerate(origins):
  cloud=points[offsets[i]:offsets[i+1]];band=cloud[(cloud[:,2]>=.10)&(cloud[:,2]<=.34)];end=np.unique(pixel(band[:,:2]),axis=0);start=tuple(pixel(origin[:2]).tolist())
  for x,y in end:cv2.line(free,start,(int(x),int(y)),1,1)
  occupied[end[:,1],end[:,0]]=1
 occupied=binary_dilation(occupied,iterations=2)
 grid=np.full_like(free,205);grid[free>0]=254;grid[occupied]=0
 Image.fromarray(np.flipud(grid)).save(out/'nav2.pgm');Image.fromarray(np.flipud(grid)).save(out/'preview.png')
 (out/'nav2.yaml').write_text(yaml.safe_dump({'image':'nav2.pgm','mode':'trinary','resolution':resolution,'origin':[float(lo[0]),float(lo[1]),0.],'negate':0,'occupied_thresh':.65,'free_thresh':.196},sort_keys=False))
 clear=distance_transform_edt(grid==254)*resolution
 source=json.loads((out/'source_hashes.json').read_text());root=Path('/home/naiwu/xxu_2026');assert all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in source.items())
 checks={}
 for name,xy in {'spawn':[2.,2.],'left_approach':[2.,3.8],'old_detour_goal':[6.3,3.8],'old_passage_goal':[6.3,5.2]}.items():
  x,y=pixel(np.array(xy));inside=0<=x<shape[0] and 0<=y<shape[1];checks[name]={'xy':xy,'cell':int(grid[y,x]) if inside else None,'known_free_clearance_m':float(clear[y,x]) if inside else None}
 validation={'date':'2026-11-09','new_baseline':True,'survey_completed':True,'three_dimensional_points':capture['points'],'map_frame':'fixed startup map = odom + [2,2,base_height]; no trajectory fitting','resolution':resolution,'size':shape.tolist(),'origin':lo.tolist(),'occupied_height_band_m':[.1,.34],'occupancy_dilation_cells':2,'free_rays':'Only observed endpoint returns in height band; transformed actual lidar origins; unknown remains unknown','grid_cells':{'free':int((grid==254).sum()),'occupied':int((grid==0).sum()),'unknown':int((grid==205).sum())},'protected_original_models_unchanged':True,'coverage_checks':checks,'limitations':['No Nav2 or TDT localization acceptance yet','Low plate below 0.10 m is excluded, as in the earlier test recipe','Unobserved/occluded regions remain unknown; no CAD geometry fills the map','Grid prior descriptors are sampled from the acquired 3D map, not additional real scans']}
 (out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n');print(json.dumps(validation,indent=2))

if __name__=="__main__":
 import sys
 if "--export-only" in sys.argv:export_grid()
 else:main()
