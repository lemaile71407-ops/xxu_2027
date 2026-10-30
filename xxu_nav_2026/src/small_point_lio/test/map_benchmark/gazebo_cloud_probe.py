"""Collect downsampled base-frame clouds and source-stamped simulator poses."""
import json,math,signal,sys,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

output=Path(sys.argv[1]);marker=Path(sys.argv[2]);output.mkdir(parents=True,exist_ok=False)
keep=True
signal.signal(signal.SIGINT,lambda *_:globals().__setitem__('keep',False))
signal.signal(signal.SIGTERM,lambda *_:globals().__setitem__('keep',False))
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('map_cloud_probe')
clouds=[];stamps=[];truth=[];odom=[];last_stamp=-1.;frames=set();raw_count=0
sec=lambda stamp:stamp.sec+stamp.nanosec*1e-9

def on_truth(msg):
    for t in msg.transforms:
        if t.child_frame_id=='sentry_bot':
            p=t.transform.translation;q=t.transform.rotation
            truth.append([sec(t.header.stamp),p.x,p.y,p.z,q.x,q.y,q.z,q.w]);break

def on_odom(msg):
    p=msg.pose.pose.position;q=msg.pose.pose.orientation
    odom.append([sec(msg.header.stamp),p.x,p.y,p.z,q.x,q.y,q.z,q.w])

def on_cloud(msg):
    global raw_count,last_stamp
    if not marker.exists() or len(clouds)>=80:return
    t=sec(msg.header.stamp)
    if t-last_stamp<.4:return
    raw_count+=1
    if [f.name for f in msg.fields[:3]]!=['x','y','z']:return
    points=point_cloud2.read_points_numpy(msg,field_names=['x','y','z'],skip_nans=True)
    if len(points)<100:return
    good=np.isfinite(points).all(axis=1)&(points[:,2]>.15)&(points[:,2]<1.5)
    points=points[good]
    if len(points)<100:return
    indices=np.linspace(0,len(points)-1,min(len(points),2500),dtype=int)
    clouds.append(np.asarray(points[indices],dtype=np.float32));stamps.append(t);frames.add(msg.header.frame_id);last_stamp=t
subs=[node.create_subscription(PointCloud2,'/cloud_deskewed',on_cloud,qos_profile_sensor_data),
      node.create_subscription(Odometry,'/odom',on_odom,qos_profile_sensor_data),
      node.create_subscription(TFMessage,'/rm_sim/ground_truth',on_truth,qos_profile_sensor_data)]
start=time.monotonic()
while keep and time.monotonic()-start<300:rclpy.spin_once(node,timeout_sec=.05)
node.destroy_node();rclpy.try_shutdown()
offsets=[0]
for cloud in clouds:offsets.append(offsets[-1]+len(cloud))
np.savez_compressed(output/'observations.npz',stamps=np.array(stamps),points=np.concatenate(clouds) if clouds else np.empty((0,3),np.float32),offsets=np.array(offsets),truth=np.array(truth),odom=np.array(odom))
(output/'manifest.json').write_text(json.dumps({'raw_cloud_messages_attempted':raw_count,'recorded_frames':len(clouds),'cloud_frames':sorted(frames),'truth_messages':len(truth),'odom_messages':len(odom),'source_interval_min_s':.4,'max_points_per_scan':2500},indent=2))
