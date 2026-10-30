import csv,hashlib,json,math,os,signal,sys,time
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from rcl_interfaces.srv import GetParameters

output=Path(sys.argv[1]); marker=Path(sys.argv[2]); expected=sys.argv[3]
output.mkdir(parents=True,exist_ok=False)
keep=True
def stop(*_):
    global keep
    keep=False
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('map_comparison_observer')
current={'truth':None,'odom':None,'clock':0.0}
counts={'truth':0,'odom':0,'clock':0}
def sec(t):return t.sec+t.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def truth_cb(msg):
    for transform in msg.transforms:
        if transform.child_frame_id=='sentry_bot':
            t=transform.transform.translation;q=transform.transform.rotation
            current['truth']=[sec(transform.header.stamp),t.x,t.y,yaw(q)]
            counts['truth']+=1
            break
def odom_cb(msg):
    p=msg.pose.pose.position;q=msg.pose.pose.orientation;v=msg.twist.twist
    current['odom']=[sec(msg.header.stamp),p.x,p.y,yaw(q),v.linear.x,v.linear.y,v.angular.z,msg.header.frame_id,msg.child_frame_id]
    counts['odom']+=1
def clock_cb(msg):
    current['clock']=sec(msg.clock);counts['clock']+=1
subs=[node.create_subscription(TFMessage,'/rm_sim/ground_truth',truth_cb,qos_profile_sensor_data),
      node.create_subscription(Odometry,'/odom',odom_cb,qos_profile_sensor_data),
      node.create_subscription(Clock,'/clock',clock_cb,qos_profile_sensor_data)]
client=node.create_client(GetParameters,'/small_point_lio/get_parameters');future=None;params={};request_names=['map_backend','moment_map_resolution','laser_point_cov','extrinsic_est_en','planar_constraint_en','defer_map_insertion_by_scan','dynamic_init_en']
pid=None;loaded_library=None;last_try=0;last_cpu=None;prev_wall=None
fields=['wall_s','sim_s','phase','truth_stamp','truth_x','truth_y','truth_yaw','odom_stamp','odom_x','odom_y','odom_yaw','odom_vx','odom_vy','odom_wz','odom_frame','odom_child','pid','cpu_seconds','cpu_percent','rss_bytes']
f=(output/'samples.csv').open('w');writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
start=time.monotonic();next_record=start
while keep and time.monotonic()-start < 900:
    rclpy.spin_once(node,timeout_sec=.05)
    now=time.monotonic()
    if not params and future is None and client.service_is_ready():
        future=client.call_async(GetParameters.Request(names=request_names))
    if future is not None and future.done():
        try:
            result=future.result()
            for key,v in zip(request_names,result.values):
                params[key] = v.string_value if v.type==4 else v.double_value if v.type==3 else v.bool_value if v.type==1 else None
        except Exception:future=None
    if pid is None and now-last_try>.5:
        last_try=now
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():continue
            try:
                argv=(p/'cmdline').read_bytes().split(b'\0')
                if argv and argv[0].endswith(b'/small_point_lio_node'):
                    pid=int(p.name)
                    for line in (p/'maps').read_text().splitlines():
                        if '/libsmall_point_lio.so' in line:
                            path=Path(line.split()[-1]);loaded_library={'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'from_temporary_install':str(path).startswith('/tmp/xxu_moment_sim_20261028/install/')};break
                    break
            except (OSError,ValueError):continue
    if now<next_record:continue
    next_record=now+.2
    row={'wall_s':now-start,'sim_s':current['clock'],'phase':'moving' if marker.exists() else 'warmup','pid':pid}
    if current['truth'] is not None:
        for k,v in zip(['truth_stamp','truth_x','truth_y','truth_yaw'],current['truth']):row[k]=v
    if current['odom'] is not None:
        for k,v in zip(['odom_stamp','odom_x','odom_y','odom_yaw','odom_vx','odom_vy','odom_wz','odom_frame','odom_child'],current['odom']):row[k]=v
    if pid is not None:
        try:
            stat=Path(f'/proc/{pid}/stat').read_text().split(') ',1)[1].split()
            cpu=(int(stat[11])+int(stat[12]))/os.sysconf('SC_CLK_TCK')
            rss=int(Path(f'/proc/{pid}/statm').read_text().split()[1])*os.sysconf('SC_PAGESIZE')
            row.update(cpu_seconds=cpu,rss_bytes=rss)
            if last_cpu is not None and prev_wall is not None and now>prev_wall:
                row['cpu_percent']=100*(cpu-last_cpu)/(now-prev_wall)
            last_cpu=cpu;prev_wall=now
        except (OSError,ValueError,IndexError):pass
    writer.writerow(row);f.flush()
f.close();node.destroy_node();rclpy.try_shutdown()
(output/'manifest.json').write_text(json.dumps({'expected_backend':expected,'parameters':params,'counts':counts,'pid':pid,'loaded_library':loaded_library,'duration_wall_s':time.monotonic()-start},indent=2))
