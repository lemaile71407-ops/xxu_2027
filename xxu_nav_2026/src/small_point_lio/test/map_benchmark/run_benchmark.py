#!/usr/bin/env python3
"""Build actual external moment map and local SmallIVox; no ROS or live robot.
Requires Eigen3, g++, and gtest already installed. Does not fetch dependencies.
Upstream sources remain outside the MIT package and are never installed.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess

p=argparse.ArgumentParser()
p.add_argument('--upstream',type=Path,required=True)
p.add_argument('--build-dir',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
here=Path(__file__).resolve().parent
package=here.parents[1]
upstream=a.upstream.resolve()
frontend=upstream/'src/frontend'
build=a.build_dir.resolve();build.mkdir(parents=True,exist_ok=True)
common=['g++','-std=c++17','-O3','-DNDEBUG','-DEIGEN_DONT_PARALLELIZE','-I/usr/include/eigen3',
        '-I'+str(frontend/'include'),'-I'+str(frontend/'src'),'-I'+str(here),
        '-I'+str(package/'src'),'-I'+str(package/'3rdparty')]
sources=[frontend/'src'/n for n in ['map.cpp','plane.cpp','state.cpp']]
env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
# Independent objects keep peak compile memory low and permit exact reproduction.
objects=[]
for source in sources:
    obj=build/(source.stem+'.o')
    subprocess.run(common+['-c',str(source),'-o',str(obj)],check=True,env=env)
    objects.append(str(obj))
binary=build/'benchmark_maps'
subprocess.run(common+[str(here/'benchmark_maps.cpp'),*objects,'-pthread','-o',str(binary)],check=True,env=env)
test=build/'upstream_map_test'
subprocess.run(common+[str(frontend/'test/integration/map_test.cpp'),*objects,'-lgtest_main','-lgtest','-pthread','-o',str(test)],check=True,env=env)
tests=subprocess.run([str(test)],check=True,capture_output=True,text=True,env=env)
(build/'upstream_map_test.log').write_text(tests.stdout+tests.stderr)
print(tests.stdout,flush=True)
header='method,scene,points,seed,voxel_m,passes,first_update_s,later_update_s,first_heap_delta_bytes,final_heap_delta_bytes,rss_delta_bytes,peak_rss_bytes,queries,matched,good,query_s,residual_error_rmse_m,normal_error_mean_deg,checksum'.split(',')
a.output.parent.mkdir(parents=True,exist_ok=True)
with a.output.open('w') as f:
    writer=csv.writer(f);writer.writerow(header)
    for scene in ['plane','corner','steps','dense_patch']:
        for count in [24000,96000,384000]:
            for seed in [20261028,20261029,20261030]:
                methods=[('ivox',.2),('moments',.2),('moments',.5)]
                # Rotate execution order to reduce thermal/order bias.
                shift=(seed-20261028)%3;methods=methods[shift:]+methods[:shift]
                for method,voxel in methods:
                    result=subprocess.run([str(binary),method,scene,str(count),str(seed),str(voxel),'3'],check=True,capture_output=True,text=True,env=env)
                    row=next(csv.reader([result.stdout.strip()]));assert len(row)==len(header)
                    writer.writerow(row);f.flush()
                print(scene,count,seed,'done',flush=True)
identity_files=[here/'benchmark_maps.cpp',here/'pch.h',Path(__file__).resolve(),
                package/'src/small_point_lio/small_ivox.h',package/'src/small_point_lio/estimator.cpp',binary,test,*sources,
                frontend/'src/storage.hpp',frontend/'src/moments.hpp',
                frontend/'include/zlio/map.hpp',frontend/'include/zlio/config.hpp',
                frontend/'test/integration/map_test.cpp',build/'upstream_map_test.log',a.output]
def identity_name(path):
    path=path.resolve()
    if path.is_relative_to(package):
        return 'small_point_lio/'+str(path.relative_to(package))
    if path.is_relative_to(upstream):
        return 'upstream/'+str(path.relative_to(upstream))
    if path.is_relative_to(build):
        return 'build/'+str(path.relative_to(build))
    return 'results/'+path.name
metadata={'date':'2026-10-28','scope':'isolated synthetic map microbenchmark; not full LIO or navigation',
          'upstream_commit':subprocess.check_output(['git','-C',str(upstream),'rev-parse','HEAD'],text=True).strip(),
          'upstream_remote':'https://github.com/mozihe/z-localization',
          'upstream_dirty':bool(subprocess.check_output(['git','-C',str(upstream),'status','--porcelain'],text=True)),
          'compiler':subprocess.check_output(['g++','--version'],text=True).splitlines()[0],
          'platform':platform.platform(),'command_flags':common[1:5],
          'input_std_m':.003,'query_std_m':.003,'query_covariance_diagonal_m2':.000009,
          'batch_size':1024,'passes':'three independent point/noise streams, seed+1000*pass; identical across methods',
          'seeds':[20261028,20261029,20261030],'isolated_process_per_row':True,
          'memory':'mallinfo2 uordblks+hblkhd delta after input/scratch allocation; includes container/node overhead; allocator-dependent',
          'matching':'native policies differ; good means <=5 deg normal error and <=0.02 m signed residual error against analytic surface',
          'upstream_map_tests':'6/6 passed; see upstream_map_test.log',
          'sha256':{identity_name(x):hashlib.sha256(x.read_bytes()).hexdigest() for x in identity_files}}
a.output.with_suffix('.metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
