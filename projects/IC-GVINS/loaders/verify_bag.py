#!/usr/bin/env python3
"""Verify IC-GVINS UTC/FRD inputs, native images and antenna NavSatFix payloads."""
import argparse
from collections import Counter
import datetime as dt
import json
from pathlib import Path

import h5py
import numpy as np
import rosbag
import rospy
import yaml
from extract_est import lla2ecef


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bag',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    prep=json.loads(Path(str(a.bag)+'.preparation.json').read_text())
    assert prep['downscale']==1 and a.bag.stat().st_size==prep['bag_bytes']
    data=Path(prep['data_dir'])
    report=dict(bag=str(a.bag.resolve()),bag_bytes=a.bag.stat().st_size,bag_mtime_ns=a.bag.stat().st_mtime_ns,topics={},checks={})
    stamps,samples={},{}
    with rosbag.Bag(str(a.bag)) as bag:
        for topic,meta in bag.get_type_and_topic_info().topics.items():
            ts=np.array(sorted(e.time.to_sec() for index in bag._get_indexes(bag._get_connections([topic])) for e in index))
            assert len(ts)==meta.message_count and np.isfinite(ts).all() and np.all(np.diff(ts)>0)
            stamps[topic]=ts
            report['topics'][topic]=dict(count=len(ts),first_utc=float(ts[0]),last_utc=float(ts[-1]))
            samples[topic]=[]
            for t in ts[[0,len(ts)//2,-1]]:
                _,msg,record=next(bag.read_messages(topics=[topic],start_time=rospy.Time.from_sec(float(t)-1e-6)))
                assert abs(record.to_sec()-t)<1e-6 and abs(msg.header.stamp.to_sec()-t)<1e-6
                samples[topic].append((t,msg))
        assert set(stamps)=={'/imu0','/cam0/image_raw','/gnss0'}
        raw_positions={}
        for line in Path(prep['rtklib_pos']).read_text().splitlines():
            if not line.strip() or line.startswith('%'):continue
            row=line.split()
            t=dt.datetime.strptime(' '.join(row[:2]),'%Y/%m/%d %H:%M:%S.%f').replace(tzinfo=dt.timezone.utc).timestamp()-18
            if prep['start_utc']<=t<=prep['gnss_end_utc']:
                raw_positions[t]=(np.array(list(map(float,row[2:5]))),int(row[5]))
        assert len(raw_positions)==4040
        report['rtklib_quality_counts_in_window']=dict(Counter(q for _,q in raw_positions.values()))
        eligible={t:v for t,v in raw_positions.items() if str(v[1]) in prep['std_by_quality']}
        assert len(stamps['/gnss0'])==len(eligible)==prep['counts']['gnss']
        assert np.array_equal(stamps['/gnss0'],np.array(sorted(eligible)))
        report['gnss_epochs_excluded_by_quality']=len(raw_positions)-len(eligible)
        max_error=0.0
        for _,msg,record in bag.read_messages(topics=['/gnss0']):
            t=record.to_sec()
            assert t==msg.header.stamp.to_sec() and t in raw_positions
            xyz,q=eligible[t]
            max_error=max(max_error,float(np.linalg.norm(lla2ecef(msg.latitude,msg.longitude,msg.altitude)-xyz)))
            h,v={1:(.05,.1),2:(.5,1)}[q]
            assert np.allclose(msg.position_covariance,[h*h,0,0,0,h*h,0,0,0,v*v],atol=1e-14,rtol=0)
        assert max_error<1e-5
        report['max_gnss_antenna_ecef_roundtrip_m']=max_error
    with h5py.File(data/'camera_data/images.h5','r') as h5:
        group=h5['port']
        keys=[k for k in sorted(group) if prep['start_utc']<=float(group[k].attrs['tstamp'])<=prep['end_utc']]
        ts=np.array([float(group[k].attrs['tstamp']) for k in keys])
        assert len(keys)==40372 and np.allclose(stamps['/cam0/image_raw'],ts,atol=1e-6,rtol=0)
        for t,msg in samples['/cam0/image_raw']:
            i=int(np.argmin(abs(ts-t)))
            assert (msg.width,msg.height,msg.step,msg.encoding)==(2048,732,2048,'mono8')
            assert np.array_equal(np.frombuffer(msg.data,np.uint8).reshape(732,2048),group[keys[i]][()])
    raw=np.loadtxt(data/'lord_imu.log',delimiter=',',comments='#')
    ts=315964800+raw[:,0]*604800+raw[:,1]+raw[:,2]-18
    keep=(ts>=prep['start_utc']-5)&(ts<=prep['end_utc'])
    assert np.allclose(stamps['/imu0'],ts[keep],atol=1e-6,rtol=0)
    for t,msg in samples['/imu0']:
        row=raw[np.argmin(abs(ts-t))]
        got=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,
             msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
        assert np.allclose(got,row[[4,3,5,7,6,8]]*[1,-1,1,1,-1,1],atol=1e-12,rtol=0)
    config=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/icgvins_texcup.yaml').read_text())
    assert config['cam0']['resolution']==[2048,732]
    assert np.allclose(config['antlever'],[-.052,-.610,-.01])
    assert np.allclose(config['cam0']['q_b_c'],[.5,.5,.5,.5])
    assert np.allclose(config['cam0']['t_b_c'],[.011,-.303,-.033])
    mono=yaml.safe_load((data/'camera-calibration-mono/texcup_port-camchain.yaml').read_text())['cam0']
    assert np.allclose(config['cam0']['intrinsic'],mono['intrinsics'],atol=1e-7,rtol=0)
    assert np.allclose(config['cam0']['distortion'],mono['distortion_coeffs'],atol=1e-7,rtol=0)
    report['checks']=dict(all_topic_times_monotonic=True,all_gnss_times_gpst_minus_18=True,
        all_gnss_positions_are_unshifted_rtklib_antenna=True,all_gnss_covariances_are_variances=True,
        all_image_times_match_hdf5=True,sampled_native_pixels_match=True,
        all_imu_times_match_raw_utc=True,sampled_imu_rates_match_frd=True,frd_lever_and_camera_extrinsic=True)
    a.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
