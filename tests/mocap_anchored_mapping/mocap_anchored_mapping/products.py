"""Native PCD validation and unchanged existing raster projection; no XYZ conversion."""
from pathlib import Path
import subprocess
import sys
import numpy as np
from .core import checked, import_projector, load, projection_args, save, sha, update_products


def read_pcd(path, cfg):
    lib = import_projector(cfg); header, offset = lib._parse_header(Path(path))
    names = header['FIELDS'].split(); sizes = list(map(int, header['SIZE'].split()))
    counts = list(map(int, header.get('COUNT', ' '.join(['1']*len(names))).split()))
    types = header['TYPE'].split()
    if not len(names)==len(sizes)==len(counts)==len(types): raise ValueError('inconsistent_PCD_fields')
    dtype = np.dtype([(n, lib._numpy_type(t,s)) if c==1 else (n,lib._numpy_type(t,s),(c,))
                      for n,s,t,c in zip(names,sizes,types,counts)])
    n = int(header['POINTS'])
    with Path(path).open('rb') as stream:
        stream.seek(offset)
        if header['DATA'].lower() == 'binary':
            payload = stream.read()
            if len(payload)!=dtype.itemsize*n: raise ValueError('truncated_or_extra_PCD_payload')
            data = np.frombuffer(payload,dtype=dtype).copy()
        elif header['DATA'].lower() == 'ascii':
            a = np.loadtxt(stream,ndmin=2)
            if a.shape!=(n,sum(counts)): raise ValueError('inconsistent_ASCII_PCD_count')
            data = np.empty(n,dtype=dtype); column = 0
            for name,c in zip(names,counts):
                data[name] = a[:,column] if c==1 else a[:,column:column+c]; column += c
        else: raise ValueError('binary_compressed unsupported; native writer uses binary')
    if not all(name in names and data[name].ndim==1 and data[name].dtype.kind=='f' for name in ('x','y','z')):
        raise ValueError('Need scalar floating-point XYZ fields')
    return header, data


def xyz(data): return np.column_stack([data[n] for n in ('x','y','z')]).astype(float)


def native_cloud(run):
    run=Path(run).resolve();cfg,_=checked(run,True);p=run/'raw/native_F.pcd';receipt=load(run/'raw/native_receipt.json')
    if receipt['frame']!='map=native_F0' or receipt['transform_count']!=0 or receipt['run_id']!=run.name or receipt['sha256']!=sha(p):
        raise ValueError('Native map provenance mismatch')
    if receipt['calibration_sha256']!=sha(run/'calibration.yaml'):raise ValueError('Map/calibration binding mismatch')
    return cfg,p


def inspect(run):
    cfg,p=native_cloud(run);_,data=read_pcd(p,cfg);a=xyz(data);a=a[np.isfinite(a).all(axis=1)]
    if not len(a):raise ValueError('No finite map points')
    return dict(frame='map=native_F0',points=len(a),bounds_m=[a.min(axis=0).tolist(),a.max(axis=0).tolist()],
        point_coordinates_changed=False,projection='Uses the original native-frame slice heights and denoising switches')


def project(run):
    run=Path(run).resolve();cfg,p=native_cloud(run)
    if any((run/name).exists() for name in ['grid/map.yaml','grid/map.pgm','grid/projection.json']):raise ValueError('Grid already exists; no overwrite')
    before=sha(p);args=projection_args(cfg,p,run/'grid/map')
    command=[sys.executable,str(Path(cfg['project_root'])/'make_2d_map_from_pcd.py'),*args]
    save(run/'grid/projection_inputs.json',dict(frame='map=native_F0',command=command,heights_source='unchanged pcd_to_2d_map.sh',
        source_sha256=sha(Path(cfg['project_root'])/'pcd_to_2d_map.sh'),PCD_XYZ_transform_count=0))
    with (run/'grid/projection.log').open('x') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    if sha(p)!=before:raise ValueError('Projection changed source PCD')
    grid=load(run/'grid/map.yaml')
    from PIL import Image
    w,h=Image.open(run/'grid/map.pgm').size;r=grid['resolution'];o=grid['origin']
    result=dict(frame='map=native_F0',pcd_sha256=before,yaml_sha256=sha(run/'grid/map.yaml'),image_sha256=sha(run/'grid/map.pgm'),
        point_coordinates_changed=False,PCD_XYZ_transform_count=0,T_map_F0=np.eye(4).tolist(),
        yaml_origin=o,resolution_m=r,width=w,height=h,xy_bounds_m=[o[:2],[o[0]+w*r,o[1]+h*r]],
        slice_heights_changed_for_mocap=False,post_3D_transform='identity',origin_source='new native point cloud bounds; old map origin not copied')
    save(run/'grid/projection.json',result);update_products(run);return result


def check_grid(run):
    cfg,p=native_cloud(run);run=Path(run);r=load(run/'grid/projection.json')
    if r['pcd_sha256']!=sha(p) or r['PCD_XYZ_transform_count']!=0:raise ValueError('Grid native map provenance mismatch')
    for name,key in [('grid/map.yaml','yaml_sha256'),('grid/map.pgm','image_sha256')]:
        if sha(run/name)!=r[key]:raise ValueError('Grid changed after generation')
    return cfg
