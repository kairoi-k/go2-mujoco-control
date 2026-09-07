#!/usr/bin/env python3
"""Extract one immutable worker snapshot for offline same-code query replay."""
import argparse,hashlib,json,math
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('run',type=Path);p.add_argument('--runtime-sha',required=True);p.add_argument('--out',required=True,type=Path)
a=p.parse_args()
manifest=json.loads((a.run/'run_manifest.json').read_text());repo=manifest['repository']
if repo['git_commit']!=a.runtime_sha or str(repo['git_dirty']).lower()!='false':raise ValueError('not exact clean runtime')
log=a.run/'controller.log';records=[json.loads(s[len('JointSnapshot '):]) for s in log.read_text().splitlines() if s.startswith('JointSnapshot ')]
if len(records)!=1:raise ValueError('expected exactly one recorded snapshot')
r=records[0];m=r['terrain']
if r['schema']!='joint-shadow-snapshot-v1' or m['frame'] not in ('world','base_link'):raise ValueError('unsupported schema/frame')
if len(m['cells'])!=m['width']*m['height']:raise ValueError('cell shape')
expected=['known','height','has_bounds','min','max','age','slope','roughness','variance','nx','ny','nz']
if m['cell_fields']!=expected:raise ValueError('cell semantics')
values=['joint-shadow-snapshot-v1',r['id'],r['pattern'],r['time'],r['phase'],r['period'],r['duty'],r['command_vx'],r['base_yaw']]
for name,n in [('position',3),('quaternion_wxyz',4),('linear_velocity_world',3),('angular_velocity_body',3),('q',12),('dq',12)]:
 if len(r[name])!=n or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in r[name]):raise ValueError('unknown actual '+name)
 values+=r[name]
if len(r['measured_contact'])!=4 or not all(type(x) is bool for x in r['measured_contact']):raise ValueError('contact shape/type')
if type(r['measured_valid']) is not bool or type(r['touchdown_reference_valid']) is not bool:raise ValueError('validity type')
if len(r['touchdown_reference_feet_base'])!=4 or any(len(f)!=3 for f in r['touchdown_reference_feet_base']):raise ValueError('foot reference shape')
values +=[r['measured_valid']]+r['measured_contact']+[r['touchdown_reference_valid']]
for f in r['touchdown_reference_feet_base']:values+=f
values +=[m['frame'],m['source'],m['epoch'],m['registered'],m['map_sequence'],m['state_stamp'],m['map_stamp'],m['age'],m['width'],m['height'],m['resolution']]
values +=m['origin']+m['registration_position']+[m['registration_yaw']]+m['capture_position']+[m['capture_yaw']]
for c in m['cells']:
 if len(c)!=12:raise ValueError('cell shape')
 values+=c
# NaN is preserved only as missing map/optional-reference data, never imputed.
def token(v):
 if v is None:return 'nan'
 if isinstance(v,bool):return str(int(v))
 if isinstance(v,float):return format(v,'.17g')
 return str(v)
with a.out.open('x') as f:f.write('\n'.join(token(v) for v in values)+'\n')
meta={'schema':'joint-shadow-replay-extraction-v1','runtime_sha':a.runtime_sha,'run':str(a.run.resolve()),'controller_log_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),'extracted_sha256':hashlib.sha256(a.out.read_bytes()).hexdigest(),'scope':'exact recorded state/map and same-code query replay; clock epoch numbering restarts; no actuation'}
with Path(str(a.out)+'.meta.json').open('x') as f:json.dump(meta,f,indent=2);f.write('\n')
print(a.out)
