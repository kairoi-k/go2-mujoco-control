#!/usr/bin/env python3
"""Independent raw-CSV PD composition and declared MJCF ctrlrange audit."""
import argparse,collections,csv,hashlib,json,math,re
from pathlib import Path
import xml.etree.ElementTree as ET
ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);ap.add_argument('--runtime-sha',required=True);ap.add_argument('--model',required=True,type=Path);ap.add_argument('--out',required=True,type=Path);args=ap.parse_args()
manifest=json.loads((args.run/'run_manifest.json').read_text())
assert manifest['repository']['git_commit']==args.runtime_sha and manifest['repository']['git_dirty']=='false', 'runtime source mismatch'
classes={}
def defaults(node,inherited):
 motor=dict(inherited)
 for child in node:
  if child.tag=='motor':motor.update(child.attrib)
 classes[node.get('class','main')]=motor
 for child in node:
  if child.tag=='default':defaults(child,motor)
root=ET.parse(args.model).getroot();defaults(root.find('default'),{})
names=[f'{leg}_{joint}' for leg in ['FR','FL','RR','RL'] for joint in ['hip','thigh','calf']]
limits=[]
for i,node in enumerate(root.find('actuator')):
 assert node.tag=='motor' and node.get('name')==names[i], 'unsupported actuator ordering/type'
 attrs={**classes[node.attrib['class']],**node.attrib};lo,hi=map(float,attrs['ctrlrange'].split());assert math.isfinite(lo) and math.isfinite(hi) and lo<hi;limits.append((lo,hi))
assert len(limits)==12
by_tick=collections.defaultdict(collections.deque)
with (args.run/'data.csv').open() as f:
 for row in csv.DictReader(f):by_tick[round(float(row['state_tick_s'])*1000)].append(row)
records=[];max_error=0
for line in (args.run/'controller.log').read_text().splitlines():
 if not line.startswith('JointMotorEnvelope '):continue
 d=dict(re.findall(r'(\w+)=([^ ]+)',line));tick=int(d['source_tick']);assert d['input_valid']=='1';assert by_tick[tick],f'missing CSV tick {tick}'
 row=by_tick[tick].popleft();worst=None
 for i,name in enumerate(names):
  v=lambda suffix:float(row[name+'_'+suffix])
  pd=v('kp')*(v('q_target')-v('q_state'))+v('kd')*(v('dq_target')-v('dq_state'))
  requested=v('tau_ff')+pd;applied=min(limits[i][1],max(limits[i][0],requested));error=max(abs(requested-float(d[f'requested{i}'])),abs(applied-float(d[f'predicted_applied{i}'])))
  max_error=max(max_error,error);assert error<1e-5,(tick,name,error)
  witness=dict(motor=name,pd_nm=pd,tau_ff_nm=v('tau_ff'),requested_nm=requested,predicted_applied_nm=applied,saturation_nm=abs(requested-applied))
  if worst is None or witness['saturation_nm']>worst['saturation_nm']:worst=witness
 records.append(dict(state_time_s=tick*.001,command_time_s=float(row['cmd_time_s']),requested_speed_mps=float(row['velocity_command_requested_mps']),**worst))
summary={'schema':'raw-motor-envelope-audit-v1','runtime_sha':args.runtime_sha,'scope':'current consumed-state composition with declared MJCF limits, not later measured actuator force','input_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [args.run/'run_manifest.json',args.run/'data.csv',args.run/'controller.log',args.model,Path(__file__)]},'maximum_csv_vs_runtime_residual_nm':max_error,'windows':{}}
for name,rs in [('all',records),('state20_28',[r for r in records if 20<=r['state_time_s']<=28])]:
 saturated=[r for r in rs if r['saturation_nm']>1e-6];summary['windows'][name]={'samples':len(rs),'saturated_samples':len(saturated),'max_saturation_nm':max(r['saturation_nm'] for r in rs),'witnesses':saturated}
args.out.write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps({k:v for k,v in summary.items() if k not in ['input_sha256','windows']}));print({k:{x:y for x,y in v.items() if x!='witnesses'} for k,v in summary['windows'].items()})
