#!/usr/bin/env python3
"""Read-only query failure and captured-cell accounting; no feasibility claim."""
import argparse,collections,hashlib,json,math
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--runtime-sha',required=True);p.add_argument('--out',type=Path,required=True)
a=p.parse_args();manifest=json.loads((a.run/'run_manifest.json').read_text());repo=manifest['repository']
if repo['git_commit']!=a.runtime_sha or str(repo['git_dirty']).lower()!='false':raise ValueError('exact clean source required')
log=a.run/'controller.log';lines=log.read_text().splitlines()
queries=[dict(token.split('=',1) for token in s.split()[1:]) for s in lines if s.startswith('JointTerrainQuery ')]
snapshots=[json.loads(s[len('JointSnapshot '):]) for s in lines if s.startswith('JointSnapshot ')]
if len(snapshots)!=1:raise ValueError('one exact snapshot required')
snap=snapshots[0];m=snap['terrain'];w,h=m['width'],m['height'];cells=m['cells']
sources=[dict(token.split('=',1) for token in s.split()[1:]) for s in lines if s.startswith('JointTerrainSource ')]
source=[s for s in sources if int(s['id'])==snap['id']]
if len(source)!=1 or int(source[0]['sequence'])!=m['map_sequence']:raise ValueError('source registration binding')
source=source[0]
if len(source['mask'])!=int(source['width'])*int(source['height']):raise ValueError('source mask shape')
if len(cells)!=w*h:raise ValueError('map shape')
mask=[''.join('K' if cells[y*w+x][0] else '?' for x in range(w)) for y in range(h)]
checks=[]
for q in queries:
 if int(q['id'])!=snap['id']:continue
 x,y,r=map(float,(q['local_x'],q['local_y'],q['radius']));res=m['resolution'];r=max(r,.5*res)
 if not all(math.isfinite(v) for v in (x,y,r)):continue
 x0=math.floor((x-r-m['origin'][0])/res);x1=math.floor((x+r-m['origin'][0])/res)
 y0=math.floor((y-r-m['origin'][1])/res);y1=math.floor((y+r-m['origin'][1])/res)
 queried=[]
 for iy in range(y0,y1+1):
  for ix in range(x0,x1+1):
   inside=0<=ix<w and 0<=iy<h;c=cells[iy*w+ix] if inside else None
   queried.append({'ix':ix,'iy':iy,'inside':inside,'known':c[0] if c else False,'age':c[5] if c else None})
 known=sum(c['known'] for c in queried);outside=sum(not c['inside'] for c in queried)
 check={'query':q,'cells':queried,'independent_known':known,'independent_total':len(queried),'independent_outside':outside}
 if q['reason']!='outside':
  check['reported_counts_match']=known==int(q['known']) and len(queried)==int(q['total']) and outside==int(q['outside'])
  if not check['reported_counts_match']:raise ValueError('independent footprint accounting disagrees')
 checks.append(check)
result={'schema':'joint-query-accounting-v1','runtime_sha':a.runtime_sha,'scope':'query coverage only; no optimizer or physical acceptance',
 'controller_log_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),'query_count':len(queries),
 'reason_counts':dict(collections.Counter(q['reason'] for q in queries)),
 'stage_reason_counts':dict(collections.Counter(q['stage']+':'+q['reason'] for q in queries)),
 'snapshot_id':snap['id'],'snapshot_time':snap['time'],'map_sequence':m['map_sequence'],
 'capture_source':source,'capture_source_known':source['mask'].count('K'),'map_known':sum(c[0] for c in cells),'map_total':len(cells),'row_masks_low_to_high_y':mask,'snapshot_query_checks':checks}
with a.out.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
print(json.dumps({k:result[k] for k in ('query_count','stage_reason_counts','map_known','map_total')}))
