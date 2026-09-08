import math,pathlib,json,hashlib
root=pathlib.Path.cwd();source=root/'docs/research/evidence/joint_execution_flat_20260908/attempt_0012/initial_source.txt';queryfile=root/'example/cpp/experiments/_runs/observed_collision_node_20260908_0001/result.json'
t=source.read_text().split();at=64
assert t[0]=='joint-shadow-snapshot-v2'
def terrain():
 global at
 m={'frame':t[at],'sequence':int(t[at+4]),'registered':int(t[at+3]),'width':int(t[at+8]),'height':int(t[at+9]),'resolution':float(t[at+10]),'origin':list(map(float,t[at+11:at+13])),'registration':list(map(float,t[at+13:at+16])),'yaw':float(t[at+16])}
 at+=21+12*m['width']*m['height'];return m
latest=terrain();header=t[at:at+6];at+=6;models=[terrain() for _ in range(int(header[5]))];assert at==len(t)
q=json.loads(queryfile.read_text())['full_body_query'];e=[]
for m in models:
 dx=q['world_x']-m['registration'][0];dy=q['world_y']-m['registration'][1];c=math.cos(m['yaw']);s=math.sin(m['yaw']);local=[c*dx+s*dy,-s*dx+c*dy];r=max(q['radius_m'],.5*m['resolution']);axes=[]
 for axis,n in enumerate([m['width'],m['height']]):
  lo=m['origin'][axis];hi=lo+n*m['resolution'];qlo=local[axis]-r;qhi=local[axis]+r;olo=max(qlo,lo);ohi=min(qhi,math.nextafter(hi,-math.inf));quotient=(ohi-lo)/m['resolution'];index=math.floor(quotient)
  axes.append({'axis':axis,'map_min':lo,'map_max':hi,'query_min':qlo,'query_max':qhi,'overlap_min':olo,'overlap_max':ohi,'nextafter_below_map_max':ohi<hi,'inbounds_overlap_max':lo<=ohi<hi,'cell_quotient':quotient,'cell_index':index,'dimension':n,'index_valid':index<n,'hex':{'map_max':hi.hex(),'overlap_max':ohi.hex(),'overlap_minus_origin':(ohi-lo).hex(),'resolution':m['resolution'].hex()}})
 e.append({'sequence':m['sequence'],'model':m,'local':local,'covers_patch':all(a['query_min']>=a['map_min'] and a['query_max']<a['map_max'] for a in axes),'axes':axes,'first_reject':'world_terrain_snapshot.h:461 upper-overlap CellIndex false; terrain_model.h:184 ix<width && iy<height false' if not all(a['index_valid'] for a in axes) else 'not reproduced'})
report={'scope':'read-only binary64 arithmetic replay of actual source metadata; no physics, no threshold or production change','query':q,'captures':e,'first_invalid_label_site':'world_terrain_snapshot.h:464 sets kInvalidQuery after overlap CellIndex failure','sources':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [source,queryfile,root/'example/cpp/terrain/stage_c/world_terrain_snapshot.h',root/'example/cpp/terrain/terrain_model.h',pathlib.Path(__file__)]}}
pathlib.Path(__file__).with_name('result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(e,indent=2))
