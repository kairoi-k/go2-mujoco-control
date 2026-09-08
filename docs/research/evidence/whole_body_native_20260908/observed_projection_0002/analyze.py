"""Read-only exact current-node primitive XY extents; no forward/step/sweep."""
import fcntl,hashlib,itertools,json,math,pathlib,subprocess
import mujoco,numpy as np
base=pathlib.Path(__file__).resolve().parent;repo=pathlib.Path.cwd();prior=repo/'example/cpp/experiments/_runs/observed_collision_node_20260908_0001'
sha=lambda p:hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(pathlib.Path(p).read_text())
def write(name,value):
 with (base/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False,default=lambda v:v.item() if isinstance(v,np.generic) else str(v))
old=read(prior/'result.json');bound=read(prior/'manifest.json')['bound_sources'];command=read(prior/'command.json');snapshot_path=pathlib.Path(command[1]);robot_path=pathlib.Path(command[2])
packet=read(repo/'example/cpp/experiments/_runs/whole_body_trajectory_packet_20260908_0002/manifest.json')
for name,digest in packet['model']['recursive_hashes'].items():
 if sha(name)!=digest:raise ValueError('canonical model closure mismatch '+name)
for p in (snapshot_path,robot_path):
 if sha(p)!=bound[str(p)]:raise ValueError('original node input mismatch')
tokens=snapshot_path.read_text().split();index=0
def word():
 global index
 value=tokens[index];index+=1;return value
def integer():return int(word())
def number():return float(word())
def array(n):return [number() for _ in range(n)]
def terrain():
 t={'frame':word(),'source':integer(),'epoch':integer(),'registered':integer(),'sequence':integer(),'state_stamp':number(),'map_stamp':number(),'age':number(),'width':integer(),'height':integer(),'resolution':number(),'origin':array(2),'registration':array(3),'yaw':number(),'capture_position':array(3),'capture_yaw':number()}
 t['cells']=[{'known':integer(),'height':number(),'has_bounds':integer(),'minimum':number(),'maximum':number(),'age':number(),'slope':number(),'roughness':number(),'variance':number(),'normal':array(3)} for _ in range(t['width']*t['height'])];return t
assert word()=='joint-shadow-snapshot-v2';snapshot_id=integer();pattern=integer();header=array(6);obs_qv=array(37);measured=[integer() for _ in range(5)];target_valid=integer();targets=array(12);latest=terrain();epoch=integer();stamp=number();stationary=integer();height_tol=number();normal_dot=number();captures=[terrain() for _ in range(integer())];assert index==len(tokens)==old['input_tokens'] and stamp==old['observation_time'] and snapshot_id==old['snapshot_id']
assert [c['sequence'] for c in captures]==sorted([c['sequence'] for c in captures],reverse=True)
def extent(kind,size,rotation):
 if kind==mujoco.mjtGeom.mjGEOM_SPHERE:return np.full(2,size[0]),'sphere r'
 if kind==mujoco.mjtGeom.mjGEOM_BOX:return abs(rotation[:2])@size,'box sum(abs(R_ij)*halfsize_j)'
 if kind==mujoco.mjtGeom.mjGEOM_CAPSULE:return size[1]*abs(rotation[:2,2])+size[0],'capsule halfsegment*abs(axis_i)+radius'
 if kind==mujoco.mjtGeom.mjGEOM_CYLINDER:return size[1]*abs(rotation[:2,2])+size[0]*np.sqrt(np.sum(rotation[:2,:2]**2,axis=1)),'cylinder halfheight*abs(axis_i)+radius*sqrt(R_i0^2+R_i1^2)'
 if kind==mujoco.mjtGeom.mjGEOM_ELLIPSOID:return np.sqrt(np.sum((rotation[:2]*size)**2,axis=1)),'ellipsoid sqrt(sum((R_ij*size_j)^2))'
 raise ValueError('unsupported collidable geometry '+str(kind))
def inspect_cells(c,lower,upper):
 origin=np.asarray(c['origin']);map_upper=origin+np.asarray([c['width'],c['height']])*c['resolution'];lo=np.maximum(lower,origin);hi=np.minimum(upper,np.nextafter(map_upper,-np.inf));records=[]
 if np.any(hi<lo):return {'cells':[],'fresh_complete':False}
 begin=np.floor((lo-origin)/c['resolution']).astype(int);end=np.floor((hi-origin)/c['resolution']).astype(int)
 for iy in range(begin[1],end[1]+1):
  for ix in range(begin[0],end[0]+1):
   cell=c['cells'][iy*c['width']+ix];age=cell['age']+stamp-c['state_stamp'];normal=np.asarray(cell['normal']);norm=np.linalg.norm(normal);low=cell['minimum'] if cell['has_bounds'] else cell['height'];high=cell['maximum'] if cell['has_bounds'] else cell['height'];finite=all(math.isfinite(v) for v in (cell['height'],cell['slope'],cell['roughness'],cell['variance'],low,high,norm)) and low<=high and norm>1e-9 and abs(norm-1)<=1e-5
   fresh=math.isfinite(age) and cell['age']>=-1e-6 and age<=old['max_cell_age_s']+1e-6
   records.append({'ix':int(ix),'iy':int(iy),'known':bool(cell['known']),'fresh':fresh,'finite_geometry':finite,'age_at_observation_s':age if math.isfinite(age) else None})
 inside=bool(np.all(lower>=origin) and np.all(upper<map_upper))
 return {'cells':records,'fresh_complete':inside and all(x['known'] and x['fresh'] and x['finite_geometry'] for x in records),'inside_map':inside,'cell_count':len(records),'unknown_count':sum(not x['known'] for x in records),'stale_count':sum(not x['fresh'] for x in records),'nonfinite_count':sum(not x['finite_geometry'] for x in records)}
with open('/tmp/go2_mujoco_experiment.lock','a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 model=mujoco.MjModel.from_xml_path(str(robot_path));data=mujoco.MjData(model);data.qpos[:]=old['qpos'];data.qvel[:]=old['qvel'];data.time=old['observation_time'];mujoco.mj_kinematics(model,data)
 geoms=[];max_center_delta=0.
 for recorded in old['collidable_geoms']:
  gid=recorded['geom_id'];center=data.geom_xpos[gid];R=data.geom_xmat[gid].reshape(3,3);size=model.geom_size[gid];kind=int(model.geom_type[gid]);radius=float(model.geom_rbound[gid]);max_center_delta=max(max_center_delta,float(max(abs(center-np.asarray(recorded['center_world'])))));world_extent,formula=extent(kind,size,R);checks=[]
  for c in captures:
   cy,sy=math.cos(c['yaw']),math.sin(c['yaw']);world_to_capture=np.array([[cy,sy,0],[-sy,cy,0],[0,0,1.]])
   local_center=world_to_capture@(center-np.asarray(c['registration']));local_rotation=world_to_capture@R;half,formula=extent(kind,size,local_rotation);lower=local_center[:2]-half;upper=local_center[:2]+half;map_lower=np.asarray(c['origin']);map_upper=map_lower+np.asarray([c['width'],c['height']])*c['resolution'];margins=np.r_[lower-map_lower,map_upper-upper];sphere_margins=np.r_[local_center[:2]-radius-map_lower,map_upper-(local_center[:2]+radius)]
   checked=inspect_cells(c,lower,upper)
   checks.append({'sequence':c['sequence'],'frame':'capture-heading registered base_link','registration_world':c['registration'],'registration_yaw_rad':c['yaw'],'map_lower_xy_m':map_lower.tolist(),'map_upper_xy_m_exclusive':map_upper.tolist(),'geom_center_capture_m':local_center.tolist(),'geom_rotation_capture':local_rotation.tolist(),'exact_halfextent_xy_m':half.tolist(),'exact_min_xy_m':lower.tolist(),'exact_max_xy_m':upper.tolist(),'signed_boundary_margins_xmin_ymin_xmax_ymax_m':margins.tolist(),'minimum_signed_margin_m':float(min(margins)),'required_extension_m':max(0.,-float(min(margins))),'old_rbound_square_minimum_margin_m':float(min(sphere_margins)),**checked})
  geoms.append({'geom_id':gid,'body':recorded['body'],'type':mujoco.mjtGeom(kind).name,'size':size.tolist(),'center_world_m':center.tolist(),'rotation_world':R.tolist(),'rbound_m':radius,'world_halfextent_xy_m':world_extent.tolist(),'formula':formula,'captures':checks})
report={'scope':'exact analytic primitive projection extents at one recorded node; capture-cell rectangular enclosure audit only, not existing API admission/continuous swept coverage/surface certificate','source_sha':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'input_tokens':len(tokens),'geometry_count':len(geoms),'canonical_model_closure_hashes_verified':len(packet['model']['recursive_hashes']),'robot_center_reproduction_delta_m':max_center_delta,'max_cell_age_s_unchanged':old['max_cell_age_s'],'height_conflict_tolerance_unchanged':height_tol,'minimum_normal_dot_unchanged':normal_dot,'no_forward_or_step':True,'history_conflict_recertified':False,'all_geoms_have_one_fresh_complete_rectangular_capture':all(any(c['fresh_complete'] for c in g['captures']) for g in geoms),'geoms':geoms,'input_hashes':{str(snapshot_path):sha(snapshot_path),str(robot_path):sha(robot_path),str(prior/'result.json'):sha(prior/'result.json'),str(prior/'manifest.json'):sha(prior/'manifest.json')}}
write('result.json',report)
summary={k:v for k,v in report.items() if k not in ('geoms','input_hashes')};summary['target_geoms']=[dict(g,captures=[{k:v for k,v in c.items() if k not in ('cells','geom_rotation_capture')} for c in g['captures']]) for g in geoms if g['geom_id'] in (25,49)];summary['geometry_failures']=[{'geom_id':g['geom_id'],'body':g['body'],'captures':[{k:c[k] for k in ('sequence','minimum_signed_margin_m','fresh_complete','unknown_count','stale_count','nonfinite_count')} for c in g['captures']]} for g in geoms if not any(c['fresh_complete'] for c in g['captures'])]
write('summary.json',summary)
write('manifest.json',{'source_sha':report['source_sha'],'scope':report['scope'],'input_hashes':report['input_hashes'],'files':{p.name:sha(p) for p in base.iterdir() if p.is_file() and p.name not in ('stdout.json','stderr.txt')}})
print(json.dumps(summary,indent=2))
