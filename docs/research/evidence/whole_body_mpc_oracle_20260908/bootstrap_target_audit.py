import hashlib, json, math, re
from pathlib import Path
import numpy as np
import mujoco
REPO = Path('/home/che/dev/go2-workspace/feat-stage-c-joint-planner')
EVID = REPO / 'docs/research/evidence/joint_execution_flat_20260908/attempt_0012'
SNAP = EVID / 'initial_gate_replay.txt'
SWING = EVID / 'swing_boundary_audit.txt'
MODEL = REPO / 'unitree_robots/go2/go2.xml'
OUT = Path('/tmp/go2-bootstrap-target-audit-20260908/bootstrap_target_audit.json')
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
def parse_vec(text):
    return np.array([float(x) for x in text], dtype=float)
def parse_swing(path):
    out = {}
    pat = re.compile(
        r'^SwingAudit leg=(\d+) .*?remaining_s=([^ ]+) '
        r'p0=\s*([^ ]+)\s+([^ ]+)\s+([^ ]+) .*?'
        r'p1=\s*([^ ]+)\s+([^ ]+)\s+([^ ]+) hermite_a0=', re.M)
    for line in path.read_text().splitlines():
        m = pat.search(line)
        if m:
            leg = int(m.group(1))
            out[leg] = {
                'remaining_s': float(m.group(2)),
                'p0_world': [float(m.group(i)) for i in (3,4,5)],
                'p1_world': [float(m.group(i)) for i in (6,7,8)],
            }
    return out
def parse_selected(path, proposal_id=881):
    out = {}
    pat = re.compile(
        r'^JointSelectedFoot id=(\d+) event=(\d+) leg=(\d+) '
        r'touchdown=([^ ]+) x=([^ ]+) y=([^ ]+) z=([^ ]+) '
        r'source_time=([^ ]+) selected_sequence=(\d+) history_used=(\d+)$')
    for line in path.read_text().splitlines():
        m = pat.search(line)
        if m and int(m.group(1)) == proposal_id:
            out[int(m.group(2))] = {
                'leg': int(m.group(3)),
                'touchdown_s': float(m.group(4)),
                'p1_surface_world': [float(m.group(5)), float(m.group(6)), float(m.group(7))],
                'source_time_s': float(m.group(8)),
                'selected_sequence': int(m.group(9)),
                'history_used': bool(int(m.group(10))),
            }
    return out
line = next(x for x in SNAP.read_text().splitlines() if x.startswith('JointSnapshot '))
source = json.loads(line[len('JointSnapshot '):])
swing = parse_swing(SWING)
selected = parse_selected(SWING)
if not all(leg in swing for leg in (1,2)):
    raise RuntimeError('missing raw SwingAudit leg 1/2')
if not all(event in selected for event in (0,1)):
    raise RuntimeError('missing raw JointSelectedFoot event 0/1')
# Static model evaluation: set recorded pose/joints, call mj_forward exactly once.
model = mujoco.MjModel.from_xml_path(str(MODEL))
data = mujoco.MjData(model)
pos = np.asarray(source['position'], dtype=float)
quat_raw = np.asarray(source['quaternion_wxyz'], dtype=float)
quat_norm = float(np.linalg.norm(quat_raw))
quat = quat_raw / quat_norm
data.qpos[:3] = pos
data.qpos[3:7] = quat
joint_names = [
    'FR_hip_joint','FR_thigh_joint','FR_calf_joint',
    'FL_hip_joint','FL_thigh_joint','FL_calf_joint',
    'RR_hip_joint','RR_thigh_joint','RR_calf_joint',
    'RL_hip_joint','RL_thigh_joint','RL_calf_joint']
for name, value in zip(joint_names, source['q']):
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        raise RuntimeError('missing joint '+name)
    data.qpos[model.jnt_qposadr[jid]] = float(value)
mujoco.mj_forward(model, data)
legacy_base = np.asarray(source['touchdown_reference_feet_base'], dtype=float)
base_yaw = float(source['base_yaw'])
cmd_v = float(source['command_vx']) * np.array([math.cos(base_yaw), math.sin(base_yaw), 0.0])
legs = {}
for event_index in (0,1):
    leg = selected[event_index]['leg']
    geom_name = ['FR','FL','RR','RL'][leg]
    site_name = ['FR_foot_contact','FL_foot_contact','RR_foot_contact','RL_foot_contact'][leg]
    gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    if gid < 0 or sid < 0:
        raise RuntimeError('missing geometry metadata')
    geom_body = int(model.geom_bodyid[gid]); site_body = int(model.site_bodyid[sid])
    geom_pos_local = np.asarray(model.geom_pos[gid], dtype=float)
    site_pos_local = np.asarray(model.site_pos[sid], dtype=float)
    geom_world = np.asarray(data.geom_xpos[gid], dtype=float)
    site_world = np.asarray(data.site_xpos[sid], dtype=float)
    old_world = pos + quat_norm * 0.0
    # Explicit quaternion rotation in the same order as Eigen::Quaterniond(w,x,y,z).
    w,x,y,z = quat
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    old_world = pos + R @ legacy_base[leg]
    dt = float(selected[event_index]['touchdown_s'] - source['time'])
    com_delta = cmd_v * dt
    geom_site = geom_world - site_world
    p1 = np.asarray(swing[leg]['p1_world'], dtype=float)
    p0 = np.asarray(swing[leg]['p0_world'], dtype=float)
    candidate_xy = p1[:2] - (old_world + geom_site + com_delta)[:2]
    delta = p1 - p0
    target_relative_to_source_geom = old_world - p0
    legs[str(leg)] = {
        'event_index': event_index,
        'touchdown_s': float(selected[event_index]['touchdown_s']),
        'delta_t_s': dt,
        'legacy_target_base': legacy_base[leg].tolist(),
        'legacy_target_world': old_world.tolist(),
        'source_geom_center_world_mj_forward': geom_world.tolist(),
        'source_foot_site_world_mj_forward': site_world.tolist(),
        'geom_minus_site_world': geom_site.tolist(),
        'geom_minus_site_norm_m': float(np.linalg.norm(geom_site)),
        'source_p0_from_raw_swing_audit': p0.tolist(),
        'raw_p0_vs_mj_forward_geom_error_m': float(np.linalg.norm(p0-geom_world)),
        'command_velocity_world': cmd_v.tolist(),
        'com_forward_delta': com_delta.tolist(),
        'candidate_xy_offset_inferred': candidate_xy.tolist(),
        'candidate_xy_offset_expected_index0': [0.0,0.0],
        'event_p1_collision_center_raw': p1.tolist(),
        'event_displacement_p1_minus_p0': delta.tolist(),
        'decomposition_xy': {
            'legacy_world_minus_source_geom': target_relative_to_source_geom[:2].tolist(),
            'geom_minus_site': geom_site[:2].tolist(),
            'com_forward': com_delta[:2].tolist(),
            'candidate_xy_offset': candidate_xy.tolist(),
            'sum': (target_relative_to_source_geom[:2] + geom_site[:2] + com_delta[:2] + candidate_xy).tolist(),
            'observed_p1_minus_p0': delta[:2].tolist(),
        },
        'geometry_metadata': {
            'geom_name': geom_name,
            'site_name': site_name,
            'geom_id': gid,
            'site_id': sid,
            'geom_body_id': geom_body,
            'site_body_id': site_body,
            'geom_pos_local': geom_pos_local.tolist(),
            'site_pos_local': site_pos_local.tolist(),
            'same_parent_body': geom_body == site_body,
        },
        'z_note': 'candidate z is terrain surface height plus radius*normal; it is not the legacy target z plus the xy decomposition',
    }
result = {
    'schema': 'go2-bootstrap-target-audit-v1',
    'scope': 'read-only static MJCF forward at recorded source pose; no mj_step/dynamics rollout',
    'mj_step_called': False,
    'mj_forward_calls': 1,
    'source': {
        'proposal_id': source['id'],
        'source_time_s': source['time'],
        'phase': source['phase'],
        'period_s': source['period'],
        'duty': source['duty'],
        'command_vx_mps': source['command_vx'],
        'base_yaw_rad': source['base_yaw'],
        'body_position_world': source['position'],
        'body_quaternion_wxyz_raw': source['quaternion_wxyz'],
        'body_quaternion_norm': quat_norm,
        'body_quaternion_wxyz_used': quat.tolist(),
        'touchdown_reference_feet_base': source['touchdown_reference_feet_base'],
        'measured_contact': source['measured_contact'],
    },
    'model': {
        'path': str(MODEL),
        'mujoco_version': mujoco.__version__,
        'nq': int(model.nq), 'nv': int(model.nv), 'nu': int(model.nu),
    },
    'candidate_formula': 'p1_xy = (body_pos + body_quat*legacy_target_base) + (geom_center_world - foot_site_world) + command_v_world*(touchdown-source_time) + xy_offsets[index]',
    'legs': legs,
    'raw_input_sha256': {
        'initial_gate_replay.txt': sha(SNAP),
        'swing_boundary_audit.txt': sha(SWING),
        'go2.xml': sha(MODEL),
    },
}
OUT.write_text(json.dumps(result, indent=2) + '\n')
print(OUT)
for leg, item in legs.items():
    d = item['decomposition_xy']
    print('leg', leg, 'legacy-source-mm', [1000*x for x in d['legacy_world_minus_source_geom']], 'geom-site-mm', [1000*x for x in d['geom_minus_site']], 'com-mm', [1000*x for x in d['com_forward']], 'candidate-mm', [1000*x for x in d['candidate_xy_offset']], 'sum-mm', [1000*x for x in d['sum']], 'obs-mm', [1000*x for x in d['observed_p1_minus_p0']])
