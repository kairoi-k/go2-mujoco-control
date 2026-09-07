#!/usr/bin/env python3
"""Independent actual-actuator check on the retained F02 reference, not a new run."""
import csv,hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parents[4]
p=root/'example/cpp/experiments/_runs/wbc_bias_step_7a8ffc6_20260907_0001/contact_ground_truth.csv'
with p.open() as f:rows=list(csv.DictReader(f))
keys=[k for k in rows[0] if k.startswith('full_qfrc_actuator_qcoord_') and k.endswith('_joint')]
assert len(keys)==12
result={'schema':'f02-actuator-reference-v1','runtime_sha':'7a8ffc6b9269490d7e46c3adfbd2e13ed8609dc6',
        'raw_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),
        'interpretation':'Actual joint actuator force versus 35 Nm WBC feedforward bound; NOT a physical motor-limit violation claim.',
        'intervals':{}}
for name,start,end in [('approach',22.416,23.216),('interaction',23.216,24.468)]:
    a=[r for r in rows if start<=float(r['time_s'])<=end]
    result['intervals'][name]={'inclusive_time_s':[start,end],'rows':len(a),
        'max_abs_actuator_joint_torque_Nm':max(abs(float(r[k])) for r in a for k in keys),
        'rows_any_joint_above_35_05_Nm':sum(any(abs(float(r[k]))>35.05 for k in keys) for r in a)}
print(json.dumps(result,indent=2,allow_nan=False))
