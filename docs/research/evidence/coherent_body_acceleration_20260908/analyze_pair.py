#!/usr/bin/env python3
"""Paired fixed-source short replay metrics; never traversal acceptance."""
import argparse,csv,json,math,pathlib
p=argparse.ArgumentParser();p.add_argument('attempt');p.add_argument('--out',required=True);a=p.parse_args()
root=pathlib.Path(a.attempt);report={'scope':'one fixed source, 0.2s privileged model replay; not B1','modes':{}}
def peak(rows,key):
 values=[float(r[key]) for r in rows if math.isfinite(float(r[key]))]
 return max(map(abs,values)) if values else None
for mode in ['baseline','coherent']:
 lines=(root/mode/'feedback.csv').read_text().splitlines()
 rows=list(csv.DictReader(l for l in lines if not l.startswith('#')))
 applied=[r for r in rows if r['status']=='applied']
 norms=[math.sqrt(sum(float(r[f'foot_error{leg}_{axis}'])**2 for axis in 'xyz')) for r in applied for leg in range(4)]
 if any(not math.isfinite(x) for x in norms):raise ValueError('nonfinite applied foot error')
 report['modes'][mode]={'state_rows':len(rows),'applied_steps':len(applied),
 'max_foot_error_m':max(norms),'max_com_error_m':peak(applied,'com_error_m'),
 'max_roll_deg':math.degrees(peak(rows,'roll_rad')),'max_pitch_deg':math.degrees(peak(rows,'pitch_rad')),
 'max_nonfoot_contact_count':peak(rows,'nonfoot_contact_count'),
 'max_motor_saturation_nm':peak(applied,'max_motor_saturation_nm'),
 'max_certificate_force_residual_n':peak(applied,'certificate_force_residual_n'),
 'max_certificate_moment_residual_nm':peak(applied,'certificate_moment_residual_nm'),
 'planned_geom_mask_mismatch_rows':sum(r['nominal_contact_mask']!=r['mujoco_geom_contact_mask'] for r in applied)}
pathlib.Path(a.out).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
