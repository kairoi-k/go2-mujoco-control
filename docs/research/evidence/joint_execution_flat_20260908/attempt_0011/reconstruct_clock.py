#!/usr/bin/env python3
"""Reconstruct first post-adoption discarded state interval; read-only raw input."""
import argparse,csv,hashlib,json,pathlib
p=argparse.ArgumentParser();p.add_argument('csv');p.add_argument('--out',required=True);a=p.parse_args()
f=pathlib.Path(a.csv)
with f.open() as stream: rows=list(csv.DictReader(l for l in stream if not l.startswith('#')))
previous=None;witness=None
for r in rows:
 t=float(r['state_tick_s']);dt=float(r['motion_dt_s']);gap=float(r['state_tick_gap_s'])
 if 21.016<=t<21.254 and gap>0.008 and dt==0:
  witness={'previous':{k:previous[k] for k in keys},'first_discard':{k:r[k] for k in keys}};break
 keys=['state_tick_s','motion_dt_s','state_tick_gap_s','telemetry_running_time_s','phase'];previous=r
assert witness is not None
before=witness['previous'];after=witness['first_discard']
lost=float(after['state_tick_s'])-float(before['state_tick_s'])-(float(after['telemetry_running_time_s'])-float(before['telemetry_running_time_s']))
assert abs(lost-0.010)<1e-9
assert before['phase']==after['phase']
report={'raw_sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'witness':witness,'lost_elapsed_s':lost,'predicted_phase_residual_ns':-round(lost*1e9),'scope':'clock loss only; not transport root cause or stability'}
with pathlib.Path(a.out).open('x') as out:json.dump(report,out,indent=2);out.write('\n')
print(json.dumps(report,indent=2))
