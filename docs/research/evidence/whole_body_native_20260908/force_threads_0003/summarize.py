"""Read-only comparison of thread-count outputs and independently verified serial oracle."""
import hashlib,json,pathlib,statistics
base=pathlib.Path(__file__).resolve().parent
reference=json.loads((base.parent/'native_force_tracking_20260908_0001/result.json').read_text())['runs'][0]
report={'scope':'fixed source computational comparison, not deadline or horizon viability certification','openmp_wait_policy':'PASSIVE','reference':'../native_force_tracking_20260908_0001/verification.json','results':{}}
for p in sorted(base.glob('threads_*.json'))+sorted(base.glob('fast_*.json')):
 rows=json.loads(p.read_text())['runs'];times=[x['elapsed_ms'] for x in rows]
 report['results'][p.name]={'min_ms':min(times),'median_ms':statistics.median(times),'max_ms':max(times),'evaluations':[x['evaluations'] for x in rows],'all_feasible':all(x['feasible'] for x in rows),'max_control_delta_to_independent_verified_serial':max(abs(a-b) for row in rows for a,b in zip(row['control'],reference['control'])),'max_force_delta_to_independent_verified_serial':max(abs(a-b) for row in rows for a,b in zip(row['forces'],reference['forces']))}
report['hashes']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in base.iterdir() if p.is_file()}
with (base/'summary.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report['results'],indent=2))
