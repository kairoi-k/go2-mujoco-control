#!/usr/bin/env python3
"""Read-only reproduction of the three retained eeb5d75 development runs."""
import argparse,collections,csv,hashlib,json,math,re,subprocess,sys
from pathlib import Path
SHA='eeb5d757620712759604d8c51b2b9075d05625dc'
RUNS={'flat':'b1_intervals_v2_flat_eeb5d75_20260907_0001',
      'step':'b1_intervals_v2_step_eeb5d75_20260907_0001',
      'debug':'b1_intervals_v2_reject_debug_eeb5d75_20260907_0001'}
def read(p):
    with p.open() as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--repo',type=Path,required=True)
    ap.add_argument('--output-dir',type=Path,required=True);args=ap.parse_args()
    repo=args.repo.resolve();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    result={'schema':'b1-eeb5d75-closeout-results-v1','runtime_sha':SHA,'candidate':False,'runs':{}}
    for key,name in RUNS.items():
        rel=Path('example/cpp/experiments/_runs')/name;run=repo/rel
        manifest=json.loads((run/'run_manifest.json').read_text())
        repository=manifest['repository']
        if repository['git_commit']!=SHA or str(repository['git_dirty']).lower()!='false':
            raise ValueError('Wrong/dirty source '+name)
        item={'run_id':name,'repository':repository,'statuses':manifest['statuses'],
              'raw_sha256':{n:sha(run/n) for n in ['run_manifest.json','data.csv','contact_ground_truth.csv','controller.log','environment.txt']},'analyses':{}}
        calls=[('cycles','audit_running_cycle_truth.py',['--start','17','--end','24'],'running_cycle_0001.json')] if key=='flat' else [
            ('dynamic_v3','analyze_b1_dynamic_v3.py',['--scene','unitree_robots/go2/b1_v3_running_step_5cm.xml'],'dynamic_v3_0001.json'),
            ('first_contact','audit_b1_approach_contact.py',[],'first_contact_0001.json')]
        for label,script,extra,old in calls:
            dest=out/(key+'_'+label+'.json')
            command=[sys.executable,'example/cpp/tools/analysis/'+script,str(rel),*extra,'--out',str(dest)]
            proc=subprocess.run(command,cwd=repo,text=True,capture_output=True)
            (out/(key+'_'+label+'.log')).write_text(proc.stdout+proc.stderr)
            if not dest.exists():raise RuntimeError('Analyzer produced no result '+script+': '+proc.stderr)
            data=json.loads(dest.read_text())
            expected_exit=1 if label=='dynamic_v3' and data.get('status')=='NOT_CERTIFIED' else 0
            if proc.returncode!=expected_exit:raise RuntimeError('Analyzer exit/status mismatch '+script)
            item['analyses'][label]=data
            if (run/old).exists():
                same=data==json.loads((run/old).read_text())
                item.setdefault('original_analysis_reproduction',{})[old]=same
                if not same:raise ValueError('Original result mismatch '+str(run/old))
        c=read(run/'data.csv')
        item['prepare_counters']={k:c[-1][k] for k in ['terrain_target_prepare_attempts','terrain_target_prepared','terrain_target_prepare_rejections']}
        active=[x for x in c if float(x['motion_stage']) in (2,3) and float(x['velocity_command_active'])==1]
        offset=float(active[0]['state_tick_s'])-float(active[0]['telemetry_gait_time_s'])
        worst=max(active,key=lambda x:abs(float(x['state_tick_s'])-float(x['telemetry_gait_time_s'])-offset))
        item['profile_clock']={'initial_offset_s':offset,'max_abs_drift_s':abs(float(worst['state_tick_s'])-float(worst['telemetry_gait_time_s'])-offset),'worst_state_tick_s':float(worst['state_tick_s'])}
        if key!='flat':
            first=item['analyses']['first_contact'];t=first['first_step_contact_s']
            item['pre_first_front_contact']={}
            for leg in ['FL','FR']:
                hit=first['first_riser_contact'].get(leg)
                if hit is None:continue
                stop=hit['time_s'];a=[x for x in c if stop-.2<=float(x['state_tick_s'])<=stop]
                fields=['terrain_plan_failure','terrain_execution_plan_usable','terrain_execution_plan_id','terrain_execution_applied_mask',f'terrain_exec_{leg}_valid',f'terrain_exec_{leg}_in_flight',f'terrain_exec_{leg}_target_required']
                item['pre_first_front_contact'][leg]={'interval_s':[stop-.2,stop],'inclusive_endpoint':True,'rows':len(a),'field_counts':{k:dict(collections.Counter(x[k] for x in a)) for k in fields}}
            if key=='debug':
                entries=[];counts=collections.Counter()
                for line in (run/'controller.log').read_text(errors='replace').splitlines():
                    m=re.search(r'Terrain planner id=(\d+) state=([0-9.]+)',line)
                    if not m or not(t-.8<=float(m[2])<t):continue
                    reasons={}
                    for reason in ['swing_clearance','unknown','reachability']:
                        a=re.search(r'\b'+reason+r'=(\d+)',line)
                        if a:reasons[reason]=int(a[1]);counts[reason]+=int(a[1])
                    z=re.search(r'start_bottom_clearance=([+\-a-zA-Z0-9.]+)',line)
                    value=float(z[1]) if z else None
                    if value is not None and not math.isfinite(value):value=None
                    entries.append({'plan_id':int(m[1]),'state_s':float(m[2]),'reasons':reasons,'start_bottom_clearance_m':value,'raw_line':line})
                item['parsed_debug_rejections']={'interval_s':[t-.8,t],'end_exclusive':True,'parsed_lines':len(entries),'reason_totals':dict(counts),'entries':entries,'coverage_claim':'parsed aggregate lines only; interleaved stdout/stderr may not preserve every event; no projection onto the no-debug run'}
        result['runs'][key]=item
    target=out/'results.json';target.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(target),'sha256':sha(target),'candidate':False}))
if __name__=='__main__':main()
