import copy,json,subprocess,sys,tempfile
from pathlib import Path
fixture,replay,extract=sys.argv[1:]
for mode in ([],["--history"]):
 raw=subprocess.check_output([fixture]+mode,text=True);expected=json.loads(raw.removeprefix('JointSnapshot '))
 assert expected['terrain']['cells'][-1][0] is False
 assert None in expected['terrain']['cells'][-1]
 assert expected['quaternion_wxyz'][3]!=0 and expected['dq'][0]!=0
 with tempfile.TemporaryDirectory() as tmp:
  d=Path(tmp);sha='a'*40
  (d/'run_manifest.json').write_text(json.dumps({'repository':{'git_commit':sha,'git_dirty':False}}))
  def extract_record(record,out):
   (d/'controller.log').write_text('JointSnapshot '+json.dumps(record)+'\n')
   return subprocess.run([sys.executable,extract,str(d),'--runtime-sha',sha,'--out',str(out)],capture_output=True,text=True)
  out=d/'snapshot.txt';r=extract_record(expected,out);assert r.returncode==0,r.stderr
  actual=json.loads(subprocess.check_output([replay,str(out),'--roundtrip'],text=True));assert actual==expected,(actual,expected)
  for field,value in [('dq',[None]*12),('q',[1]*11),('measured_contact',[True]),('touchdown_reference_feet_base',[[0,0,0]])]:
   bad=copy.deepcopy(expected);bad[field]=value
   r=extract_record(bad,d/('bad_'+field));assert r.returncode!=0,field
  extra=d/'extra';extra.write_text(out.read_text()+'unexpected\n');assert subprocess.run([replay,str(extra),'--roundtrip'],capture_output=True).returncode!=0
  truncated=d/'truncated';truncated.write_text('\n'.join(out.read_text().splitlines()[:-1]));assert subprocess.run([replay,str(truncated),'--roundtrip'],capture_output=True).returncode!=0
 print('snapshot exact numeric/unknown roundtrip and malformed input rejection passed')
