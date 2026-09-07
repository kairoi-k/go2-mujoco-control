#!/usr/bin/env python3
"""Verify committed packet, runtime sources and immutable raw evidence hashes."""
import argparse,hashlib,json
from pathlib import Path

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[4])
    ap.add_argument('--skip-raw',action='store_true',help='source/packet only; cannot verify experiment evidence')
    ap.add_argument('--check-binaries',action='store_true',help='compare retained local binaries; rebuilt binaries may differ')
    args=ap.parse_args(); packet=Path(__file__).resolve().parent
    manifest=json.loads((packet/'manifest.json').read_text())
    groups=[('packet_sha256',packet),('source_sha256',args.repo)]
    if not args.skip_raw:groups.append(('raw_sha256',args.repo))
    if args.check_binaries:groups.append(('artifact_sha256',args.repo))
    count=0
    for name,base in groups:
        for relative,expected in manifest[name].items():
            p=base/relative
            if not p.is_file():raise SystemExit('Missing '+str(p))
            actual=hashlib.sha256(p.read_bytes()).hexdigest()
            if actual!=expected:raise SystemExit('Hash mismatch '+str(p))
            count+=1
    print(json.dumps({'verified_files':count,'raw_verified':not args.skip_raw,
                      'binaries_verified':args.check_binaries,'acceptance_claim':False}))
if __name__=='__main__':main()
