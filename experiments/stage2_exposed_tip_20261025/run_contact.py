"""Reuse the signed-force B0 runner for one frozen exposed-tip G/D/R sequence."""
import argparse
import json
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
sys.path[:0]=[str(HERE),str(ROOT),str(ROOT/'experiments/stage2_contact_20261025')]
from experiments.stage2_contact_20261025 import run_contact as baseline
from experiments.stage2_contact_20261025 import geometry as stock
from model import Adapter

def main():
    parser=argparse.ArgumentParser(add_help=False);parser.add_argument('--phase',choices=('G','D','R'),required=True)
    parser.add_argument('--approved-G',dest='approved_G',type=Path);parser.add_argument('--predecessor-D',dest='predecessor_D',type=Path)
    args,remaining=parser.parse_known_args();sys.argv=[sys.argv[0],*remaining]
    if args.phase=='R' and (args.predecessor_D is None or not json.loads(args.predecessor_D.read_text())['proceed_gate_passed']):raise RuntimeError('Reserved requires independent D PASS')
    adapter=Adapter(args.phase,args.approved_G)
    baseline.audit=adapter.audit
    def ownership(point,separation,finger,protocol,outward=None):
        result=stock.own_point(point,separation,finger,protocol,outward)
        head=[stock.distance_to_triangles(point,c['points_finger_m'],c['triangle_indices']) for c in finger['colliders'] if c.get('role')=='head']
        white=[stock.distance_to_triangles(point,c['points_finger_m'],c['triangle_indices']) for c in finger['colliders'] if c.get('role')=='white']
        result.update(distance_head_m=min(head),distance_white_m=min(white),compatible_roles=[role for role,distance in (('body',result['distance_pad_m']),('head',min(head)),('white',min(white))) if distance<=result['tolerance_m']])
        return result
    baseline.own_point=ownership
    baseline.main(adapter=adapter)

if __name__=='__main__':main()
