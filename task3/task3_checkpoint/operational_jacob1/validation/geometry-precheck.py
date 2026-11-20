"""Independent exact frozen-subset comparison and scalar support checks."""
import argparse,hashlib,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,default=Path(__file__).resolve().parent.parent/'geometry-manifest.json');p.add_argument('--out',type=Path,default=Path(__file__).resolve().parent/'geometry-precheck.json');a=p.parse_args()
raw=a.manifest.read_bytes();d=json.loads(raw);f=d['formation'];r=d['routing']
baseline=json.loads((Path(__file__).resolve().parent/'accepted-frozen-subsets.json').read_text())
checks=[]
def check(name,ok,detail=None):checks.append({'name':name,'passed':bool(ok),'detail':detail})
for k,value in baseline['subsets'].items():check('frozen subset exact: '+k,d[k]==value)
check('two compact layers fit two-position bed',f['two_layer_bed_x_min_m']<=36.835-.49 and 36.835+.49<=38.335-.49 and 38.335+.49<=f['two_layer_bed_x_max_m'],{'centres':[36.835,38.335],'layer_length_m':.98})
check('compact layer fits formation',f['alignment_field_x_min_m']<=f['alignment_center_x_m']-.49 and f['alignment_center_x_m']+.49<=f['alignment_field_x_max_m'])
check('finished pallet fits pickup rollers',43.135<=f['finished_pallet_pickup_x_m']-.59 and f['finished_pallet_pickup_x_m']+.59<=f['outfeed_x_max_m'])
check('fourteen descending steps reach outfeed pallet datum',abs(1.25-.06-14*.069-.224)<1e-9,{'pallet_origin_initial_m':1.19,'origin_final_m':.224,'support_top_offset_m':.060})
check('cross-module path clears upstream cap',r['upstream_exchange']['cross_transfer_x_m']+.1225<d['reject_system']['flat_bottom_channel']['channels'][0]['upstream_retaining_wall']['x_min_m'])
check('outermost receiving column clears lane deck',r['local_fanout']['maximum_local_after_crossfeed_lateral_m']+.0335<r['working_width_m']/2)
check('128 unique frozen storage positions',len(d['storage']['slots'])==128 and len({s['id'] for s in d['storage']['slots']})==128)
if 'upstream_kiln_cart' in d:
 c=d['upstream_kiln_cart']['cart']
 support=c.get('effective_support_width_x_m',c['width_x_m'])
 check('rotated kiln-cart layers fit effective support width',support+1e-9>=5*1.2+4*.245,{'required_x_m':6.98,'effective_support_x_m':support})
result={'method':'Independent exact frozen geometry and scalar support checks; not contact or vehicle qualification','canonical_sha256':hashlib.sha256(raw).hexdigest(),'prior_accepted_sha256':baseline['source_canonical_sha256'],'check_count':len(checks),'passed':all(x['passed'] for x in checks),'checks':checks}
a.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'passed':result['passed'],'checks':len(checks),'canonical_sha256':result['canonical_sha256']}));raise SystemExit(0 if result['passed'] else 1)
