"""Build viewer configuration from the single accepted operational manifest."""
import hashlib,json
from pathlib import Path
p=Path(__file__).resolve().parent
raw=(p.parent/'geometry-manifest.json').read_bytes();d=json.loads(raw)
c=json.loads((p/'baseline-config.json').read_text());r=d['routing']
assert c['dispatch']==d['adopted_dispatch']
assert c['gantry']==d['gantry']
c.update(version='operational-simulation-20261028-v1',manifestSha256=hashlib.sha256(raw).hexdigest(),upstreamTranslation=d['upstream_translation']['delta_x_m'],inspectionCameras=r['inspection_camera_x_m'],cover=[r['shared_cover_x_min_m'],r['shared_cover_x_max_m']],crossfeed=[r['upstream_exchange']['x_min_m'],r['upstream_exchange']['x_max_m']],crossCenterX=r['upstream_exchange']['cross_transfer_x_m'],router=[r['local_fanout']['x_min_m'],r['local_fanout']['x_max_m']],rejectChannels=d['reject_system']['flat_bottom_channel'],usableForward=r['local_fanout']['usable_forward_length_m'],rejectService=d['reject_system'])
c['reject']['branchStart']=d['reject_system']['branch_collectors'][0]['x_min_m']
c['upstreamCart']=d.get('upstream_kiln_cart')
(p/'layout-config.js').write_text('window.PROPOSED_LAYOUT='+json.dumps(c,separators=(',',':'))+';\n')
print(c['manifestSha256'])
