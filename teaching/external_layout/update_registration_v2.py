"""Explicit cached-image registration authoring; never opens a camera or robot."""
import hashlib,json
from pathlib import Path
import cv2
from perception import load_config,compile_layout,COLORS
R=Path(__file__).resolve().parent
p=R/'config_v2_candidate.json';c=json.loads(p.read_text());image=R/'captures/current_all12.jpg'
c['reference_image']='captures/current_all12.jpg';c['reference_image_sha256']=hashlib.sha256(image.read_bytes()).hexdigest()
new={0:[[613,38],[723,40],[727,86],[612,85]],1:[[613,94],[727,95],[736,149],[612,148]],2:[[613,155],[736,156],[747,218],[612,218]]}
for f in c['footprints']:
 if f['bank']==1 and f['row']<3:
  f['surface_status']='registered_flat_visible_top';f['top_plane_landmarks']={'footprint_st_m':[[0,0],[.0954,0],[.0954,.0632],[0,.0632]],'image_px':new[f['row']],'source':'Actual current_all12.jpg visible brick TOP corners; user reports all12filled; no inferred occupancy','top_height_scope':'Current single flat top, independent of floor plane'}
c['classification']['feature_scales']=[35.,12.,12.,2.5,20.]
c['classification']['feature_definition']='Median Lab,Hwrapped,S of saturation-selected pigment pixels; per-reference prototypes; L downweighted for illumination'
c['temporal']={'window_frames':5,'occupied_frames':3,'empty_frames':4}
c['static_occlusion_polygons_px']=[[[0,0],[501,0],[494,39],[480,81],[451,117],[406,145],[348,153],[0,153]],[[0,0],[395,0],[395,720],[0,720]]]
c['static_occlusion_polylines_px']=[{'points':[[300,222],[411,224],[526,245],[585,256],[650,290],[723,288],[808,275]],'thickness_px':12}]
c.pop('white_gap_polygons_px',None)
regions=[([442,304,589,309],['external_r3_c0','external_r4_c0']),([425,404,586,411],['external_r4_c0','external_r5_c0']),([599,330,607,391],['external_r4_c0','external_r4_c1']),([597,430,606,504],['external_r5_c0','external_r5_c1']),([390,422,403,508],['external_r5_c0']),([800,430,810,503],['external_r5_c1']),([782,330,789,392],['external_r4_c1']),([768,246,774,292],['external_r3_c1'])]
c['white_gap_regions']=[{'polygon_px':[[a,b],[d,b],[d,e],[a,e]],'affected_footprints':ids} for (a,b,d,e),ids in regions]
c['registration_checks']={'search_radius_px':50,'minimum_match_score':.75,'maximum_translation_px':4,'minimum_good_landmarks':2,'landmark_rectangles_px':[[790,506,870,578],[324,499,398,567],[1110,1,1278,69]],'scope':'Fixed image landmarks only; translation invalidates observation without warping'}
c['empty_references']={}
c['setup_ready_confirmation']='User reports all12filled; current_all12 cached frame shows visible flat tops. Occupancy still requires pixel evidence.'
c['color_reference_status']='Manual apparent-color labels from current patches; not independently operator-confirmed or accuracy-calibrated'
p.write_text(json.dumps(c,indent=2)+'\n');cfg=load_config(p);layout=compile_layout(cfg);slots={s['slot_id']:s for s in layout['slots']}
refs={300121:[(5,0,0,1),(4,1,0,0),(3,1,1,0)],6487946:[(5,0,1,2),(4,0,0,3),(5,1,2,2)],6135191:[(5,0,1,0),(5,0,1,1),(4,0,0,2)],4211201:[(5,0,0,3),(5,0,1,3),(5,0,2,0)],6117418:[(5,0,0,0),(4,0,1,2),(4,0,0,0)]}
c['color_references']={str(k):{'label':COLORS[k],'slot_ids':[f'external_r{r}_c{b}_m{m}_n{n}' for r,b,m,n in ids],'polygons_px':[slots[f'external_r{r}_c{b}_m{m}_n{n}']['sample_polygon_px'] for r,b,m,n in ids]} for k,ids in refs.items()}
# Separate material patch observed as white paper; not an empty-state target label.
target=slots['external_r5_c0_m0_n1'];x,y=__import__('numpy').mean(target['sample_polygon_px'],axis=0)
c['empty_references'][target['slot_id']]={'material_reference_verified':True,'polygon_px':[[x-8,y-4],[x+8,y-4],[x+8,y+4],[x-8,y+4]],'reference_polygon_px':[[598,440],[606,440],[606,494],[598,494]],'reference_image':c['reference_image'],'reference_image_sha256':c['reference_image_sha256'],'provenance':'White-paper MATERIAL reference BEFORE any removal test; target remains red in baseline. Floor parallax unvalidated; no removal outcome claimed.'}
p.write_text(json.dumps(c,indent=2)+'\n')
