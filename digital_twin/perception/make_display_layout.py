"""Generate isolated visual poses from existing factory anchors and schematic.

No authoritative coordinates are modified. Bank0 aliases the existing six
source footprints. Bank1 has display IDs only. Height follows the twin's existing
estimated table plane, not a new camera/world reconstruction or taught TCP Z.
"""
import hashlib
import json
import math
from pathlib import Path


def build(positions_path, registration_path, factory=None, job=None, moving=None):
    p = json.loads(positions_path.read_text())
    r = json.loads(registration_path.read_text())
    t = r['T_base_board']
    normal = [t[i][2] for i in range(3)]
    origin = [t[i][3] for i in range(3)]
    gap=(factory['clear_tape_span_m']-2*3*.0318)/3 if factory else .015
    def z_at(x,y):return origin[2]-(normal[0]*(x-origin[0])+normal[1]*(y-origin[1]))/normal[2]
    slots = []
    for row in range(6):
        for bank in range(2):
            for m in range(3):
                source = p['points'][f'point_{row}_{m}']['xyz_m']
                for n in range(4):
                    x = source[0] - bank*(3*.0318 + gap)
                    y = source[1] + (n-1.5)*.0158
                    table_z = z_at(x,y)
                    slots.append(dict(slot_id=f'external_r{row}_c{bank}_m{m}_n{n}',
                        footprint_id=f'external_r{row}_c{bank}',
                        motion_pose_id=f'{row}_{m}' if bank == 0 else None,
                        display_base_xyz_m=[x,y,table_z+.0096/2],
                        display_size_m=[.0318,.0158,.0096]))
    conveyor=[]
    if job is not None and moving is not None:
        step=p['factory_source_grid']['conveyor_row_step_in_base_m']
        for row in range(6):
            target=job['conveyor_target_O_T_EE'].copy()
            for axis in range(3):target[12+axis]+=(row-moving['conveyor_reference_layer'])*step[axis]
            for n in range(4):
                x=target[12];y=target[13]+(n-1.5)*.0158
                conveyor.append(dict(slot_id=f'conveyor_r{row}_n{n}',conveyor_row=row,brick_index=n,
                    motion_pose_id=None,display_base_xyz_m=[x,y,z_at(x,y)+.0096/2+job.get('conveyor_surface_height_offset_m',0.)],
                    display_size_m=[.0318,.0158,.0096]))
    result=dict(schema='fr3.external_layout.display_poses.v1',conveyor_slots=conveyor,
        registration=dict(status='Display-only XY from taught grid; Z uses estimated table plane',
            operator_mapping_confirmed=True,
            row_mapping='r=0..5 image top to bottom; bank0 is existing left bank',
            pick_mapping='m=0..2 image left to right; bank1 has no motion IDs',
            brick_mapping='n=0..3 image top to bottom inside a four-brick pick',
            height='Single visible brick bodies on existing estimated table plane; no hidden layers inferred',
            uncertainty_m=r['registration']['approximate_registration_uncertainty_m'],
            dimensions_m=dict(brick=[.0318,.0158,.0096],footprint=[.0954,.0632],
                total_schematic=[.2358,.4692],bank_gap=gap,row_gap=.015,side_margin=gap,end_margin=.0075),
            source_sha256={str(path):hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (positions_path,registration_path)}), slots=slots)
    if factory:
        tapes={p['name']:p['points'] for p in r['geometry_board_m']['tape_polygons_xy']}
        old_width=lambda name:math.dist(tapes[name][0],tapes[name][3] if name=='lower_crossline' else tapes[name][1])
        left=max(s['display_base_xyz_m'][0]+.0318/2 for s in slots)+gap
        right=left-factory['clear_tape_span_m']
        front=max(s['display_base_xyz_m'][1]+.0158/2 for s in slots)
        cross=front+sum(factory['crossline_gap_from_front_bricks_range_m'])/2
        lower=cross+old_width('lower_crossline')
        end=lower+factory['crossline_lower_edge_to_table_end_m']
        def rect(name,x0,x1,y0,y1,rgb):
            return dict(name=name,rgb=rgb,color=int('181818',16) if rgb[0]<.1 else int('e0ded4',16),
                points_base_m=[[x,y,z_at(x,y)+(.0003 if rgb[0]>.1 else .0006)] for x,y in [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]])
        world_table=[[sum(t[i][j]*([*a,0][j]) for j in range(3))+t[i][3] for i in range(3)] for a in r['geometry_board_m']['table_polygon_xy']]
        y0=min(sum(t[1][j]*([*a,0][j]) for j in range(3))+t[1][3] for name in ('inner_left_vertical','right_vertical') for a in tapes[name])
        polys=[rect('TableEndExtension',right-factory['right_tape_width_m'],left+old_width('inner_left_vertical'),.25,end,[.88,.87,.83]),
            rect('LeftTape',left,left+old_width('inner_left_vertical'),y0,end,[.025,.025,.026]),
            rect('RightTape',right-factory['right_tape_width_m'],right,y0,end,[.025,.025,.026]),
            rect('CrossTape',right,left,cross,lower,[.025,.025,.026])]
        result['registration']['factory_scene']=dict(
            replace_registered_names=['inner_left_vertical','right_vertical','lower_crossline'],polygons=polys,
            reference=factory,clear_span_m=left-right,right_tape_width_m=factory['right_tape_width_m'],
            crossline_gap_m=cross-front,crossline_width_m=lower-cross,table_end_gap_m=end-lower,
            registration_status='Visual estimate anchored to taught grid; actual 12–17mm front gap retained instead of forcing7.5mm; no calibration change')
        result['registration']['status']='Display memory · measured242mm clear span /65mm right tape; estimated table registration'
    return result


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    factory=json.loads(Path(__file__).with_name('factory_reference.json').read_text())
    job=json.loads((root.parent/'teaching/factory_vision/config.json').read_text())
    moving=json.loads((root.parent/'teaching/moving_conveyor/config.json').read_text())
    packet = build(root.parent/'teaching/positions.json',root/'registration/scene_registration.json',factory,job,moving)
    path = Path(__file__).with_name('display_layout.json')
    path.write_text(json.dumps(packet,indent=2)+'\n')
    print(path)
