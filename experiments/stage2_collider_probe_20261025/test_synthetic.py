"""Tiny event-sign/redaction/empty-LOST/source checks; no Kit imports."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
from native_observer import NativeObserver,role_packet


def main():
    roles={'/robot/pad':{'side':'left','role':'body'}};actors={'/robot/finger':'left'}
    point={'position':[0.,0.,0.],'normal':[0.,1.,0.],'impulse':[.00001,.0002,0.],'separation':0.}
    header={'actor0':'/robot/finger','actor1':'/private/object','collider0':'/robot/pad','collider1':'/private/shape',
            'proto_index0':0xffffffff,'proto_index1':0xffffffff,'points':[point]}
    packet=role_packet([{'headers':[header]}],roles,actors,.01)
    assert packet['robot_contacts'][0]['normal_force_on_robot_world_N']==[0.,.02,0.]
    assert '/private' not in json.dumps(packet,allow_nan=False)
    reverse={**header,'actor0':header['actor1'],'actor1':header['actor0'],'collider0':header['collider1'],'collider1':header['collider0'],
             'points':[{**point,'normal':[0.,-1.,0.],'impulse':[-.00001,-.0002,0.]}]}
    assert packet==role_packet([{'headers':[reverse]}],roles,actors,.01)
    for bad in ({**header,'proto_index0':0},{**header,'collider0':'/unknown'},
                {**header,'points':[{**point,'normal':[0.,0.,0.]}]}):
        assert role_packet([{'headers':[bad]}],roles,actors,.01)['unknown_robot_shapes']==1
    empty={**header,'collider0':'/unknown','proto_index0':0,'points':[]}
    assert role_packet([{'headers':[empty]}],roles,actors,.01)=={'robot_contacts':[],'unknown_robot_shapes':0}
    # An empty LOST header's original offset is unspecified: never dereference it.
    observer=NativeObserver.__new__(NativeObserver);observer.active=True;observer.callbacks=[];observer.errors=[]
    observer.sim=SimpleNamespace(current_time=.01);observer.decode=str;observer.stage_id=7
    lost=SimpleNamespace(actor0=1,actor1=2,collider0=3,collider1=4,stage_id=7,type='LOST',
                         proto_index0=0xffffffff,proto_index1=0xffffffff,contact_data_offset=-123,num_contact_data=0)
    observer._callback([lost],[])
    assert not observer.errors and observer.callbacks[0]['headers'][0]['points']==[]
    assert observer.callbacks[0]['headers'][0]['raw_collider0']==3
    bad=SimpleNamespace(**{**lost.__dict__,'num_contact_data':1})
    observer._callback([bad],[]);assert observer.errors
    source=ast.parse((Path(__file__).parent/'run_probe.py').read_text())
    tick=next(n for n in ast.walk(source) if isinstance(n,ast.FunctionDef) and n.name=='tick')
    calls=[n.func.attr for n in ast.walk(tick) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
    assert 'set_kinematic_targets' in calls and 'write_root_pose_to_sim' not in calls
    print('PASS: fixed signs/projection/redaction, UNKNOWN cases, empty LOST, raw IDs, target-only measured motion')


if __name__=='__main__':main()
