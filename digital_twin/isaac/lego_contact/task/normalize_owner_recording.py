"""Normalize a CLOSED foreground-owner recording with an explicit wrench profile.

No SDK/camera IO. Raw data remain unchanged. A missing sample, failed health,
unsafe state, frame/profile mismatch or uncalibrated wrench stops the whole
conversion; no rows are silently removed. Output is not training authorization.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path


CONDITION_IDS=("sdk_version","clock_id","tool_profile_id","payload_profile_id","impedance_profile_id",
               "limiter_profile_id","filter_profile_id","wrench_profile_id","fixture_grasp_id")


class RecordingStop(ValueError):
    pass


def read_json(text):
    def reject(value):
        raise RecordingStop("nonfinite JSON token: "+value)
    return json.loads(text,parse_constant=reject)


def digest(path):
    h=hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda:stream.read(65536),b""):
            h.update(block)
    return h.hexdigest()


def finite(value,name):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise RecordingStop("finite numeric "+name+" required")
    return float(value)


def vector(value,size,name):
    if not isinstance(value,(list,tuple)) or len(value)!=size:
        raise RecordingStop("invalid "+name)
    return [finite(v,name) for v in value]


def integer(value,name):
    if isinstance(value,bool) or not isinstance(value,int) or value<0:
        raise RecordingStop("nonnegative integer "+name+" required")
    return value


def rigid(value,name):
    array=vector(value,16,name)
    if [array[i] for i in (3,7,11,15)]!=[0.,0.,0.,1.]:
        raise RecordingStop("invalid homogeneous "+name)
    r=[[array[4*j+i] for j in range(3)] for i in range(3)]
    if any(abs(sum(r[i][k]*r[j][k] for k in range(3))-(i==j))>1e-6 for i in range(3) for j in range(3)):
        raise RecordingStop("non-orthonormal "+name)
    determinant=sum(r[0][i]*(r[1][(i+1)%3]*r[2][(i+2)%3]-r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
    if determinant<.999999:
        raise RecordingStop("reflected "+name)
    return array


def prepare(header,health,profile,profile_root):
    if header.get("schema")!="lego_owner_raw_v1" or header.get("type")!="owner_recording_header":
        raise RecordingStop("foreground owner recording header required")
    if header.get("provenance") not in ("physical","synthetic"):
        raise RecordingStop("explicit physical/synthetic provenance required")
    if any(not isinstance(header.get(key),str) or not header[key] or health.get(key)!=header[key] for key in ("trial_id","recording_id")):
        raise RecordingStop("health/trace recording identity mismatch")
    if health.get("profile_revision")!=header.get("profile_revision"):
        raise RecordingStop("health/trace profile revision mismatch")
    origin=integer(header.get("episode_origin_monotonic_ns"),"declared episode origin")
    conditions=header.get("conditions",{})
    for key in CONDITION_IDS:
        if not isinstance(conditions.get(key),str) or not conditions[key] or conditions[key].startswith("UNMEASURED"):
            raise RecordingStop("unidentified condition: "+key)
    if conditions.get("sample_phase")!="before_command_application":
        raise RecordingStop("sample phase mismatch")
    if finite(conditions.get("control_period_s"),"control_period_s")<=0:
        raise RecordingStop("positive control period required")
    if health.get("schema")!="lego_owner_health_v1" or health.get("finalized") is not True or health.get("integrity_valid") is not True:
        raise RecordingStop("recording is not finalized and intact")
    if any(integer(health.get(key),key)!=0 for key in ("dropped","rejected","discarded","disk_errors","fault_flags")):
        raise RecordingStop("recording dropped/rejected samples or encountered a disk error")
    if health.get("stop_requested") is not False or health.get("worker_policy")!=0 or health.get("worker_priority")!=0:
        raise RecordingStop("worker failure or nonordinary recorder scheduling")
    counts=[integer(health.get(key),key) for key in ("offered","enqueued","written")]
    if not counts[0] or len(set(counts))!=1:
        raise RecordingStop("recording counts do not conserve samples")
    if profile.get("schema")!="local_wrench_calibration_v1" or profile.get("validated") is not True:
        raise RecordingStop("validated local wrench calibration required")
    if profile.get("provenance")!=header["provenance"]:
        raise RecordingStop("calibration/data physical provenance mismatch")
    if profile.get("conditions")!=conditions:
        raise RecordingStop("calibration conditions/clock/tool/payload/filter mismatch")
    if profile.get("reference_frame")!="K_at_K" or profile.get("units")!=["N","N","N","Nm","Nm","Nm"]:
        raise RecordingStop("wrench frame/units must be explicit K at K")
    bias=vector(profile.get("bias_raw_K"),6,"raw K bias")
    gain=profile.get("gain_K")
    if not isinstance(gain,list) or len(gain)!=6:
        raise RecordingStop("explicit calibrated wrench gain required; no identity default")
    gain=[vector(row,6,"wrench gain row") for row in gain]
    rank_check=[row[:] for row in gain]
    scale=max(abs(x) for row in gain for x in row)
    for column in range(6):
        pivot=max(range(column,6),key=lambda i:abs(rank_check[i][column]))
        if abs(rank_check[pivot][column])<=max(1.,scale)*1e-12:
            raise RecordingStop("wrench correction loses an observed channel")
        rank_check[column],rank_check[pivot]=rank_check[pivot],rank_check[column]
        for row in range(column+1,6):
            ratio=rank_check[row][column]/rank_check[column][column]
            for j in range(column,6):rank_check[row][j]-=ratio*rank_check[column][j]
    noise=vector(profile.get("noise_std_K"),6,"wrench noise")
    latency=finite(profile.get("latency_s"),"wrench latency")
    if min(noise)<0 or latency<0:
        raise RecordingStop("invalid measured noise/latency")
    q0=vector(profile.get("reference_q"),7,"calibration reference q")
    envelope=vector(profile.get("maximum_joint_delta_rad"),7,"calibrated pose envelope")
    if min(envelope)<=0:
        raise RecordingStop("positive validated pose envelope required")
    start=integer(profile.get("valid_from_monotonic_ns"),"calibration start")
    end=integer(profile.get("valid_until_monotonic_ns"),"calibration end")
    if end<=start:
        raise RecordingStop("calibration validity interval unresolved")
    if finite(profile.get("maximum_hand_age_s"),"hand age")<=0:
        raise RecordingStop("positive hand age bound required")
    if finite(profile.get("maximum_sample_gap_s"),"sample gap")<=0:
        raise RecordingStop("positive measured recording gap bound required")
    evidence=profile.get("evidence",[])
    if not evidence:
        raise RecordingStop("calibration requires source evidence, not a boolean alone")
    for item in evidence:
        path=profile_root/item["path"]
        if digest(path)!=item["sha256"]:
            raise RecordingStop("calibration source evidence hash mismatch")
    return {"conditions":conditions,"bias":bias,"gain":gain,"q0":q0,"envelope":envelope,
            "F_T_EE":rigid(profile.get("F_T_EE"),"calibrated F_T_EE"),
            "EE_T_K":rigid(profile.get("EE_T_K"),"calibrated EE_T_K"),
            "start":start,"end":end,"expected_count":counts[0],"provenance":header["provenance"],
            "profile_revision":integer(header.get("profile_revision"),"profile revision"),"origin_ns":origin,
            "wrench_filter_latency_s":latency,
            "maximum_hand_age_ns":int(profile["maximum_hand_age_s"]*1e9),
            "maximum_sample_gap_ns":int(profile["maximum_sample_gap_s"]*1e9)}


def normalize_sample(row,context,previous):
    if row.get("type")!="owner_sample" or row.get("sample_point")!="before_command_application":
        raise RecordingStop("wrong sample type/phase")
    if row.get("condition_profile_revision")!=context["profile_revision"]:
        raise RecordingStop("conditions changed during trial")
    stamp=integer(row.get("monotonic_ns"),"sample time")
    sequence=integer(row.get("sequence"),"sample sequence")
    robot_ms=integer(row.get("robot_time_ms"),"robot time ms")
    origin_ns=context["origin_ns"]
    if stamp<origin_ns:
        raise RecordingStop("sample predates declared episode origin")
    if not context["start"]<=stamp<=context["end"]:
        raise RecordingStop("wrench calibration expired or predates validity")
    if previous and (sequence!=previous["sequence"]+1 or stamp<=previous["monotonic_ns"] or robot_ms<=previous["robot_time_ms"]):
        raise RecordingStop("recording clock/sequence discontinuity")
    if previous and stamp-previous["monotonic_ns"]>context["maximum_sample_gap_ns"]:
        raise RecordingStop("recording sample interval exceeds declared bound")
    if integer(row.get("dropped_samples_since_last"),"sample drops"):
        raise RecordingStop("sample gap; no silent filtering")
    for key in ("estimated_wrench_valid","singularity_valid","grasp_valid","hand_valid"):
        if row.get(key) is not True:
            raise RecordingStop("invalid/uncertain "+key)
    if row.get("robot_fault") is not False:
        raise RecordingStop("robot fault state")
    if row.get("wrench_calibration_status")!="uncalibrated" or row.get("wrench_valid") is not False:
        raise RecordingStop("raw stream must not masquerade as calibrated wrench")
    result={"t_s":(stamp-origin_ns)*1e-9,"sequence":sequence,"dropped_samples_since_last":0,
            "wrench_valid":True,"singularity_valid":True,"grasp_valid":True,"robot_fault":False}
    for key,size in (("requested_q",7),("sent_q",7),("desired_q",7),("measured_q",7),("measured_dq",7),("F_T_EE",16),("EE_T_K",16),("O_T_EE",16)):
        result[key]=vector(row.get(key),size,key)
    for key in ("F_T_EE","EE_T_K","O_T_EE"):
        rigid(result[key],key)
    if any(result[key]!=context[key] for key in ("F_T_EE","EE_T_K")):
        raise RecordingStop("actual tool/stiffness frame differs from wrench calibration")
    if any(abs(q-ref)>bound for q,ref,bound in zip(result["measured_q"],context["q0"],context["envelope"])):
        raise RecordingStop("state outside measured wrench calibration envelope")
    raw=vector(row.get("wrench_K_raw"),6,"raw wrench")
    corrected=[x-b for x,b in zip(raw,context["bias"])]
    result["wrench_K"]=[sum(g*x for g,x in zip(gain,corrected)) for gain in context["gain"]]
    if not all(math.isfinite(x) for x in result["wrench_K"]):
        raise RecordingStop("nonfinite corrected wrench")
    hand_ns=integer(row.get("hand_monotonic_ns"),"hand time")
    if not 0<=stamp-hand_ns<=context["maximum_hand_age_ns"]:
        raise RecordingStop("stale/future hand state")
    result.update(gripper_width_m=finite(row.get("gripper_width_m"),"hand width"),
                  hand_t_s=(hand_ns-origin_ns)*1e-9,robot_time_ms=robot_ms,phase_id=row.get("phase_id"),
                  monotonic_ns=stamp,wrench_calibration_id=context["conditions"]["wrench_profile_id"],
                  wrench_state_t_s=result["t_s"],wrench_filter_latency_s=context["wrench_filter_latency_s"],
                  wrench_nominal_force_t_s=result["t_s"]-context["wrench_filter_latency_s"])
    if result["gripper_width_m"]<0:
        raise RecordingStop("negative gripper width")
    return result


def convert(trace,health_path,calibration,output):
    output.mkdir(parents=True,exist_ok=False)
    report={"status":"recoverable_stop","training_ready":False,"hardware_deployment_enabled":False,"written":0}
    partial=output/"trace.ndjson.partial"
    try:
        header_line=None
        with trace.open() as source,partial.open("x") as destination:
            header_line=next(source)
            header=read_json(header_line)
            profile=read_json(calibration.read_text())
            health=read_json(health_path.read_text())
            context=prepare(header,health,profile,calibration.resolve().parent)
            if integer(health.get("bytes_written"),"closed trace size")!=trace.stat().st_size:
                raise RecordingStop("health/trace byte count mismatch or truncated recording")
            previous=None
            for line in source:
                if not line.strip():
                    raise RecordingStop("blank/truncated sample row")
                row=read_json(line)
                normalized=normalize_sample(row,context,previous)
                destination.write(json.dumps(normalized,allow_nan=False)+"\n")
                previous=row
                report["written"]+=1
            if report["written"]!=context["expected_count"]:
                raise RecordingStop("closed health/trace sample count mismatch")
        os.replace(partial,output/"trace.ndjson")
        report.update(status="normalized_bias_gain_corrected_owner_recording",provenance=header["provenance"],
                      physical_data=header["provenance"]=="physical",conditions=context["conditions"],
                      source_sha256={str(p):digest(p) for p in (trace,health_path,calibration)},
                      note="Integrity and supplied local calibration only; not independent calibration, controller, contact or policy validation. Rows stay on SDK state time; wrench nominal force time represents declared filter delay, not a resampled signal. Do not apply this correction twice to the live actor raw-wrench path.")
    except (ValueError,KeyError,TypeError,OSError,StopIteration) as error:
        report["reason"]=str(error)
    (output/"normalization_report.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("trace",type=Path);p.add_argument("--health",required=True,type=Path)
    p.add_argument("--calibration",required=True,type=Path);p.add_argument("--output",required=True,type=Path)
    args=p.parse_args()
    report=convert(args.trace,args.health,args.calibration,args.output)
    print(json.dumps(report))
    return 0 if report["status"]=="normalized_bias_gain_corrected_owner_recording" else 2


if __name__=="__main__":
    raise SystemExit(main())
