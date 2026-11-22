"""Synthetic-only raw-owner normalization and failure transaction tests."""
import copy
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
try:
    from .normalize_owner_recording import prepare,normalize_sample,convert,RecordingStop
except ImportError:
    from normalize_owner_recording import prepare,normalize_sample,convert,RecordingStop


def fixture(root):
    conditions={key:"synthetic-"+key for key in ("sdk_version","clock_id","tool_profile_id","payload_profile_id","impedance_profile_id","limiter_profile_id","filter_profile_id","wrench_profile_id","fixture_grasp_id")}
    conditions.update(control_period_s=.001,sample_phase="before_command_application")
    header={"type":"owner_recording_header","schema":"lego_owner_raw_v1","provenance":"synthetic","trial_id":"synthetic-only","recording_id":"synthetic-recording-unique",
            "episode_origin_monotonic_ns":1000000000,"conditions":conditions,"profile_revision":1}
    health={"schema":"lego_owner_health_v1","finalized":True,"integrity_valid":True,"offered":2,"enqueued":2,"written":2,
            "dropped":0,"rejected":0,"discarded":0,"disk_errors":0,"fault_flags":0,"stop_requested":False,"worker_policy":0,"worker_priority":0}
    health.update(trial_id=header["trial_id"],recording_id=header["recording_id"],profile_revision=1,bytes_written=1)
    source=root/"synthetic-calibration-evidence.txt"
    source.write_text("Synthetic fixture, not a physical calibration.\n")
    gain=[[float(i==j) for j in range(6)] for i in range(6)];gain[0][0]=2
    calibration={"schema":"local_wrench_calibration_v1","validated":True,"provenance":"synthetic","conditions":conditions,
                 "reference_frame":"K_at_K","units":["N","N","N","Nm","Nm","Nm"],"bias_raw_K":[1.,0.,0.,0.,0.,0.],"gain_K":gain,
                 "noise_std_K":[.01]*6,"latency_s":.002,"reference_q":[0.]*7,"maximum_joint_delta_rad":[.1]*7,
                 "valid_from_monotonic_ns":900000000,"valid_until_monotonic_ns":2000000000,"maximum_hand_age_s":.01,"maximum_sample_gap_s":.002,
                 "evidence":[{"path":source.name,"sha256":hashlib.sha256(source.read_bytes()).hexdigest()}]}
    identity=[float(i%5==0) for i in range(16)]
    calibration["F_T_EE"]=identity
    calibration["EE_T_K"]=identity
    row={"type":"owner_sample","sample_point":"before_command_application","condition_profile_revision":1,
         "sequence":1,"monotonic_ns":1000000000,"robot_time_ms":1000,"dropped_samples_since_last":0,
         "estimated_wrench_valid":True,"singularity_valid":True,"grasp_valid":True,"hand_valid":True,"robot_fault":False,
         "wrench_calibration_status":"uncalibrated","wrench_valid":False,"wrench_K_raw":[3.,0.,0.,0.,0.,0.],
         "gripper_width_m":.014,"hand_monotonic_ns":999000000,"phase_id":0}
    for key in ("requested_q","sent_q","desired_q","measured_q","measured_dq"):row[key]=[0.]*7
    for key in ("F_T_EE","EE_T_K","O_T_EE"):row[key]=identity
    second={**row,"sequence":2,"monotonic_ns":1001000000,"robot_time_ms":1001}
    return header,health,calibration,[row,second]


class OwnerNormalizationTests(unittest.TestCase):
    def test_bias_then_gain_no_identity_or_clock_defaults(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);header,health,calibration,rows=fixture(root)
            context=prepare(header,health,calibration,root)
            normalized=normalize_sample(rows[1],context,rows[0])
            self.assertEqual(normalized["wrench_K"][0],4.)
            self.assertEqual(normalized["t_s"],.001)
            self.assertEqual(normalized["wrench_filter_latency_s"],.002)
            self.assertEqual(normalized["wrench_nominal_force_t_s"],-.001)
            changed=copy.deepcopy(calibration);changed["conditions"]["payload_profile_id"]="other"
            with self.assertRaises(RecordingStop):prepare(header,health,changed,root)
            changed=copy.deepcopy(calibration);changed["gain_K"]=None
            with self.assertRaises(RecordingStop):prepare(header,health,changed,root)
            changed=copy.deepcopy(calibration);changed["gain_K"][1]=changed["gain_K"][0][:]
            with self.assertRaises(RecordingStop):prepare(header,health,changed,root)

    def test_drops_fault_singular_clock_age_and_envelope_stop(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);header,health,calibration,rows=fixture(root)
            context=prepare(header,health,calibration,root)
            changes=[{"dropped_samples_since_last":1},{"robot_fault":True},{"singularity_valid":False},
                     {"sequence":3},{"monotonic_ns":1000000000},{"robot_time_ms":1000},{"hand_monotonic_ns":900000000},
                     {"measured_q":[1.]*7},{"wrench_valid":True},
                     {"F_T_EE":[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,.01,0.,0.,1.]}]
            for patch in changes:
                with self.subTest(patch=patch),self.assertRaises(RecordingStop):
                    normalize_sample({**rows[1],**patch},context,rows[0])
            health["dropped"]=1
            with self.assertRaises(RecordingStop):prepare(header,health,calibration,root)

    def test_complete_transaction_and_partial_never_promoted(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);header,health,calibration,rows=fixture(root)
            trace=root/"raw.ndjson";hp=root/"health.json";cp=root/"calibration.json"
            def write_trace(data):
                trace.write_text("\n".join(json.dumps(row) for row in [header,*data])+"\n")
                health["bytes_written"]=trace.stat().st_size
                hp.write_text(json.dumps(health))
            write_trace(rows);hp.write_text(json.dumps(health));cp.write_text(json.dumps(calibration))
            report=convert(trace,hp,cp,root/"ok")
            self.assertEqual(report["written"],2)
            self.assertFalse(report["physical_data"])
            self.assertFalse(report["training_ready"])
            self.assertTrue((root/"ok/trace.ndjson").exists())
            rows[1]["grasp_valid"]=False;write_trace(rows)
            report=convert(trace,hp,cp,root/"failed")
            self.assertEqual(report["status"],"recoverable_stop")
            self.assertFalse((root/"failed/trace.ndjson").exists())
            self.assertTrue((root/"failed/trace.ndjson.partial").exists())

    def test_recording_identity_and_declared_origin_are_not_guessed(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);header,health,calibration,rows=fixture(root)
            header["episode_origin_monotonic_ns"]-=5000000
            context=prepare(header,health,calibration,root)
            self.assertEqual(normalize_sample(rows[0],context,None)["t_s"],.005)
            health["recording_id"]="other-recording-with-same-counts"
            with self.assertRaisesRegex(RecordingStop,"identity"):
                prepare(header,health,calibration,root)


if __name__=="__main__":
    unittest.main()
