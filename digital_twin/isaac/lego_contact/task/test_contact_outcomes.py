"""Synthetic protocol regressions, not physical LEGO outcome evidence."""
import copy
import unittest

try:
    from .contact_outcomes import LIMITS, classify_trial, evaluate, support_pattern
    from .stack_layout import generate_layout
except ImportError:
    from contact_outcomes import LIMITS, classify_trial, evaluate, support_pattern
    from stack_layout import generate_layout


def layout():
    return generate_layout({"baseplate":{"grid_shape":[32,32],"stud_pitch_m":.008},
                            "stack":{"center_mode":"plate_center","layer_count":2,"layer_pitch_m":.0096}})


def criteria():
    return {"predeclared":True,"provenance":"synthetic_fixture","conditions_id":"synthetic-coupon",
            "measurement_profile_id":"synthetic-external-measurements",
            "xy_error_m":.001,"depth_error_m":.001,"yaw_error_rad":.01,"tilt_rad":.01,
            "fixture_motion_m":.001,"fixture_tilt_rad":.01,"grasp_slip_m":.0005,"grasp_slip_rad":.01,
            "max_force_N":2.,"max_torque_Nm":.1,"minimum_partial_depth_m":.0001,
            "retention_load_N":.5,"retention_duration_s":.5,"retention_displacement_m":.0005,
            "jam_force_N":.5,"jam_window_s":.2,"jam_depth_excursion_m":.0001,
            "jam_xy_excursion_m":.0001,"jam_yaw_excursion_rad":.001,"max_load_sample_gap_s":.15,
            "unload_force_N":.25,"unload_torque_Nm":.01,"max_unload_to_release_s":.15,
            "max_retention_to_final_pose_s":.2,"retention_transverse_force_N":.05,"retention_test_torque_Nm":.02}


def trial(geometry, identity="b_0_0_0"):
    profile=criteria()["measurement_profile_id"]
    owners=next(b["support"]["contributors"] for b in geometry["bricks"] if b["id"]==identity)
    return {"trial_id":"one","recording_id":"synthetic-recording-one","split":"validation","provenance":"synthetic",
            "conditions_id":"synthetic-coupon","brick_id":identity,"support_pattern":support_pattern(geometry,identity),
            "insertion_rate_m_s":.001,"withdrawal_rate_m_s":None,
            "calibration":{"valid":True,"external_brick_pose":True,"wrench_bias_corrected":True,
                           "profile_id":profile,"conditions_id":"synthetic-coupon"},
            "pose":{"valid":True,"independent":True,"source":"external_brick_relative_pose","measurement_profile_id":profile,
                    "observed_at_s":1.5,"xy_error_m":0.,"xy_error_m_uncertainty":0.,"depth_error_m":0.,"depth_error_m_uncertainty":0.,
                    "yaw_error_rad":0.,"yaw_error_rad_uncertainty":0.,"tilt_rad":0.,"tilt_rad_uncertainty":0.,
                    "engagement_depth_m":.002,"engagement_depth_uncertainty_m":0.},
            "fixture":{"valid":True,"independent":True,"support_layout_verified":True,"support_owners":owners,
                       "measurement_profile_id":profile,"motion_m":0.,"motion_uncertainty_m":0.,"tilt_rad":0.,"tilt_uncertainty_rad":0.,
                       "observed_from_s":0.,"observed_until_s":1.5},
            "grasp":{"valid":True,"observable":True,"measurement_profile_id":profile,
                     "slip_m":0.,"slip_uncertainty_m":0.,"slip_rad":0.,"slip_uncertainty_rad":0.,"observed_from_s":0.,"observed_until_s":.3},
            "loads":{"valid":True,"measurement_profile_id":profile,"frame":"M_at_M","depth_source":"external_brick_relative_pose",
                     "force_uncertainty_N":0.,"torque_uncertainty_Nm":0.,"depth_uncertainty_m":0.,"xy_uncertainty_m":0.,"yaw_uncertainty_rad":0.,
                     "samples":[{"valid":True,"t_s":t,"phase":"unload" if t==.2 else "insertion","depth_m":d,"relative_x_m":0.,"relative_y_m":0.,"relative_yaw_rad":0.,
                                 "wrench":[0,0,.2,0,0,0]} for t,d in [(0,0),(.1,.001),(.2,.002)]]},
            "retention":{"valid":True,"independent":True,"unloaded_before_release":True,"gripper_released":True,"fixture_valid":True,
                         "method":"external_relative_pose_under_measured_pull","measurement_profile_id":profile,
                         "release_s":.3,"start_s":.4,"end_s":1.4,"minimum_outward_force_N":.6,"load_uncertainty_N":0.,
                         "maximum_measured_load_N":.6,"maximum_measured_torque_Nm":0.,"torque_uncertainty_Nm":0.,
                         "axis_calibrated":True,"frame":"M_at_M","outward_axis_M":[0,0,1],"maximum_transverse_force_N":0.,
                         "max_relative_motion_m":0.,"motion_uncertainty_m":0.}}


class ContactOutcomeTests(unittest.TestCase):
    def setUp(self):
        self.layout=layout(); self.criteria=criteria(); self.trial=trial(self.layout)

    def classify(self,record=None):
        return classify_trial(self.trial if record is None else record,self.criteria,self.layout)

    def test_success_requires_independent_release_retention_for_all_support_patterns(self):
        for identity in ("b_0_0_0","b_1_1_1","b_0_0_1"):
            record=trial(self.layout,identity)
            result=self.classify(record)
            self.assertEqual(result["outcome"],"success")
            self.assertFalse(result["training_ready"])
            self.assertFalse(result["inventory_advanced"])
            record["support_pattern"]="baseplate_8_studs"
            if identity!="b_0_0_0":self.assertEqual(self.classify(record)["outcome"],"recoverable_stop")

    def test_no_pose_tcp_attachment_force_drop_or_resting_shortcuts(self):
        for mutate in (lambda r:r.pop("pose"),lambda r:r.pop("retention"),
                       lambda r:r["retention"].update(method="simulator_attachment_flag"),
                       lambda r:r["retention"].update(gripper_released=False),
                       lambda r:r["retention"].update(minimum_outward_force_N=0.)):
            record=copy.deepcopy(self.trial)
            record.update(tcp_z_m=-100.,force_drop=True,attached=True,seated=True)
            mutate(record)
            self.assertEqual(self.classify(record)["outcome"],"recoverable_stop")

    def test_unknown_criteria_uncertainty_fixture_and_slip_stop(self):
        self.assertEqual(classify_trial(self.trial,{key:None for key in LIMITS},self.layout)["outcome"],"recoverable_stop")
        self.trial["pose"]["valid"]=False
        self.assertEqual(self.classify()["outcome"],"recoverable_stop")
        self.trial["pose"]["valid"]=True
        self.trial["pose"]["depth_error_m_uncertainty"]=.01
        self.assertEqual(self.classify()["outcome"],"recoverable_stop")
        self.trial["pose"]["depth_error_m_uncertainty"]=0.
        self.trial["fixture"]["motion_m"]=.002
        self.assertEqual(self.classify()["outcome"],"recoverable_stop")
        self.trial["fixture"]["motion_m"]=0.
        self.trial["grasp"]["slip_m"]=.002
        self.assertEqual(self.classify()["outcome"],"slip")

    def test_partial_jam_and_alignment_motion_are_distinct(self):
        self.trial["pose"]["depth_error_m"]=.01
        self.assertEqual(self.classify()["outcome"],"partial_seating")
        samples=self.trial["loads"]["samples"]
        for index,sample in enumerate(samples):
            sample.update(t_s=index*.13,phase="insertion",depth_m=.001,wrench=[0,0,.8,0,0,0])
        self.assertEqual(self.classify()["outcome"],"jam")
        samples[-1]["relative_x_m"]=.01
        self.assertEqual(self.classify()["outcome"],"partial_seating")
        samples[-1]["relative_x_m"]=0.
        samples[-1]["t_s"]=1.
        self.assertEqual(self.classify()["outcome"],"recoverable_stop")

    def test_retention_failure_time_order_and_unsafe_load(self):
        self.trial["retention"]["max_relative_motion_m"]=.01
        self.assertEqual(self.classify()["outcome"],"partial_seating")
        self.trial["retention"]["max_relative_motion_m"]=0.
        self.trial["retention"]["end_s"]=4.
        self.assertEqual(self.classify()["outcome"],"recoverable_stop")
        self.trial["retention"]["end_s"]=1.4
        self.trial["loads"]["samples"][0]["wrench"]=[0,0,3.,0,0,0]
        result=self.classify()
        self.assertEqual(result["outcome"],"recoverable_stop")
        self.assertEqual(result["peak_force_N"],3.)

    def test_retention_safety_and_fixture_coverage_cannot_be_skipped(self):
        self.trial["retention"]["maximum_measured_load_N"]=100.
        result=self.classify()
        self.assertEqual(result["outcome"],"recoverable_stop")
        self.assertEqual(result["peak_force_N"],100.)
        self.trial["retention"]["maximum_measured_load_N"]=.6
        self.trial["retention"]["maximum_measured_torque_Nm"]=1.
        self.assertEqual(self.classify()["outcome"],"recoverable_stop")
        self.trial["retention"]["maximum_measured_torque_Nm"]=0.
        self.trial["fixture"]["observed_until_s"]=.3
        self.assertEqual(self.classify()["outcome"],"recoverable_stop")

    def test_withdrawal_stall_and_uncertain_slip_are_not_jam_or_slip(self):
        self.trial["pose"]["depth_error_m"]=.01
        for index,sample in enumerate(self.trial["loads"]["samples"]):
            sample.update(t_s=index*.13,phase="withdrawal",depth_m=.001,wrench=[0,0,.8,0,0,0])
        self.assertEqual(self.classify()["outcome"],"partial_seating")
        self.trial["grasp"]["slip_uncertainty_m"]=.01
        self.assertEqual(self.classify()["outcome"],"recoverable_stop")

    def test_unloaded_boolean_without_load_trace_is_insufficient(self):
        self.trial["loads"]["samples"][-1]["wrench"]=[0,0,.4,0,0,0]
        self.assertEqual(self.classify()["outcome"],"recoverable_stop")

    def test_signed_tensile_direction_and_observation_coverage_are_required(self):
        for packet,key,value in (("retention","minimum_outward_force_N",-.6),("retention","minimum_outward_force_N",0.),
                                 ("retention","maximum_transverse_force_N",.6),("retention","frame","unknown"),
                                 ("retention","axis_calibrated",False),("grasp","observed_until_s",.2),
                                 ("fixture","observed_until_s",1.4)):
            record=copy.deepcopy(self.trial)
            record[packet][key]=value
            self.assertEqual(self.classify(record)["outcome"],"recoverable_stop")
        record=copy.deepcopy(self.trial)
        record["retention"].update(release_s=100.,start_s=100.1,end_s=101.1)
        record["pose"]["observed_at_s"]=101.2
        record["fixture"]["observed_until_s"]=101.2
        record["grasp"]["observed_until_s"]=100.
        self.assertEqual(self.classify(record)["outcome"],"recoverable_stop")
        self.trial["loads"]["samples"][-1]["wrench"]=[0,0,.2,0,0,0]
        self.trial["loads"]["samples"][-1]["phase"]="contact"
        self.assertEqual(self.classify()["outcome"],"recoverable_stop")

    def test_whole_trials_rates_and_physical_evidence_are_not_synthetic(self):
        second=copy.deepcopy(self.trial)
        second.update(trial_id="two",recording_id="synthetic-two",split="fit",insertion_rate_m_s=.002,withdrawal_rate_m_s=.002)
        self.trial["withdrawal_rate_m_s"]=.001
        manifest={"schema":"lego_contact_outcome_v1","criteria":self.criteria,"trials":[self.trial,second]}
        report=evaluate(manifest,self.layout)
        self.assertEqual(report["counts_by_provenance"]["synthetic"]["success"],2)
        self.assertEqual(report["counts_by_provenance"]["physical"]["success"],0)
        self.assertTrue(report["multiple_rates_present"])
        self.assertFalse(report["physical_multiple_rates_present"])
        self.assertFalse(report["all_support_patterns_have_physical_retained_success"])
        second["recording_id"]=self.trial["recording_id"]
        report=evaluate(manifest,self.layout)
        self.assertFalse(report["recording_split_and_repeat_identity_valid"])
        self.assertEqual(report["results"][1]["outcome"],"recoverable_stop")
        manifest["trials"]=[self.trial]
        self.trial["provenance"]="physical"
        self.criteria["provenance"]="physical_repeatability_and_load_limits"
        self.assertEqual(evaluate(manifest,self.layout)["results"][0]["outcome"],"recoverable_stop")


if __name__ == "__main__":
    unittest.main()
