"""Small invented protocol/model cases only; no robot, source dataset or Isaac."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

PATH = Path(__file__).resolve().parents[1]/'experiments/control_alignment_20261026/evaluate.py'
spec = importlib.util.spec_from_file_location('independent_control_audit', PATH)
audit = importlib.util.module_from_spec(spec); spec.loader.exec_module(audit)


def tokens():
    identity = np.eye(4).reshape(-1, order='F').tolist()
    coefficients = np.array([[0., 0., .01, .1]]*7)
    return ([2, 2]+identity*3+[.1]*7+[0.]*18+[-1.]*3+[1.]*3
        +[1., .02, .4, .3, 1., .001, .017453292519943295, .5, 2., 6., .3, .1, .08, 50., 0., .15, 0., 1.]
        +[-3.]*7+[3.]*7+identity*8+[0.]+coefficients.reshape(-1).tolist()+[1.])


class ControlAuditTests(unittest.TestCase):
    def test_parse_spline_nested_rebase(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'joint_input.txt'; path.write_text('\n'.join(map(str, tokens())))
            plan = audit.parse_plan(path)
            comparisons = {'timeout': 0. <= plan['timeout'], 'endpoint': 0. <= plan['position_tolerance'],
                'settle': plan['seconds']+.8 <= 2., 'stable': .5 >= plan['stable']}
            self.assertTrue(all(type(value) is bool for value in comparisons.values()))
            self.assertEqual(json.loads(json.dumps(comparisons)), comparisons)
            np.testing.assert_allclose(audit.spline_q(plan, .5), [.105]*7, atol=1e-15)
            reference = plan['q0']+.0005
            np.testing.assert_array_equal(audit.rebased_command(plan, 0., reference)[2], reference)
            u, raw, command = audit.rebased_command(plan, .5, reference)
            self.assertEqual(u, .5); np.testing.assert_allclose(command, [.10525]*7, atol=1e-15)
            np.testing.assert_allclose(audit.rebased_command(plan, 2., reference)[2], [.11]*7, atol=1e-15)
            for values in ([1, *tokens()[1:]], [*tokens(), 99], [*tokens()[:-1], float('nan')]):
                path.write_text('\n'.join(map(str, values)))
                with self.assertRaises(ValueError): audit.parse_plan(path)

    def test_independent_limiter_first_reference_and_small_step(self):
        parameters = np.array([[2.6, 0., 6., 3., -3.]]*7); zeros = np.zeros(7); command = np.full(7, .1)
        output = audit.sdk_limit(parameters, command, zeros, zeros, zeros)
        np.testing.assert_allclose(output, [4.999999e-6]*7, atol=1e-15, rtol=0)
        np.testing.assert_array_equal(audit.sdk_limit(parameters, command, command, zeros, zeros), command)
        with self.assertRaises(ValueError): audit.sdk_limit(parameters, [float('nan')]*7, zeros, zeros, zeros)

    def test_actual_target_channel_not_measured_alias(self):
        command = np.linspace(.101, .701, 7); accepted = command.astype(np.float32).astype(float)
        state = {'q': (accepted+.0002).tolist(), 'q_d': accepted.tolist(),
            'joint_target_readback': accepted.tolist(), 'raw_requested_q': command.tolist()}
        self.assertTrue(all(audit.target_channels(state, command).values()))
        aliased = dict(state, q_d=state['q'])
        self.assertFalse(audit.target_channels(aliased, command)['public_qd_readback'])
        wrong = dict(state, joint_target_readback=(accepted+.0001).tolist())
        self.assertFalse(audit.target_channels(wrong, command)['accepted_limited_command'])

    def test_column_major_FK_and_independent_jacobian(self):
        origins = []
        for i in range(7):
            t = np.eye(4); t[:3, :3] = Rotation.from_euler('xyz', [.1*i, -.12*i, .04]).as_matrix()
            t[:3, 3] = [.01*i, -.008*i, .05]; origins.append(t)
        ee = np.eye(4); ee[:3, 3] = [.02, .01, .1034]
        plan = {'origins': origins, 'ee': ee, 'length': .3}; q = np.linspace(-.3, .3, 7)
        pose, jacobian = audit.model_fk(plan, q)
        for i in range(7):
            plus = q.copy(); minus = q.copy(); plus[i] += 1e-7; minus[i] -= 1e-7
            derivative = (audit.model_fk(plan, plus)[0][:3, 3]-audit.model_fk(plan, minus)[0][:3, 3])/2e-7
            np.testing.assert_allclose(jacobian[:3, i], derivative, atol=2e-9, rtol=0)
        np.testing.assert_array_equal(audit.matrix(pose.reshape(-1, order='F')), pose)
        with self.assertRaises(ValueError): audit.matrix([0.]*15)


if __name__ == '__main__': unittest.main()
