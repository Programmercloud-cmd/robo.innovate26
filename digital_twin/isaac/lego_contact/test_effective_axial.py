"""Small meaningful checks of the effective law, independent of Isaac."""
import copy
import json
import unittest
from pathlib import Path
import numpy as np
from effective_axial import EffectiveAxialModel, ModelDomainError


class EffectiveAxialTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads(Path(__file__).with_name("synthetic_fixture.json").read_text())
        self.model=EffectiveAxialModel(self.config)

    def test_no_contact_finite_and_loading_withdrawal(self):
        self.assertEqual(self.model.evaluate(-.002,1.),self.model.evaluate(0.,-1.))
        self.assertEqual(self.model.evaluate(-.002,1.)["force_N"],0.)
        loading=self.model.evaluate(.002,.02)
        withdrawal=self.model.evaluate(.002,-.02)
        self.assertGreater(loading["force_N"],withdrawal["force_N"])
        self.assertLess(withdrawal["force_N"],0.)
        for value in loading.values():self.assertTrue(np.isfinite(value))
        with self.assertRaises(ModelDomainError):self.model.evaluate(np.nan,0.)
        with self.assertRaises(ModelDomainError):self.model.evaluate(.005,0.)

    def test_rejects_unidentified_nonpassive_or_negative_damping(self):
        unknown=json.loads(Path(__file__).with_name("effective_fit_schema.json").read_text())
        with self.assertRaises(ValueError):EffectiveAxialModel(unknown)
        for key,values in [("damping_N_s_m",[0,-1,1,1,1]),("elastic_force_N",[0,-1,-1,-1,1])]:
            cfg=copy.deepcopy(self.config);cfg[key]=values
            with self.assertRaises(ValueError):EffectiveAxialModel(cfg)

    def test_offsets_are_rejected_not_captured(self):
        self.model.check_admission(0.,0.,0.)
        for state in [(.001,0.,0.),(0.,.01,0.),(0.,0.,.01)]:
            with self.assertRaises(ModelDomainError):self.model.check_admission(*state)

    def test_continuity_and_deterministic_no_hidden_state(self):
        for depth in self.config["depth_knots_m"][:-1]:
            before=self.model.evaluate(depth-1e-10,.01)
            after=self.model.evaluate(depth+1e-10,.01)
            self.assertLess(abs(after["force_N"]-before["force_N"]),1e-6)
        expected=self.model.evaluate(.002,-.01)
        self.model.evaluate(.0035,.03)
        self.assertEqual(expected,self.model.evaluate(.002,-.01))

    def test_closed_loop_no_energy_generation(self):
        for count in (101,201,401):
            depth=np.linspace(0.,.0035,count)
            force_in=np.array([self.model.evaluate(d,.01)["force_N"] for d in depth])
            force_out=np.array([self.model.evaluate(d,-.01)["force_N"] for d in depth])
            work=float(np.trapz(force_in-force_out,depth))
            expected=float(2*.01*np.trapz(np.interp(depth,self.model.depth,self.model.damping),depth))
            self.assertGreater(work,0.)
            self.assertAlmostEqual(work,expected,places=12)

    def test_equal_opposite_force_moment_and_power(self):
        lower=np.array([.01,.02,.03]);upper=lower+np.array([1e-5,-2e-5,.011])
        axis=np.array([0.,0.,1.]);force=axis*.7;reaction=-force
        lower_v=np.array([.01,-.02,.003]);upper_v=np.array([0.,.01,-.02]);lower_w=np.array([.001,-.002,.001])
        moment=np.cross(upper-lower,reaction)
        depth_rate=-np.dot(axis,upper_v-lower_v)-np.dot(np.cross(lower_w,axis),upper-lower)
        power=np.dot(force,upper_v)+np.dot(reaction,lower_v)+np.dot(moment,lower_w)
        np.testing.assert_allclose(force+reaction,0.,atol=1e-15)
        np.testing.assert_allclose(np.cross(upper,force)+np.cross(lower,reaction)+moment,0.,atol=1e-15)
        self.assertAlmostEqual(power,-.7*depth_rate,places=15)

    def test_dissipative_dynamics_dt_refinement(self):
        finals=[]
        for dt in (.0002,.0001):
            depth=.00325;speed=0.;mass=.02
            energy0=self.model.evaluate(depth,speed)["stored_energy_J"]
            maximum=energy0
            for _ in range(round(.05/dt)):
                response=self.model.evaluate(depth,speed)
                speed-=response["force_N"]*dt/mass
                depth+=speed*dt
                energy=self.model.evaluate(depth,speed)["stored_energy_J"]+.5*mass*speed**2
                maximum=max(maximum,energy)
            self.assertLessEqual(maximum,energy0+1e-12)
            self.assertLess(energy,energy0)
            finals.append(depth)
        self.assertLess(abs(finals[0]-finals[1]),1e-6)


if __name__=="__main__":unittest.main()
