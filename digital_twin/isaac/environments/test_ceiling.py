import unittest

from digital_twin.isaac.environments.ceiling import calculate


class CeilingTests(unittest.TestCase):
    def spec(self,clearance):
        return {'whole_layer_size_m':[.0954,.0632,.0096],
                'ceiling':{'fixed_top_reference_thickness_m':.0032,'nominal_thickness_m':.01745,
                           'fingertip_to_brick_bottom_clearance_m':clearance,'clearance_verified':True,
                           'clearance_evidence':'synthetic unit-test input, not a physical measurement'}}

    def test_top_is_pinned_and_underside_moves_down_to_studs(self):
        # Independent positions from the retained CAD profile, in metres.
        a=calculate(self.spec(0.),.03609036868907001,.06164037057020194,.0017)
        b=calculate(self.spec(.002),.03609036868907001,.06164037057020194,.0017)
        self.assertAlmostEqual(a['top_surface_z_finger_m'],b['top_surface_z_finger_m'])
        self.assertAlmostEqual(b['thickness_m'],.01945)
        self.assertAlmostEqual(b['underside_z_finger_m']-a['underside_z_finger_m'],.002)
        self.assertLess(abs(b['stud_contact_error_m']),1e-7)

    def test_missing_verification_does_not_become_zero_clearance(self):
        for value in (None,-.001,.010):
            with self.assertRaises(ValueError):
                calculate(self.spec(value),.03609036868907001,.06164037057020194,.0017)
        spec=self.spec(.002);spec['ceiling']['clearance_verified']=False
        with self.assertRaises(ValueError):
            calculate(spec,.03609036868907001,.06164037057020194,.0017)

    def test_spacing_only_revision_retains_existing_ceiling(self):
        spec=self.spec(None)
        spec['ceiling'].update(mode='preserve_current',retained_thickness_m=.0032,clearance_verified=False)
        value=calculate(spec,.03609036868907001,.06164037057020194,.0017)
        self.assertAlmostEqual(value['thickness_m'],.0032)
        self.assertAlmostEqual(value['underside_z_finger_m'],.03609036868907001)
        self.assertIsNone(value['stud_contact_error_m'])


if __name__=='__main__':
    unittest.main()
