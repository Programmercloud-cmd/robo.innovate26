import copy
import json
from pathlib import Path
import unittest

import numpy as np

from digital_twin.isaac.environments.cad_fingers import load_cad,ceiling_extension,ray_distance


class SuppliedCadTests(unittest.TestCase):
    def spec(self):
        return json.loads(Path(__file__).with_name('profiles.json').read_text())['fingers']['wide']

    def test_export_scale_mount_and_cavity(self):
        points,dim=load_cad(self.spec())
        self.assertEqual(points.shape,(474*3,3))
        self.assertAlmostEqual(dim['inside_span_m'],.0958,places=7)
        self.assertAlmostEqual(dim['inside_reach_m'],.020,places=7)
        self.assertAlmostEqual(dim['top_thickness_m'],.00655,places=7)
        self.assertAlmostEqual(dim['inside_height_m'],.019,places=7)
        self.assertAlmostEqual(dim['wall_thickness_m'],.005,places=7)
        self.assertAlmostEqual(float(points[:,0].min()),-11.,places=5)
        self.assertAlmostEqual(float(points[:,2].max()),27.6000061,places=5)

    def test_wrong_scale_or_asset_cannot_replace_a_working_variant(self):
        for scale in (1.,.01):
            spec=self.spec();spec['cad_to_original_scale']=scale
            with self.assertRaises(ValueError):load_cad(spec)
        spec=self.spec();spec['cad_sha256']='wrong'
        with self.assertRaises(ValueError):load_cad(spec)

    def test_six_mm_cavity_thickens_downward_without_moving_roof_or_mount(self):
        spec=self.spec();points,original=load_cad(spec);before=points.copy()
        spec['ceiling']=dict(mode='target_cavity_height',target_cavity_height_m=.006)
        (vertices,faces),modified=ceiling_extension(spec,points,original)
        np.testing.assert_array_equal(points,before)
        self.assertEqual(modified['roof_top_cad_z_mm'],original['roof_top_cad_z_mm'])
        self.assertAlmostEqual(modified['top_thickness_m'],.01955,places=7)
        self.assertAlmostEqual(modified['ceiling_adjustment']['added_downward_thickness_m'],.013,places=7)
        union=np.concatenate([points.reshape(-1,3,3),vertices[faces]])
        center=np.array([5.,-15.5,points[:,2].min()+3.])
        roof=center[2]+ray_distance(union,center,[0,0,1])
        self.assertAlmostEqual(roof-points[:,2].min(),6.,places=5)
        self.assertLess(float(vertices[:,2].max()),float(points[:,2].max()))
        for invalid in (-.006,.060,.020,float('nan')):
            spec['ceiling']['target_cavity_height_m']=invalid
            with self.assertRaises(ValueError):ceiling_extension(spec,points,original)


if __name__=='__main__':unittest.main()
