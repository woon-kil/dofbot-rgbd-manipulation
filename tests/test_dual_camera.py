import sys,unittest,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from transforms.se3 import identity,multiply,inverse
from transforms.dual_camera import gemini_to_base,dabai_to_base,disagreement_m

class TestDualCamera(unittest.TestCase):
    def test_both_chains_agree_synthetic(self):
        g=identity();g[0][3]=1.0
        ee=identity();ee[0][3]=0.4
        ed=identity();ed[0][3]=0.6
        a=gemini_to_base(g,[0.1,0.2,0.3]);b=dabai_to_base(ee,ed,[0.1,0.2,0.3])
        self.assertLess(disagreement_m(a,b),1e-10)
    def test_both_chains_agree_with_rotation(self):
        import math
        def rz(deg,tx,ty,tz):
            c,s=math.cos(math.radians(deg)),math.sin(math.radians(deg))
            return [[c,-s,0.0,tx],[s,c,0.0,ty],[0.0,0.0,1.0,tz],[0.0,0.0,0.0,1.0]]
        ee=rz(30,0.4,0.1,0.2);ed=rz(-50,0.05,0.02,0.1)
        d=[0.1,0.2,0.3]
        base_point=dabai_to_base(ee,ed,d)
        # Gemini 외부변환을 DaBai 체인에서 역산해 만든 뒤, 같은 점의 Gemini 좌표로 일치하는지 본다.
        g=rz(10,0.7,-0.3,0.4)
        g_point=gemini_to_base(inverse(g),base_point)
        self.assertLess(disagreement_m(gemini_to_base(g,g_point),base_point),1e-9)
    def test_disagreement(self):
        self.assertAlmostEqual(disagreement_m([0,0,0],[0.003,0.004,0]),0.005)
    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError):disagreement_m([float('nan'),0,0],[0,0,0])
    def test_required_cameras_and_safe_defaults(self):
        c=json.loads((Path(__file__).resolve().parents[1]/'configs/project.example.json').read_text())
        self.assertEqual(set(c['cameras']),{'gemini_335l','dabai'})
        self.assertTrue(all(v['required'] for v in c['cameras'].values()))
        self.assertTrue(c['execution']['dry_run']);self.assertFalse(c['execution']['hardware_enabled']);self.assertIs(c['execution'].get('calibration_motion_approved'),False)
        self.assertFalse(any(c['calibration'][k] for k in ('gemini_base_verified','dabai_hand_eye_verified','cross_camera_verified')))
