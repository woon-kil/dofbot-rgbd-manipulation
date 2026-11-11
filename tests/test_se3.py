import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import math
from transforms.se3 import identity,multiply,transform_point,inverse

class TransformContract(unittest.TestCase):
    def test_identity(self):
        a=identity();self.assertEqual(multiply(a,a),a)
    def test_chain(self):
        base_ee=identity();base_ee[0][3]=1
        ee_cam=identity();ee_cam[1][3]=2
        base_cam=multiply(base_ee,ee_cam)
        self.assertEqual(transform_point(base_cam,[0,0,3]),[1,2,3])
    def test_invalid(self):
        with self.assertRaises(ValueError):multiply(identity(),[[float('nan')]*4]*4)
    def test_invalid_row(self):
        a=identity();a[3][0]=1
        with self.assertRaises(ValueError):multiply(a,identity())

def rot_z(deg,tx=0.0,ty=0.0,tz=0.0):
    c,s=math.cos(math.radians(deg)),math.sin(math.radians(deg))
    return [[c,-s,0.0,tx],[s,c,0.0,ty],[0.0,0.0,1.0,tz],[0.0,0.0,0.0,1.0]]

class RigidContract(unittest.TestCase):
    def test_rotation_chain_is_order_sensitive(self):
        a=rot_z(90,1,0,0);b=rot_z(0,0,2,0)
        # a·b: b로 (0,2,0) -> a: Rz90(0,2,0)=(-2,0,0), +(1,0,0) = (-1,0,0)
        self.assertEqual([round(v,9) for v in transform_point(multiply(a,b),[0,0,0])],[-1.0,0.0,0.0])
        # b·a 는 다른 결과여야 한다(비가환)
        self.assertEqual([round(v,9) for v in transform_point(multiply(b,a),[0,0,0])],[1.0,2.0,0.0])
    def test_inverse_roundtrip(self):
        t=rot_z(37,0.3,-0.2,0.5)
        r=multiply(t,inverse(t))
        for i in range(4):
            for j in range(4):
                self.assertAlmostEqual(r[i][j],float(i==j),places=9)
    def test_scaled_rotation_rejected(self):
        a=identity();a[0][0]=2.0
        with self.assertRaises(ValueError):multiply(a,identity())
    def test_reflection_rejected(self):
        a=identity();a[2][2]=-1.0
        with self.assertRaises(ValueError):multiply(a,identity())
    def test_tiny_float_error_accepted(self):
        a=rot_z(30);a[0][0]+=1e-9
        multiply(a,identity())
    def test_inverse_rejects_nonrigid(self):
        a=identity();a[0][0]=2.0
        with self.assertRaises(ValueError):inverse(a)

if __name__=='__main__':unittest.main()
