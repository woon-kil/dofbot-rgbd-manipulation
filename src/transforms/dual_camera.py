"""Pure coordinate-chain utilities. No robot commands or sensor drivers."""
from .se3 import multiply,transform_point

def gemini_to_base(t_base_gemini,point_gemini):
    return transform_point(t_base_gemini,point_gemini)

def dabai_to_base(t_base_ee,t_ee_dabai,point_dabai):
    return transform_point(multiply(t_base_ee,t_ee_dabai),point_dabai)

def disagreement_m(point_a,point_b):
    import math
    if len(point_a)!=3 or len(point_b)!=3: raise ValueError('3D points required')
    if not all(math.isfinite(v) for v in [*point_a,*point_b]): raise ValueError('nonfinite point')
    return math.dist(point_a,point_b)
