"""Game perturbation coefficients, not a physical missile disturbance model."""
import math
from kernels import f32,add,mul


def coefficients(amplitude,body_random,immersion=0.):
    a=f32(amplitude);s=f32(body_random)
    if f32(immersion)>0. or a<=0.:
        return dict(force_scale=1.,angle=0.,cosine=1.,sine=0.,lever_fraction=0.)
    scale=mul(.01,a) if a<2. else f32(.02)
    angle=mul(mul(f32(math.pi/4),a) if a<2. else f32(math.pi/2),s)
    return dict(force_scale=add(mul(scale,s),1.),angle=angle,
                cosine=f32(math.cos(angle)),sine=f32(math.sin(angle)),
                lever_fraction=mul(a,1.6) if a<f32(.5) else f32(.8))
