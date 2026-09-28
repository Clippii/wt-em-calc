import math
from functools import lru_cache
from component_assembly import f32,add,sub,mul
from piston_model import div


from structural_limits import interval


IDENTITY=[1.,0.,0.,0.,1.,0.,0.,0.,1.]


def transform(b,v):
    return [add(add(mul(v[2],b[i+6]),mul(v[1],b[i+3])),mul(v[0],b[i])) for i in range(3)]


def inverse(b):
    a,d,g,c,e,h,k,f,i=b

    det=sub(add(add(mul(mul(a,e),i),mul(mul(d,h),k)),mul(mul(g,c),f)),
            add(add(mul(mul(g,e),k),mul(mul(d,c),i)),mul(mul(a,h),f)))
    return [div(x,det) for x in [sub(mul(e,i),mul(h,f)),sub(mul(g,f),mul(d,i)),sub(mul(d,h),mul(g,e)),
            sub(mul(h,k),mul(c,i)),sub(mul(a,i),mul(g,k)),sub(mul(g,c),mul(a,h)),
            sub(mul(c,f),mul(e,k)),sub(mul(d,k),mul(a,f)),sub(mul(a,e),mul(d,c))]]


@lru_cache(maxsize=128)
def inverse_basis(b):
    return tuple(inverse(b))


@lru_cache(maxsize=512)
def local_flow(basis,r,v,w):
    local=[add(sub(mul(r[1],w[2]),mul(r[2],w[1])),v[0]),
           add(sub(mul(r[2],w[0]),mul(r[0],w[2])),v[1]),
           add(sub(mul(r[0],w[1]),mul(r[1],w[0])),v[2])]
    inv=inverse_basis(basis)
    local=[add(add(mul(local[0],inv[i]),mul(local[1],inv[i+3])),mul(local[2],inv[i+6])) for i in range(3)]
    local_w=[add(add(mul(w[2],inv[i+6]),mul(w[1],inv[i+3])),mul(w[0],inv[i])) for i in range(3)]
    transverse_sq=add(mul(local[2],local[2]),mul(local[1],local[1]))
    return local,local_w,transverse_sq,f32(math.sqrt(transverse_sq))


