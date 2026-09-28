from component_assembly import f32,add,sub,mul
from instructor_keyboard import fixed_source as portable_source


def fixed_source(model,state):
    result=portable_source(model,state)
    if model.get('aircraft_trim_mode')=='quasi_steady':return result
    p=dict(result['polar'])
    distance=sub(p['aoaCritH'],p['aoaLineH']);denominator=mul(distance,distance)
    slope=mul(p['clLineCoeff'],p['cyMult'])
    numerator=sub(p['cyCritH'],add(mul(p['aoaLineH'],slope),p['cl0']))
    p['parabCyCoeffH']=f32(numerator/denominator) if denominator>f32(4e-19) else 0.
    return dict(result,polar=p)
