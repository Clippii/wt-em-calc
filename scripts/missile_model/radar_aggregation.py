"""Independent 143fdacd0 aggregation with ground/clutter contribution disabled.

Consumes ordered per-unit return descriptors; their generation remains separate.
Client 2.59.0.34. Output values correspond to native offsets 110..140.
"""
from kernels import f32,add,sub,mul,div,safe_div


def inverse_lateral(axes,vector):
    a,d,g,b,e,h,c,f,i=map(f32,axes);x,y,z=map(f32,vector)
    ea=mul(e,a);hd=mul(h,d);gb=mul(g,b);bd=mul(b,d);ha=mul(h,a);ge=mul(g,e)
    determinant=sub(add(add(mul(hd,c),mul(ea,i)),mul(gb,f)),add(add(mul(ha,f),mul(bd,i)),mul(ge,c)))
    if determinant==0.:raise ValueError('Singular receiver frame')
    inverse=div(1.,determinant)
    yy=mul(add(add(mul(sub(mul(g,f),mul(d,i)),x),mul(sub(mul(c,d),mul(f,a)),z)),mul(sub(mul(i,a),mul(g,c)),y)),inverse)
    zz=mul(add(add(mul(sub(ea,bd),z),mul(sub(hd,ge),x)),mul(sub(gb,ha),y)),inverse)
    return yy,zz


def aggregate(properties,returns,*,cache=None,initial_values=None):
    """Return descriptor: values[12], target_id, unit, unit_type, accepted, cache.

    Cache is the shared visibility result {valid,clear,weight}. Each supplied
    return records the cache result that the per-unit generator produced.
    Candidate table starts empty, as in the inspected observation caller.
    """
    initial=dict(valid=False,clear=True,weight=1.) if cache is None else dict(cache)
    shared=dict(initial);values=list(map(f32,initial_values)) if initial_values is not None else [0.]*13
    totals=[0.]*12;reference=0.;mask=0;target_id=-1;best=0.;ranked=[];accepted=[]
    for row in returns:
        v=list(map(f32,row['values']));local=row['cache']
        if local['valid'] and not initial['valid']:
            if not shared['valid']:shared=dict(valid=True,clear=True,weight=1.)
            shared['clear']=bool(shared['clear'] and local['clear'])
            shared['weight']=min(f32(shared['weight']),f32(local['weight']))
        reference=add(reference,v[1])
        if not row['accepted']:continue
        accepted.append(v);mask|=1<<(int(row['unit_type'])&31)
        if len(ranked)<16:
            index=next((i for i,r in enumerate(ranked) if v[0]>r['weight']),len(ranked))
            ranked.insert(index,dict(unit=row['unit'],weight=v[0]))
        for i in (0,2,3,4,5,6,7,8,9,10,11):totals[i]=add(totals[i],v[i])
        if v[0]>best:best=v[0];target_id=row['target_id']
    values[9:12]=[totals[0],0.,reference]
    threshold=add(reference,properties['minimum_signal'])
    detected=mul(properties['detection_scale'],totals[0])>threshold
    if not detected:return dict(accepted=False,mask=mask,target_id=target_id,ranked=ranked,values=values,cache=shared)
    inverse=safe_div(1.,totals[0])
    values[12]=mul(totals[2],inverse)
    values[:3]=[mul(totals[i],inverse) for i in (3,4,5)]
    values[3:5]=[mul(totals[i],inverse) for i in (6,7)]
    values[5]=mul(totals[8],inverse)
    values[6]=max(mul(totals[9],inverse),f32(properties['range_width']))
    values[7]=mul(totals[10],inverse)
    values[8]=max(mul(totals[11],inverse),f32(properties['doppler_width']))
    if len(accepted)>=2:
        means=[*values[:3],values[5],values[7]];indices=(3,4,5,8,10)
        n4=len(accepted)//4*4;deviation=[]
        for key,mean in zip(indices,means):
            lanes=[0.,0.,0.,0.]
            for j in range(n4):lanes[j%4]=add(lanes[j%4],abs(sub(accepted[j][key],mul(accepted[j][0],mean))))
            total=add(add(lanes[3],lanes[1]),add(lanes[2],lanes[0]))
            for v in accepted[n4:]:total=add(total,abs(sub(v[key],mul(v[0],mean))))
            deviation.append(total)
        local_y,local_z=inverse_lateral(properties['receiver_axes'],[mul(x,inverse) for x in deviation[:3]])
        values[3]=add(add(abs(local_z),abs(local_z)),values[3])
        values[4]=add(add(abs(local_y),abs(local_y)),values[4])
        values[6]=add(mul(deviation[3],inverse),values[6])
        values[8]=add(mul(deviation[4],inverse),values[8])
    for row in ranked:row['weight']=mul(row['weight'],inverse)
    return dict(accepted=True,mask=mask,target_id=target_id,ranked=ranked,values=values,cache=shared)
