from component_assembly import f32,add,mul


def reciprocal(value):
    return f32(1./value) if abs(value)>f32(4e-19) else 0.


def intact_ratios(areas):
    return [mul(total,reciprocal(total)) for total in
            (add(add(a[1],a[0]),a[2]) for a in areas)]


def intact_polars(polar,areas):
    left_ratio,right_ratio=intact_ratios(areas)
    left=dict(polar,indCoeff=mul(reciprocal(left_ratio),polar['indCoeff']))

    base=left if left_ratio==right_ratio else polar
    right=dict(base,indCoeff=mul(reciprocal(right_ratio),base['indCoeff']))
    return [left,right]
