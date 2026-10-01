//! Aerodynamic polar kernels. Every helper rounds at the same boundary as Python.
//! Double intermediates are intentional: replacing them with f32 arithmetic can
//! introduce double-rounding differences from the reference implementation.
use std::cell::Cell;
mod aero;
thread_local! {static OVERFLOW: Cell<bool> = Cell::new(false);}
#[inline]
fn f(x: f64) -> f64 {
    if x.is_finite() && x.abs() > f32::MAX as f64 {
        OVERFLOW.with(|flag| flag.set(true));
    }
    (x as f32) as f64
}
fn reset_overflow() {
    OVERFLOW.with(|flag| flag.set(false));
}
fn success() -> u32 {
    OVERFLOW.with(|flag| u32::from(!flag.get()))
}
#[inline]
fn add(a: f64, b: f64) -> f64 {
    f(a + b)
}
#[inline]
fn sub(a: f64, b: f64) -> f64 {
    f(a - b)
}
#[inline]
fn mul(a: f64, b: f64) -> f64 {
    f(a * b)
}
#[inline]
fn div(a: f64, b: f64) -> f64 {
    f(a / b)
}
#[inline]
fn sin(x: f64) -> f64 {
    f(x.sin())
}

// ABI order matches polar_model.FIELDS; values retain their original f64 values.
fn cl(p: &[f64], angle: f64) -> f64 {
    let a = f(angle);
    if p[9] <= a && a <= p[8] {
        return add(mul(mul(a, p[3]), p[23]), p[0]);
    }
    let positive = add(a, f(0.01)) >= p[8];
    let s = if positive { 1. } else { -1. };
    let crit = p[if positive { 6 } else { 7 }];
    let cy = p[if positive { 4 } else { 5 }];
    let after = p[if positive { 20 } else { 19 }];
    let da = sub(a, crit);
    if mul(da, s) <= 0. {
        let x = sub(crit, a);
        return sub(
            cy,
            mul(mul(mul(x, x), s), p[if positive { 10 } else { 11 }]),
        );
    }
    let maxang = 40_f64.max(p[17]);
    let sa = mul(s, a);
    if sa <= maxang {
        if sa <= p[17] {
            let pa = p[15];
            if mul(da, s) < pa {
                return sub(cy, mul(mul(mul(da, da), s), p[16]));
            }
            let h = mul(sin(mul(f(0.039269909262657166), p[17])), after);
            let x = sub(mul(sub(p[17], pa), s), crit);
            let den = mul(x, x);
            let coeff = if den > f(4e-19) {
                div(sub(sub(cy, h), mul(mul(mul(pa, pa), s), p[16])), den)
            } else {
                0.
            };
            let x = sub(mul(p[17], s), a);
            return add(mul(mul(x, x), coeff), h);
        }
        return mul(after, sin(mul(mul(a, f(0.039269909262657166)), s)));
    }
    if sa <= 140. {
        let local_sign = if sa > 90. { -s } else { s };
        let local_angle = if sa > 90. { sub(180., sa) } else { sa };
        let one = sin(mul(f(0.039269909262657166), maxang));
        let two = sin(sub(
            f(-1.5707963705062866),
            mul(0_f64.max(add(maxang, -40.)), f(0.03141592815518379)),
        ));
        let correction = add(two, one);
        let correction = mul(
            add(
                mul(add(mul(sa, f(-0.01)), f(0.3999999761581421)), correction),
                correction,
            ),
            s,
        );
        let wave = mul(
            sin(add(
                mul(local_angle, f(0.03141592815518379)),
                f(0.3141592741012573),
            )),
            local_sign,
        );
        return mul(add(wave, correction), mul(after, s));
    }
    mul(
        mul(mul(s, s), after),
        sin(add(
            mul(sa, f(0.039269909262657166)),
            f(-7.0685834884643555),
        )),
    )
}

fn cd(p: &[f64], angle: f64) -> f64 {
    let a = f(angle);
    let line = add(mul(p[3], a), p[0]);
    let delta = sub(a, p[if a >= 0. { 6 } else { 7 }]);
    let delta = if a < 0. { -delta } else { delta };
    let cd = add(
        add(mul(mul(line, line), p[2]), p[1]),
        if delta >= 0. { mul(delta, p[18]) } else { 0. },
    );
    let bound = add(
        mul(sin(mul(a, f(0.01745329238474369))).abs(), p[4]),
        f(0.15),
    );
    cd.min(bound)
}

fn coefficients(p: &[f64], a: f64, angle: f64, cl_add: f64, cd_coeff: f64) -> [f64; 2] {
    let cd = mul(cd(p, a), f(cd_coeff));
    let cl = add(cl(p, a), f(cl_add));
    let radians = mul(f(angle), f(0.01745329238474369));
    let sn = sin(radians);
    let cs = f(radians.cos());
    [
        mul(sub(mul(cs, cd), mul(cl, sn)), p[21]),
        mul(add(mul(cs, cl), mul(cd, sn)), p[22]),
    ]
}

/// Caller supplies 24 readable doubles and output space for two doubles.
/// mode 0=lift, 1=drag, 2=rotated force coefficients. No pointer is retained.
#[no_mangle]
pub unsafe extern "C" fn wt_polar(
    p: *const f64,
    a: f64,
    angle: f64,
    cl_add: f64,
    cd_coeff: f64,
    mode: u32,
    out: *mut f64,
) -> u32 {
    reset_overflow();
    let p = std::slice::from_raw_parts(p, 24);
    let result = match mode {
        0 => [cl(p, a), 0.],
        1 => [cd(p, a), 0.],
        _ => coefficients(p, a, angle, cl_add, cd_coeff),
    };
    std::ptr::copy_nonoverlapping(result.as_ptr(), out, 2);
    success()
}

/// One FFI crossing for an entire angle sweep. Each row is [a, rotation,
/// added lift, drag multiplier]; outputs are consecutive coefficient pairs.
#[no_mangle]
pub unsafe extern "C" fn wt_polar_batch(
    p: *const f64,
    inputs: *const f64,
    count: usize,
    out: *mut f64,
) -> u32 {
    reset_overflow();
    let p = std::slice::from_raw_parts(p, 24);
    for i in 0..count {
        let row = std::slice::from_raw_parts(inputs.add(i * 4), 4);
        let result = coefficients(p, row[0], row[1], row[2], row[3]);
        std::ptr::copy_nonoverlapping(result.as_ptr(), out.add(i * 2), 2);
    }
    success()
}

/// Eight force vectors ordered as component_assembly.NAMES, then parasite.
#[no_mangle]
pub unsafe extern "C" fn wt_force(input: *const f64, out: *mut f64) -> u32 {
    reset_overflow();
    let v = std::slice::from_raw_parts(input, 24);
    let mut q = [0.; 24];
    for i in 0..24 {
        q[i] = f(v[i]);
    }
    let [l, r, h, j, v, b, c, p] = std::array::from_fn::<_, 8, _>(|i| &q[i * 3..i * 3 + 3]);
    let result = [
        add(
            add(
                add(v[0], add(j[0], add(h[0], add(add(l[0], c[0]), r[0])))),
                b[0],
            ),
            p[0],
        ),
        add(
            add(
                add(add(add(add(l[1], c[1]), r[1]), j[1]), add(v[1], h[1])),
                p[1],
            ),
            b[1],
        ),
        add(add(add(b[2], add(p[2], add(r[2], l[2]))), c[2]), v[2]),
    ];
    std::ptr::copy_nonoverlapping(result.as_ptr(), out, 3);
    success()
}

/// Seven forces, seven positions and a center of gravity (45 doubles).
#[no_mangle]
pub unsafe extern "C" fn wt_moment(input: *const f64, out: *mut f64) -> u32 {
    reset_overflow();
    let data = std::slice::from_raw_parts(input, 45);
    let mut forces = [0.; 21];
    let mut arms = [0.; 21];
    for i in 0..21 {
        forces[i] = f(data[i]);
        arms[i] = sub(f(data[21 + i]), f(data[42 + i % 3]));
    }
    let term = |n: usize, fi: usize, ri: usize| mul(forces[n * 3 + fi], arms[n * 3 + ri]);
    let paired = |fi, ri| {
        let wings = add(term(1, fi, ri), term(0, fi, ri));
        let tails = add(term(2, fi, ri), term(3, fi, ri));
        let main = add(wings, tails);
        let extra = add(term(5, fi, ri), term(4, fi, ri));
        add(add(main, extra), term(6, fi, ri))
    };
    let pos_w = add(term(1, 1, 2), term(0, 1, 2));
    let pos_h = add(term(2, 1, 2), term(3, 1, 2));
    let pos = add(
        add(term(6, 1, 2), add(term(5, 1, 2), term(4, 1, 2))),
        add(pos_h, pos_w),
    );
    let neg_w = add(term(1, 2, 1), term(0, 2, 1));
    let neg_h = add(term(2, 2, 1), term(3, 2, 1));
    let neg = add(
        term(6, 2, 1),
        add(add(term(5, 2, 1), term(4, 2, 1)), add(neg_h, neg_w)),
    );
    let result = [
        sub(pos, neg),
        sub(paired(2, 0), paired(0, 2)),
        sub(paired(0, 1), paired(1, 0)),
    ];
    std::ptr::copy_nonoverlapping(result.as_ptr(), out, 3);
    success()
}

#[no_mangle]
pub extern "C" fn wt_numeric_abi() -> u32 {
    2
}

#[no_mangle]
pub unsafe extern "C" fn wt_atmosphere(height: f64, out: *mut f64) {
    // Missile helpers round both operands before arithmetic, unlike EM helpers.
    let add = |a, b| f(f(a) + f(b));
    let mul = |a, b| f(f(a) * f(b));
    let div = |a, b| f(f(a) / f(b));
    let h = f(height);
    let z = h.min(f(18300.));
    let poly = |coefficients: &[f64]| {
        let mut value = f(coefficients[0]);
        for c in &coefficients[1..] {
            value = add(mul(value, z), *c);
        }
        value
    };
    let density = div(
        mul(
            mul(1.225, 18300.),
            poly(&[2.28719e-19, -5.83556e-14, 3.53118e-9, -9.59387e-5, 1.]),
        ),
        h.max(18300.),
    );
    let sound = mul(
        f(mul(
            poly(&[3.97306e-18, -5.71104e-14, 2.18069e-10, -2.27712e-5, 1.]),
            288.16,
        )
        .sqrt()),
        20.1,
    );
    let pressure = div(
        mul(
            mul(101300., 18300.),
            poly(&[1.60373e-18, -1.3738e-13, 5.6763e-9, -1.18441e-4, 1.]),
        ),
        h.max(18300.),
    );
    let result = [density, sound, pressure];
    std::ptr::copy_nonoverlapping(result.as_ptr(), out, 3);
}

#[inline]
fn ma(a: f64, b: f64) -> f64 {
    f(f(a) + f(b))
}
#[inline]
fn ms(a: f64, b: f64) -> f64 {
    f(f(a) - f(b))
}
#[inline]
fn mm(a: f64, b: f64) -> f64 {
    f(f(a) * f(b))
}
#[inline]
fn md(a: f64, b: f64) -> f64 {
    f(f(a) / f(b))
}

fn half_angle(increment: f64) -> Option<[f64; 4]> {
    let h = mm(-f(increment), 0.5);
    let quadrant_input = ms(0.5_f64.copysign(h), mm(increment, 0.31830987334251404));
    let quadrant = quadrant_input.trunc();
    if !quadrant.is_finite() || !(-2147483648. ..2147483648.).contains(&quadrant) {
        return None;
    }
    let k = quadrant as i64;
    let quadrant = k as f64; // Python math.trunc produces integer zero, never -0.
    let t = ma(mm(f(quadrant), -1.5707963705062866), h);
    let t2 = mm(t, t);
    let mut c = ma(
        mm(
            ma(
                mm(ma(mm(-0.0013602249091491103, t2), 0.04165669530630112), t2),
                -0.4999990165233612,
            ),
            t2,
        ),
        1.,
    );
    let mut s = ma(
        mm(
            ma(
                mm(ma(mm(-0.0001950727018993348, t2), 0.00833207555115223), t2),
                -0.16666652262210846,
            ),
            mm(t2, t),
        ),
        t,
    );
    if k & 1 != 0 {
        std::mem::swap(&mut s, &mut c);
    }
    if k & 2 != 0 {
        s = -s;
    }
    if (k + 1) & 2 != 0 {
        c = -c;
    }
    Some([s, c, quadrant, t])
}

/// Quaternion and Euler increment -> trig records, delta, raw and normalized
/// quaternion. Returns 0 outside the reference's audited conversion domain.
#[no_mangle]
pub unsafe extern "C" fn wt_orientation(input: *const f64, out: *mut f64) -> u32 {
    let v = std::slice::from_raw_parts(input, 7);
    let mut trig = [[0.; 4]; 3];
    for i in 0..3 {
        match half_angle(v[4 + i]) {
            Some(t) => trig[i] = t,
            None => return 0,
        }
    }
    let [sx, sy, sz] = std::array::from_fn::<_, 3, _>(|i| trig[i][0]);
    let [cx, cy, cz] = std::array::from_fn::<_, 3, _>(|i| trig[i][1]);
    let a = [
        mm(mm(sy, cx), sz),
        mm(mm(sy, cx), cz),
        mm(mm(cy, cx), sz),
        mm(mm(cy, cx), cz),
    ];
    let b = [
        mm(mm(cy, sx), cz),
        mm(mm(cy, sx), sz),
        mm(mm(sy, sx), cz),
        mm(mm(sy, sx), sz),
    ];
    let [dx, dy, dz, dw] = [
        ma(b[0], a[0]),
        ma(b[1], a[1]),
        ms(a[2], b[2]),
        ms(a[3], b[3]),
    ];
    let [x, y, z, w] = std::array::from_fn::<_, 4, _>(|i| f(v[i]));
    let raw = [
        ms(ma(mm(dz, y), ma(mm(dx, w), mm(dw, x))), mm(dy, z)),
        ms(ma(mm(dx, z), ma(mm(dw, y), mm(dy, w))), mm(dz, x)),
        ms(ma(mm(dy, x), ma(mm(dw, z), mm(dz, w))), mm(dx, y)),
        ms(mm(dw, w), ma(mm(dx, x), ma(mm(dz, z), mm(dy, y)))),
    ];
    let norm2 = ma(
        ma(mm(raw[3], raw[3]), mm(raw[2], raw[2])),
        ma(mm(raw[1], raw[1]), mm(raw[0], raw[0])),
    );
    let q = if norm2 != 0. {
        raw.map(|v| mm(v, md(1., f(norm2.sqrt()))))
    } else {
        [0.; 4]
    };
    for (i, t) in trig.iter().enumerate() {
        std::ptr::copy_nonoverlapping(t.as_ptr(), out.add(i * 4), 4);
    }
    for (i, row) in [[dx, dy, dz, dw], raw, q].iter().enumerate() {
        std::ptr::copy_nonoverlapping(row.as_ptr(), out.add(12 + i * 4), 4);
    }
    1
}
