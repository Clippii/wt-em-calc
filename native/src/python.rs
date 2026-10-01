//! Direct CPython builtins over the same audited kernels. Only public Stable ABI
//! functions are used; addresses are supplied by the running interpreter, so the
//! portable cdylib does not link a particular libpython. Callbacks always retain
//! the GIL. Their self tuple owns all interned keys (no global Python objects).
//! Unsupported inputs return None for the existing reference fallback.
use std::ffi::{c_char, c_void};
use std::ptr;
use std::sync::OnceLock;
type O = *mut c_void;
static API: OnceLock<[usize; 19]> = OnceLock::new();
macro_rules! api {
    ($i:expr, $t:ty) => {
        std::mem::transmute::<usize, $t>(API.get().unwrap()[$i])
    };
}
unsafe fn inc(o: O) {
    api!(0, unsafe extern "C" fn(O))(o);
}
unsafe fn dec(o: O) {
    api!(1, unsafe extern "C" fn(O))(o);
}
struct Owned(O);
impl Drop for Owned {
    fn drop(&mut self) {
        unsafe { dec(self.0) }
    }
}
impl Owned {
    fn take(self) -> O {
        let o = self.0;
        std::mem::forget(self);
        o
    }
}
unsafe fn owned(o: O) -> Option<Owned> {
    if o.is_null() {
        None
    } else {
        Some(Owned(o))
    }
}
unsafe fn key(s: O, i: usize) -> O {
    api!(2, unsafe extern "C" fn(O, isize) -> O)(s, i as isize)
}
unsafe fn number(o: O) -> Option<f64> {
    if o.is_null() {
        return None;
    }
    let v = api!(3, unsafe extern "C" fn(O) -> f64)(o);
    // Check conversion errors once at the callback boundary, rather than after
    // every scalar. A pending error always discards the computed result.
    if !v.is_finite() || v.abs() > f32::MAX as f64 {
        None
    } else {
        Some(v)
    }
}
unsafe fn dictionary(s: O, d: O) -> Option<()> {
    // A dict subclass can override __getitem__; preserve its Python behavior.
    let typ = owned(api!(18, unsafe extern "C" fn(O) -> O)(d))?;
    if typ.0 != key(s, 51) {
        return None;
    }
    Some(())
}
unsafe fn field(s: O, d: O, i: usize) -> Option<O> {
    let o = api!(5, unsafe extern "C" fn(O, O) -> O)(d, key(s, i));
    if o.is_null() {
        None
    } else {
        Some(o)
    }
}
unsafe fn scalar(s: O, d: O, i: usize) -> Option<f64> {
    number(field(s, d, i)?)
}
unsafe fn sequence(s: O, o: O, out: &mut [f64]) -> Option<()> {
    if o.is_null() {
        return None;
    }
    let typ = owned(api!(18, unsafe extern "C" fn(O) -> O)(o))?;
    if typ.0 != key(s, 49) && typ.0 != key(s, 50) {
        return None;
    }
    let list = if typ.0 == key(s, 49) {
        api!(6, unsafe extern "C" fn(O) -> isize)(o)
    } else {
        -1
    };
    let get: unsafe extern "C" fn(O, isize) -> O;
    let n;
    if list >= 0 {
        n = list;
        get = api!(7, unsafe extern "C" fn(O, isize) -> O)
    } else {
        api!(8, unsafe extern "C" fn())();
        n = api!(9, unsafe extern "C" fn(O) -> isize)(o);
        get = api!(2, unsafe extern "C" fn(O, isize) -> O)
    }
    if n != out.len() as isize {
        return None;
    }
    for (i, v) in out.iter_mut().enumerate() {
        *v = number(get(o, i as isize))?;
    }
    Some(())
}
unsafe fn float(v: f64) -> Option<Owned> {
    owned(api!(10, unsafe extern "C" fn(f64) -> O)(v))
}
unsafe fn list(v: &[f64]) -> Option<Owned> {
    let result = owned(api!(11, unsafe extern "C" fn(isize) -> O)(v.len() as isize))?;
    for (i, x) in v.iter().enumerate() {
        let value = float(*x)?.take();
        // PyList_SetItem steals the new reference, including on failure.
        if api!(12, unsafe extern "C" fn(O, isize, O) -> i32)(result.0, i as isize, value) < 0 {
            return None;
        }
    }
    Some(result)
}
unsafe fn dict() -> Option<Owned> {
    owned(api!(13, unsafe extern "C" fn() -> O)())
}
unsafe fn put(d: &Owned, name: &'static [u8], v: Owned) -> Option<()> {
    if api!(14, unsafe extern "C" fn(O, *const c_char, O) -> i32)(d.0, name.as_ptr().cast(), v.0)
        < 0
    {
        None
    } else {
        Some(())
    }
}
unsafe fn put_float(d: &Owned, name: &'static [u8], v: f64) -> Option<()> {
    put(d, name, float(v)?)
}
unsafe fn put_list(d: &Owned, name: &'static [u8], v: &[f64]) -> Option<()> {
    put(d, name, list(v)?)
}
unsafe fn finish(s: O, result: Option<Owned>) -> O {
    let error = api!(4, unsafe extern "C" fn() -> O)();
    if error.is_null() {
        if let Some(o) = result {
            return o.take();
        }
    }
    // Input errors are handled by the original Python implementation. Allocation
    // errors must propagate instead of becoming an apparently valid fallback.
    if !error.is_null() && api!(17, unsafe extern "C" fn(O) -> i32)(key(s, 48)) != 0 {
        return ptr::null_mut();
    }
    api!(8, unsafe extern "C" fn())();
    let none = key(s, 47);
    inc(none);
    none
}
unsafe fn args<'a>(values: *const O, n: isize, expected: usize) -> Option<&'a [O]> {
    if n != expected as isize {
        None
    } else {
        Some(std::slice::from_raw_parts(values, expected))
    }
}
unsafe fn polar_input(s: O, p: O) -> Option<[f64; 24]> {
    dictionary(s, p)?;
    let mut out = [0.; 24];
    for (i, v) in out.iter_mut().enumerate() {
        *v = scalar(s, p, i)?;
    }
    Some(out)
}
unsafe extern "C" fn polar(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 6)?;
            let p = polar_input(s, a[0])?;
            let mode = number(a[5])?;
            if ![0., 1., 2.].contains(&mode) {
                return None;
            }
            let mut out = [0.; 2];
            if super::wt_polar(
                p.as_ptr(),
                number(a[1])?,
                number(a[2])?,
                number(a[3])?,
                number(a[4])?,
                mode as u32,
                out.as_mut_ptr(),
            ) == 0
                || !out.iter().all(|v| v.is_finite())
            {
                return None;
            }
            list(&out)
        })(),
    )
}
unsafe extern "C" fn packed_assembly(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 2)?;
            let moment = number(a[1])? != 0.;
            let mut input = [0.; 45];
            sequence(s, a[0], &mut input[..if moment { 45 } else { 24 }])?;
            let mut out = [0.; 3];
            let valid = if moment {
                super::wt_moment(input.as_ptr(), out.as_mut_ptr())
            } else {
                super::wt_force(input.as_ptr(), out.as_mut_ptr())
            };
            if valid == 0 || !out.iter().all(|v| v.is_finite()) {
                None
            } else {
                list(&out)
            }
        })(),
    )
}
unsafe extern "C" fn force(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 1)?;
            dictionary(s, a[0])?;
            let mut input = [0.; 24];
            for i in 0..8 {
                sequence(s, field(s, a[0], 24 + i)?, &mut input[i * 3..i * 3 + 3])?
            }
            let mut out = [0.; 3];
            if super::wt_force(input.as_ptr(), out.as_mut_ptr()) == 0 {
                None
            } else {
                list(&out)
            }
        })(),
    )
}
unsafe extern "C" fn moment(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 3)?;
            dictionary(s, a[0])?;
            dictionary(s, a[1])?;
            let mut input = [0.; 45];
            for i in 0..7 {
                sequence(s, field(s, a[0], 24 + i)?, &mut input[i * 3..i * 3 + 3])?;
                sequence(
                    s,
                    field(s, a[1], 24 + i)?,
                    &mut input[21 + i * 3..24 + i * 3],
                )?
            }
            sequence(s, a[2], &mut input[42..45])?;
            let mut out = [0.; 3];
            if super::wt_moment(input.as_ptr(), out.as_mut_ptr()) == 0 {
                None
            } else {
                list(&out)
            }
        })(),
    )
}
unsafe extern "C" fn batch(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 2)?;
            let p = polar_input(s, a[0])?;
            let count = api!(6, unsafe extern "C" fn(O) -> isize)(a[1]);
            if count < 0 {
                return None;
            }
            let result = owned(api!(11, unsafe extern "C" fn(isize) -> O)(count))?;
            for i in 0..count {
                let row = api!(7, unsafe extern "C" fn(O, isize) -> O)(a[1], i);
                let mut input = [0.; 4];
                sequence(s, row, &mut input)?;
                let mut out = [0.; 2];
                if super::wt_polar(
                    p.as_ptr(),
                    input[0],
                    input[1],
                    input[2],
                    input[3],
                    2,
                    out.as_mut_ptr(),
                ) == 0
                    || !out.iter().all(|v| v.is_finite())
                {
                    return None;
                }
                if api!(12, unsafe extern "C" fn(O, isize, O) -> i32)(
                    result.0,
                    i,
                    list(&out)?.take(),
                ) < 0
                {
                    return None;
                }
            }
            Some(result)
        })(),
    )
}
unsafe extern "C" fn atmosphere(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 1)?;
            let mut out = [0.; 3];
            super::wt_atmosphere(number(a[0])?, out.as_mut_ptr());
            if !out.iter().all(|v| v.is_finite()) {
                return None;
            }
            let d = dict()?;
            put_float(&d, b"density\0", out[0])?;
            put_float(&d, b"sound_speed\0", out[1])?;
            put_float(&d, b"pressure\0", out[2])?;
            Some(d)
        })(),
    )
}
unsafe fn rotation(out: &[f64]) -> Option<Owned> {
    let result = dict()?;
    let trig = owned(api!(11, unsafe extern "C" fn(isize) -> O)(3))?;
    for i in 0..3 {
        let d = dict()?;
        put_float(&d, b"sine\0", out[i * 4])?;
        put_float(&d, b"cosine\0", out[i * 4 + 1])?;
        put(
            &d,
            b"quadrant\0",
            owned(api!(15, unsafe extern "C" fn(i64) -> O)(
                out[i * 4 + 2] as i64,
            ))?,
        )?;
        put_float(&d, b"reduced\0", out[i * 4 + 3])?;
        if api!(12, unsafe extern "C" fn(O, isize, O) -> i32)(trig.0, i as isize, d.take()) < 0 {
            return None;
        }
    }
    put(&result, b"trig\0", trig)?;
    put_list(&result, b"delta\0", &out[12..16])?;
    put_list(&result, b"raw\0", &out[16..20])?;
    put_list(&result, b"quaternion\0", &out[20..24])?;
    Some(result)
}
unsafe extern "C" fn orientation(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 2)?;
            let mut input = [0.; 7];
            sequence(s, a[0], &mut input[..4])?;
            sequence(s, a[1], &mut input[4..])?;
            let mut out = [0.; 24];
            if super::wt_orientation(input.as_ptr(), out.as_mut_ptr()) == 0
                || !out.iter().all(|v| v.is_finite())
            {
                None
            } else {
                rotation(&out)
            }
        })(),
    )
}
unsafe extern "C" fn vector(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 4)?;
            let mode = number(a[3])?;
            if ![0., 1., 2.].contains(&mode) {
                return None;
            }
            let mut input = [0.; 10];
            sequence(s, a[0], &mut input[..4])?;
            sequence(s, a[1], &mut input[4..7])?;
            if mode == 1. {
                sequence(s, a[2], &mut input[7..])?;
            }
            let mut out = [0.; 3];
            if super::aero::wt_vector(input.as_ptr(), mode as u32, out.as_mut_ptr()) == 0 {
                None
            } else {
                list(&out)
            }
        })(),
    )
}
unsafe extern "C" fn matrix(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 3)?;
            let mut input = [0.; 9];
            for i in 0..3 {
                sequence(s, a[i], &mut input[i * 3..i * 3 + 3])?
            }
            let mut out = [0.; 4];
            if super::aero::wt_matrix_quaternion(input.as_ptr(), out.as_mut_ptr()) == 0 {
                None
            } else {
                list(&out)
            }
        })(),
    )
}
unsafe extern "C" fn integrate(s: O, a: *const O, n: isize) -> O {
    finish(
        s,
        (|| {
            let a = args(a, n, 6)?;
            dictionary(s, a[0])?;
            let mut input = [0.; 33];
            for (key, start, len) in [(32, 0, 3), (33, 3, 3), (34, 6, 3), (35, 9, 4), (37, 14, 4)] {
                sequence(s, field(s, a[0], key)?, &mut input[start..start + len])?;
            }
            for (key, i) in [(36, 13), (38, 18), (39, 19), (40, 20)] {
                input[i] = scalar(s, a[0], key)?;
            }
            sequence(s, a[1], &mut input[21..24])?;
            sequence(s, a[2], &mut input[24..27])?;
            input[27] = number(a[3])?;
            input[28] = number(a[4])?;
            sequence(s, a[5], &mut input[29..33])?;
            let mut out = [0.; 51];
            if super::aero::wt_integrate(input.as_ptr(), out.as_mut_ptr()) == 0 {
                return None;
            }
            let result = dict()?;
            let state = dict()?;
            let rot = rotation(&out[27..])?;
            for (name, start, len) in [
                (b"position\0".as_slice(), 0, 3),
                (b"velocity\0", 3, 3),
                (b"omega\0", 6, 3),
                (b"clocks\0", 14, 4),
            ] {
                put_list(&state, name, &out[start..start + len])?;
            }
            let q = api!(5, unsafe extern "C" fn(O, O) -> O)(rot.0, key(s, 35));
            if q.is_null() {
                return None;
            }
            inc(q);
            put(&state, b"quaternion\0", Owned(q))?;
            for (name, i) in [
                (b"time\0".as_slice(), 13),
                (b"distance\0", 18),
                (b"water_distance\0", 19),
                (b"immersion\0", 20),
            ] {
                put_float(&state, name, out[i])?;
            }
            put(&result, b"state\0", state)?;
            put_list(&result, b"displacement\0", &out[21..24])?;
            put_list(&result, b"increment\0", &out[24..27])?;
            put(&result, b"rotation\0", rot)?;
            Some(result)
        })(),
    )
}
#[repr(C)]
struct Method {
    name: *const c_char,
    function: unsafe extern "C" fn(O, *const O, isize) -> O,
    flags: i32,
    doc: *const c_char,
}
// Static definitions have process lifetime; CPython retains pointers to them.
unsafe impl Sync for Method {}
macro_rules! method {
    ($name:literal,$f:ident) => {
        Method {
            name: concat!($name, "\0").as_ptr().cast(),
            function: $f,
            flags: 0x80,
            doc: ptr::null(),
        }
    };
}
static METHODS: [Method; 10] = [
    method!("polar", polar),
    method!("assembly", packed_assembly),
    method!("force", force),
    method!("moment", moment),
    method!("batch", batch),
    method!("atmosphere", atmosphere),
    method!("orientation", orientation),
    method!("vector", vector),
    method!("matrix", matrix),
    method!("integrate", integrate),
];
#[no_mangle]
pub unsafe extern "C" fn wt_python_init(addresses: *const usize, len: usize, context: O) -> O {
    if len != 19 || context.is_null() {
        return ptr::null_mut();
    }
    let table: [usize; 19] = std::slice::from_raw_parts(addresses, 19)
        .try_into()
        .unwrap();
    if table.contains(&0) {
        return ptr::null_mut();
    }
    if let Some(current) = API.get() {
        if current != &table {
            return ptr::null_mut();
        }
    } else {
        let _ = API.set(table);
    }
    if api!(9, unsafe extern "C" fn(O) -> isize)(context) != 53 {
        return ptr::null_mut();
    }
    let result = (|| {
        let d = dict()?;
        for m in &METHODS {
            let f = owned(api!(16, unsafe extern "C" fn(*const Method, O, O) -> O)(
                m,
                context,
                ptr::null_mut(),
            ))?;
            if api!(14, unsafe extern "C" fn(O, *const c_char, O) -> i32)(d.0, m.name, f.0) < 0 {
                return None;
            }
        }
        Some(d)
    })();
    result.map_or(ptr::null_mut(), Owned::take)
}
