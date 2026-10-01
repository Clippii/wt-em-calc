"""Reference parity and reproducible kernel benchmarks; run after building Rust."""
import argparse
import importlib.util
import math
import random
import struct
import timeit
import unittest
import sys
import os
import json
import subprocess
import statistics
import time
import hashlib
import tempfile
from pathlib import Path
from unittest.mock import patch

import component_assembly as assembly
import polar_f32 as polar
import rust_backend as rust


def reference(name):
    spec = importlib.util.spec_from_file_location('_reference_' + name, rust.ROOT/'scripts'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module._rust = None
    return module


REF_ASSEMBLY = reference('component_assembly')
REF_POLAR = reference('polar_f32')
REF_POLAR.calc_cl.__globals__['_rust'] = None
sys.path.insert(0, str(rust.ROOT/'scripts/missile_model'))
import kernels
import body_integration
POLAR = dict(zip(rust.FIELDS, [0.1,0.02,0.06,0.08,1.2,-1.1,16.,-14.,10.,-10.,
                              0.005,0.004,0.,0.,0.,5.,0.004,40.,0.01,-0.7,0.8,120.,130.,1.]))


class Parity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.library = rust.load(__file__)
        if cls.library is None:
            raise RuntimeError('Build Rust kernels before running parity tests')

    def exact(self, actual, expected):
        self.assertEqual(struct.pack('<d', actual), struct.pack('<d', expected))

    def test_polars(self):
        rng = random.Random(932)
        boundaries = [-180.,-140.,-90.,-40.,-19.,-14.,-10.,0.,10.,16.,21.,40.,90.,140.,180.]
        for n in range(16):
            p = dict(POLAR, maxDistAng=40.+n, cyMult=0.7+n/25, kq=70.+n)
            angles = [a+d for a in boundaries for d in (-1e-5,0.,1e-5)]
            angles += [rng.uniform(-180.,180.) for _ in range(200)]
            for a in angles:
                for mode, fn in ((0,REF_POLAR.calc_cl),(1,REF_POLAR.calc_cd)):
                    self.exact(rust.polar(self.library,p,a,mode=mode)[0], fn(p,a))
                rotation=rng.uniform(-90.,90.)
                actual=rust.polar(self.library,p,a,rotation,0.17,0.83)
                expected=REF_POLAR.calc_c(p,a,rotation,0.17,0.83)
                for x,y in zip(actual,expected):self.exact(x,y)

    def test_assembly(self):
        rng=random.Random(48)
        for _ in range(1000):
            forces={name:[rng.uniform(-1e6,1e6) for _ in range(3)] for name in (*assembly.NAMES,'parasite')}
            positions={name:[rng.uniform(-15,15) for _ in range(3)] for name in assembly.NAMES}
            cog=[rng.uniform(-2,2) for _ in range(3)]
            for actual,expected in ((assembly.assemble_force(forces),REF_ASSEMBLY.assemble_force(forces)),
                                    (assembly.assemble_moment(forces,positions,cog),REF_ASSEMBLY.assemble_moment(forces,positions,cog))):
                for x,y in zip(actual,expected):self.exact(x,y)

    def test_batch(self):
        rows=[(a,a/2,0.2,0.9) for a in range(-180,181)]
        self.assertEqual(rust.polar_batch(POLAR,rows),[REF_POLAR.calc_c(POLAR,*row) for row in rows])
        self.assertEqual(rust.polar_batch(POLAR,[]),[])
        with self.assertRaises(ValueError):rust.polar_batch(POLAR,[(1,2)])

    def test_mutated_polar(self):
        p=dict(POLAR)
        polar.calc_c(p,12.,4.)
        p['cl0']=0.8
        self.assertEqual(polar.calc_c(p,12.,4.),REF_POLAR.calc_c(p,12.,4.))

    def test_atmosphere(self):
        native = rust.atmosphere_function(self.library, kernels.atmosphere)
        rng=random.Random(18300)
        heights=[0.,-0.,-1000.,18299.999,18300.,18300.001,30000.,100000.]
        heights.extend(rng.uniform(-1000,100000) for _ in range(2000))
        for h in heights:
            actual=native(h); expected=kernels.atmosphere(h)
            for key in expected:self.exact(actual[key],expected[key])

    def test_orientation(self):
        native=rust.orientation_function(self.library,body_integration.orientation)
        rng=random.Random(349)
        cases=[([0.,0.,0.,1.],[0.,-0.,0.]),([0.,0.,0.,0.],[0.,0.,0.])]
        cases.extend(([rng.uniform(-1,1) for _ in range(4)],
                      [rng.uniform(-100,100) for _ in range(3)]) for _ in range(2000))
        for q,increment in cases:
            actual=native(q,increment);expected=body_integration.orientation(q,increment)
            for key in ('delta','raw','quaternion'):
                for x,y in zip(actual[key],expected[key]):self.exact(x,y)
            for a,b in zip(actual['trig'],expected['trig']):
                self.assertEqual(a['quadrant'],b['quadrant'])
                for key in ('sine','cosine','reduced'):self.exact(a[key],b[key])
        with self.assertRaises(ValueError):native([0,0,0,1],[1e20,0,0])

    def test_exception_fallback(self):
        with self.assertRaises(OverflowError):polar.calc_cl(POLAR,1e100)
        with self.assertRaises(ValueError):polar.calc_c(POLAR,1.,math.inf)
        # A finite capped drag result must not mask an intermediate overflow.
        extreme=dict(POLAR,clLineCoeff=1e30)
        with self.assertRaises(OverflowError):polar.calc_cd(extreme,100.)

    def test_missing_or_stale_build(self):
        saved=(rust._library,rust._attempted)
        try:
            with tempfile.TemporaryDirectory() as directory, patch.object(rust,'DIRECTORY',Path(directory)):
                for manifest in (None,dict(signature='stale'),
                                 dict(signature=rust.signature(),binary='invalid.dll',sha256='wrong')):
                    path=Path(directory)/'manifest.json'
                    if manifest is not None:
                        path.write_text(json.dumps(manifest))
                        (Path(directory)/'invalid.dll').write_bytes(b'invalid')
                    rust._library=None;rust._attempted=False
                    with patch.dict(os.environ,WT_NUMERIC_BACKEND='auto'):
                        self.assertIsNone(rust.load(__file__))
                    with patch.dict(os.environ,WT_NUMERIC_BACKEND='rust'):
                        with self.assertRaises(RuntimeError):rust.load(__file__)
        finally:
            rust._library,rust._attempted=saved

    def test_full_missile_flights(self):
        for missile in ('us_aim9l_sidewinder', 'us_aim7f_sparrow', 'su_r_73'):
            request=dict(missile=missile,duration=3,
                         launcher=dict(position=[0,5000,0],velocity=[300,0,0],angles=[0,0,0]),
                         target=dict(position=[4000,5000,100],velocity=[200,0,0],angles=[0,0,0]))
            outputs=[]
            for mode in ('python', 'rust'):
                env=dict(os.environ,WT_MISSILE_BACKEND='python',WT_NUMERIC_BACKEND=mode)
                run=subprocess.run([sys.executable,str(rust.ROOT/'scripts/missile_worker.py')],
                                   input=json.dumps(request),capture_output=True,text=True,env=env,timeout=60)
                self.assertEqual(run.returncode,0,run.stdout+run.stderr)
                output=json.loads(run.stdout.splitlines()[-1])
                self.assertEqual(output['type'],'result',output)
                outputs.append(output)
            self.assertEqual(outputs[0],outputs[1],missile)


def benchmark():
    forces={n:[1e4,-2e4,3e4] for n in (*assembly.NAMES,'parasite')}
    positions={n:[1.,-2.,3.] for n in assembly.NAMES}
    rows=[(a,a/2,0.2,0.9) for a in range(-180,181)]
    pairs={
        'polar force':(lambda:REF_POLAR.calc_c(POLAR,37.,12.,0.2,0.9),lambda:polar.calc_c(POLAR,37.,12.,0.2,0.9)),
        'force assembly':(lambda:REF_ASSEMBLY.assemble_force(forces),lambda:assembly.assemble_force(forces)),
        'moment assembly':(lambda:REF_ASSEMBLY.assemble_moment(forces,positions,[0.,0.,0.]),lambda:assembly.assemble_moment(forces,positions,[0.,0.,0.])),
        'missile atmosphere':(lambda:kernels.atmosphere(5000.),lambda:atmosphere(5000.)),
        'missile orientation':(lambda:body_integration.orientation([0.,0.,0.,1.],[0.01,-0.02,0.03]),lambda:orientation([0.,0.,0.,1.],[0.01,-0.02,0.03])),
        '361-angle sweep':(lambda:[REF_POLAR.calc_c(POLAR,*row) for row in rows],lambda:rust.polar_batch(POLAR,rows))}
    atmosphere=rust.atmosphere_function(rust.load(__file__),kernels.atmosphere)
    orientation=rust.orientation_function(rust.load(__file__),body_integration.orientation)
    for name,(old,new) in pairs.items():
        count=300 if 'sweep' in name else 10000
        a=min(timeit.repeat(old,number=count,repeat=3))/count
        b=min(timeit.repeat(new,number=count,repeat=3))/count
        print(f'{name}: Python {a*1e6:.2f} us; Rust {b*1e6:.2f} us; speedup {a/b:.2f}x')


def flight_worker():
    import missile_worker
    request=dict(missile='us_aim9l_sidewinder',duration=10,
                 launcher=dict(position=[0,5000,0],velocity=[300,0,0],angles=[0,0,0]),
                 target=dict(position=[4000,5000,0],velocity=[200,0,0],angles=[0,0,0]))
    times=[]
    for _ in range(7):
        started=time.perf_counter();result=missile_worker.simulate(request)
        times.append(time.perf_counter()-started)
    digest=hashlib.sha256(json.dumps(result,sort_keys=True).encode()).hexdigest()
    print(json.dumps(dict(median_s=statistics.median(times),times_s=times,digest=digest)))


def benchmark_flight():
    results=[]
    for mode in ('python','rust'):
        env=dict(os.environ,WT_MISSILE_BACKEND='python',WT_NUMERIC_BACKEND=mode)
        run=subprocess.run([sys.executable,__file__,'--flight-worker'],env=env,
                           capture_output=True,text=True,check=True,timeout=120)
        result=json.loads(run.stdout);results.append(result)
        print(f"10-second AIM-9L flight, {mode}: median {result['median_s']:.6f} s (7 runs; excludes startup)")
    if results[0]['digest']!=results[1]['digest']:raise AssertionError('Flight output changed')
    print(f"Whole-flight speedup: {results[0]['median_s']/results[1]['median_s']:.3f}x; output matches exactly")


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--benchmark',action='store_true')
    parser.add_argument('--benchmark-flight',action='store_true')
    parser.add_argument('--flight-worker',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.flight_worker:
        flight_worker();raise SystemExit(0)
    result=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromTestCase(Parity))
    if not result.wasSuccessful():raise SystemExit(1)
    if args.benchmark:benchmark()
    if args.benchmark_flight:benchmark_flight()
