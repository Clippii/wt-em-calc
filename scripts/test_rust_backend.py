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
import copy
from missile_copy import deepcopy as fast_copy, validate_flight_numbers as fast_validate

if '--em-cython-worker' in sys.argv:
    os.environ['WT_NUMERIC_BACKEND']='python'
    import em_backend
    em_backend.activate()

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

    def test_aerodynamic_forces(self):
        import aero_vectors
        p=kernels.aero_properties(dict(finsAoaHor=.2,finsAoaVer=.3))
        p.update(axis_quaternion=[0.,0.,0.,1.],cy_table=[[0.,1.,1.],[1.,1.,1.2],[4.,0.,.7]])
        f=rust.aero_function(self.library,lambda *a,**k:self.fail('Unexpected aerodynamic fallback'))
        def exact(a,b,path=''):
            if isinstance(a,dict):
                self.assertEqual(a.keys(),b.keys(),path)
                for key in a:exact(a[key],b[key],path+'/'+key)
            elif isinstance(a,list):
                self.assertEqual(len(a),len(b),path)
                for i,(x,y) in enumerate(zip(a,b)):exact(x,y,path+'/'+str(i))
            else:
                self.assertEqual(struct.pack('<d',a),struct.pack('<d',b),path)
        r=random.Random(934)
        for i in range(1000):
            q=[r.uniform(-1,1) for _ in range(4)]
            norm=sum(x*x for x in q)**.5
            q=[x/norm for x in q]
            args=(p,r.uniform(-100,30000),[r.uniform(-1400,1400) for _ in range(3)],q,[r.uniform(-5,5) for _ in range(3)])
            kwargs=dict(fins=[r.uniform(-1,1),r.uniform(-1,1)],
                        perturbation=r.choice([0,.49,.5,1.99,2,3]),body_random=r.uniform(-1,1),
                        gravity=i%2==0,use_cxi=i%3==0,angular_environment=[.01,-.02,.03],
                        mass_lost=r.uniform(0,20),mass_term=.02,additional_cx=.03,
                        additional_lever=.1,wind=[10.,-2.,3.],torque=[.1,.2,-.3],force=[20.,-10.,30.])
            exact(f(*args,**kwargs),aero_vectors.forces(*args,**kwargs))
        for mach in (0.,.61,1.,1.4,4.):
            for delta in (-1e-6,0.,1e-6):
                speed=max(0.,mach+delta)*kernels.atmosphere(0.)['sound_speed']
                args=(p,0.,[speed,0.,0.],[0.,0.,0.,1.],[0.,0.,0.])
                exact(f(*args),aero_vectors.forces(*args))
        p['mass']=0.
        args=(p,0.,[100.,0.,0.],[0.,0.,0.,1.],[0.,0.,0.])
        exact(f(*args),aero_vectors.forces(*args))
        result=f(p,0.,[0.,0.,0.],[0.,0.,0.,1.],[0.,0.,0.])
        self.assertIs(result['baseline'],result['steering'])
        self.assertIs(result['flow'],result['fin_flow'])

    def test_matrix_quaternion(self):
        import control_frame
        accelerated=rust.matrix_quaternion_function(self.library,lambda *a:self.fail('Unexpected quaternion fallback'))
        rng=random.Random(1024)
        rows=[([1.,0.,0.],[0.,1.,0.],[0.,0.,1.]),([0.,0.,0.],)*3]
        rows.extend(tuple([rng.uniform(-1.,1.) for _ in range(3)] for _ in range(3)) for _ in range(1000))
        for args in rows:
            for a,b in zip(accelerated(*args),control_frame.matrix_quaternion(*args)):
                self.exact(a,b)

    def test_vector_transforms(self):
        import motor_vector,shared_seeker
        rng=random.Random(1125)
        for mode,reference in enumerate((motor_vector.rotate_thrust,shared_seeker.world_residual,shared_seeker.coast_body)):
            accelerated=rust.vector_function(self.library,lambda *a:self.fail('Unexpected vector fallback'),mode)
            for i in range(1000):
                q=[rng.uniform(-1.,1.) for _ in range(4)]
                vector=[rng.uniform(-100.,100.) for _ in range(3)]
                args=(q,vector,[rng.uniform(-1.,1.) for _ in range(3)]) if mode==1 else (q,vector)
                for a,b in zip(accelerated(*args),reference(*args)):self.exact(a,b)
            args=([0.,-0.,0.,1.],[0.,-0.,0.])
            if mode==1:args+=([0.,-0.,0.],)
            for a,b in zip(accelerated(*args),reference(*args)):self.exact(a,b)

    def test_state_validation(self):
        import state_binary32
        for state in ({'body':{'position':[1.,2.,3.]}},
                      {'guidance':{'orientation':{'pid':[[float('nan')]*8,[float('inf')]*8]}}},
                      {'guidance':{'propulsion':{'pid':[float('-inf')]*8}}}):
            self.assertIsNone(fast_validate(state))
            self.assertIsNone(state_binary32.validate_flight_numbers(state))
        for state in ({'body':{'position':[1.,float('inf'),3.]}},
                      {'guidance':{'orientation':{'pid':[[0.]*8,[0.]*8,[float('nan')]]}}},
                      {'guidance':{'propulsion':{'pid':[0.]*8+[float('inf')]}}}):
            with self.assertRaises(ValueError) as expected:state_binary32.validate_flight_numbers(state)
            with self.assertRaises(ValueError) as actual:fast_validate(state)
            self.assertEqual(str(actual.exception),str(expected.exception))

    def test_body_integration(self):
        accelerated=rust.integrate_function(self.library,lambda *a:self.fail('Unexpected integration fallback'))
        rng=random.Random(1126)
        def exact(a,b):
            if isinstance(a,dict):
                self.assertEqual(a.keys(),b.keys())
                for key in a:exact(a[key],b[key])
            elif isinstance(a,list):
                self.assertEqual(len(a),len(b))
                for x,y in zip(a,b):exact(x,y)
            else:self.exact(a,b)
        for i in range(1000):
            vector=lambda:[rng.uniform(-100.,100.) for _ in range(3)]
            state=dict(position=vector(),velocity=vector(),omega=vector(),quaternion=[rng.uniform(-1.,1.) for _ in range(4)],time=10.,clocks=[1.,2.,3.,4.],distance=20.,water_distance=-0.,water=i%2==0)
            args=(state,vector(),vector(),rng.uniform(1e-5,.1),10.2,[1.,.5,0.,-.5])
            result=accelerated(*args)
            exact(result,body_integration.integrate(*args))
            self.assertIs(result['state']['quaternion'],result['rotation']['quaternion'])

    def test_controller(self):
        import acceleration_control
        accelerated=rust.controller_function(self.library,lambda *a,**k:self.fail('Unexpected controller fallback'))
        rng=random.Random(1127)
        def exact(a,b,path=''):
            if isinstance(a,dict):
                self.assertEqual(a.keys(),b.keys())
                for key in a:exact(a[key],b[key],path+'/'+key)
            elif isinstance(a,list):
                for x,y in zip(a,b):exact(x,y,path)
            elif a is None:self.assertIsNone(b)
            else:self.assertEqual(struct.pack('<d',a),struct.pack('<d',b),path)
        for i in range(1000):
            vector=lambda:[rng.uniform(-50.,50.) for _ in range(3)]
            p=dict(velocity_frame=False,max_accel=30.,limit_aoa=i%2==0,aoa_max=.3,base_speed_squared=90000. if i%3 else 0.,schedule=[],coefficients=[[.01,.02,.03,1.]])
            aero=dict(mass=100.,cy=2.,cy_limit=.5,side_area=.2,cy_table=[[0.,1.,1.],[1.,1.,.9],[4.,0.,.7]]) if i%4 else None
            q=[rng.uniform(-1.,1.) for _ in range(4)]
            state=[[rng.uniform(-1.,1.) for _ in range(8)] for _ in range(2)]
            args=(p,aero,dict(thrust=100.,mass_lost=3.),state,vector(),vector(),q,vector(),rng.uniform(-10.,25000.),2.,1/48)
            exact(accelerated(*args),acceleration_control.update(*args))

    def test_shared_seeker(self):
        import shared_seeker
        fail=lambda *a,**k:self.fail('Unexpected seeker fallback')
        update,slew=rust.seeker_functions(self.library,fail,fail)
        rng=random.Random(1128)
        for i in range(2000):
            p=dict(angle_max=.8,lock_angle_max=.4,rate_max=2.,alpha=.85,beta=.2,gate_rate=.1 if i%2 else 100.)
            vec=lambda:[rng.uniform(-1.,1.) for _ in range(3)]
            q=[rng.uniform(-1.,1.) for _ in range(4)]
            state=dict(angles=[rng.uniform(-.2,.2) for _ in range(2)],direction=vec(),angular_rate=vec())
            coast=i%3==0;measurement=None if coast else vec();dt=rng.uniform(1e-5,.1)
            args=(p,q,measurement,state,dt);kwargs=dict(coast=coast,authored_rate=i%2==0,lock_limit=i%4==0)
            actual=update(*args,**kwargs);expected=shared_seeker.update(*args,**kwargs)
            self.assertEqual(actual['accepted'],expected['accepted'])
            for key in ('angles','direction','angular_rate'):
                for a,b in zip(actual[key],expected[key]):self.exact(a,b)
            args=(p,vec(),state['angles'],dt,i%2==0,i%3==0)
            for a,b in zip(slew(*args),shared_seeker.slew(*args)):self.exact(a,b)

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
            for mode in ('python', 'fast-python', 'rust'):
                env=dict(os.environ,WT_MISSILE_BACKEND='python' if mode=='python' else 'auto',
                         WT_NUMERIC_BACKEND='rust' if 'rust' in mode else 'python',
                 WT_BENCH_PREVIOUS_RUST='1' if mode=='previous-rust' else '0',
                 WT_BENCH_PREVIOUS_VECTOR='1' if mode in ('python','fast-python','previous-rust','previous-vector-rust') else '0',
                 WT_BENCH_PREVIOUS_CONTROLLER='0' if mode in ('rust','previous-seeker-rust') else '1',
                 WT_BENCH_PREVIOUS_SEEKER='0' if mode=='rust' else '1')
                run=subprocess.run([sys.executable,str(rust.ROOT/'scripts/missile_worker.py')],
                                   input=json.dumps(request),capture_output=True,text=True,env=env,timeout=60)
                self.assertEqual(run.returncode,0,run.stdout+run.stderr)
                output=json.loads(run.stdout.splitlines()[-1])
                self.assertEqual(output['type'],'result',output)
                outputs.append(output)
            self.assertEqual(outputs[0],outputs[1],missile+' Python free wins')
            self.assertEqual(outputs[0],outputs[2],missile+' Rust + free wins')

    def test_snapshot_copy(self):
        child=[1.,-0.,float('inf')]
        source={'a':child,'b':child,'tuple':(child,)}
        source['self']=source
        result=fast_copy(source)
        self.assertIs(result['self'],result)
        self.assertIs(result['a'],result['b'])
        self.assertIs(result['a'],result['tuple'][0])
        self.assertIsNot(result['a'],child)
        result['a'][0]=9.
        self.assertEqual(child[0],1.)
        class Custom:
            def __deepcopy__(self,memo):return 'custom-copy'
        self.assertEqual(fast_copy({'x':Custom()}),copy.deepcopy({'x':Custom()}))
        class ListSubclass(list):pass
        self.assertIsInstance(fast_copy(ListSubclass([child])),ListSubclass)


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
    import aero_vectors
    props=kernels.aero_properties(dict(finsAoaHor=.2,finsAoaVer=.3))
    props.update(axis_quaternion=[0.,0.,0.,1.],cy_table=[])
    aero_args=(props,5000.,[600.,10.,20.],[0.,0.,0.,1.],[.1,.2,.3])
    aero=rust.aero_function(rust.load(__file__),aero_vectors.forces)
    pairs['missile aerodynamic forces']=(lambda:aero_vectors.forces(*aero_args,fins=(.1,.2)),lambda:aero(*aero_args,fins=(.1,.2)))
    import control_frame
    matrix=rust.matrix_quaternion_function(rust.load(__file__),control_frame.matrix_quaternion)
    columns=([.8,.6,0.],[-.6,.8,0.],[0.,0.,1.])
    pairs['controller quaternion search']=(lambda:control_frame.matrix_quaternion(*columns),lambda:matrix(*columns))
    import motor_vector,shared_seeker
    rotate=rust.vector_function(rust.load(__file__),motor_vector.rotate_thrust,0)
    residual=rust.vector_function(rust.load(__file__),shared_seeker.world_residual,1)
    coast=rust.vector_function(rust.load(__file__),shared_seeker.coast_body,2)
    q=[.1,.2,.3,.9];v=[10.,20.,30.];pred=[.1,.2,.3]
    pairs['propulsion rotation']=(lambda:motor_vector.rotate_thrust(q,v),lambda:rotate(q,v))
    pairs['seeker residual']=(lambda:shared_seeker.world_residual(q,v,pred),lambda:residual(q,v,pred))
    pairs['seeker coast transform']=(lambda:shared_seeker.coast_body(q,v),lambda:coast(q,v))
    integrate=rust.integrate_function(rust.load(__file__),body_integration.integrate)
    state=dict(position=[0.,5000.,0.],velocity=[600.,0.,0.],omega=[.1,.2,.3],quaternion=q,time=10.,clocks=[1.,2.,3.,4.],distance=20.,water_distance=0.,water=False)
    args=(state,[10.,-5.,3.],[.1,-.2,.3],1/48,10.02)
    pairs['complete body integration']=(lambda:body_integration.integrate(*args),lambda:integrate(*args))

    orientation=rust.orientation_function(rust.load(__file__),body_integration.orientation)
    for name,(old,new) in pairs.items():
        count=300 if 'sweep' in name else 10000
        a=min(timeit.repeat(old,number=count,repeat=3))/count
        b=min(timeit.repeat(new,number=count,repeat=3))/count
        print(f'{name}: Python {a*1e6:.2f} us; Rust {b*1e6:.2f} us; speedup {a/b:.2f}x')


def flight_worker():
    if os.environ.get('WT_BENCH_PREVIOUS_RUST') == '1':
        rust.aero_function=lambda library,reference:reference
        rust.matrix_quaternion_function=lambda library,reference:reference
    if os.environ.get('WT_BENCH_PREVIOUS_VECTOR') == '1':
        rust.vector_function=lambda library,reference,mode:reference
        rust.integrate_function=lambda library,reference:reference
    if os.environ.get('WT_BENCH_PREVIOUS_VECTOR') == '1':
        import missile_copy
        missile_copy.validate_flight_numbers=missile_copy.reference_validator
    if os.environ.get('WT_BENCH_PREVIOUS_CONTROLLER') == '1':
        rust.controller_function=lambda library,reference:reference
    if os.environ.get('WT_BENCH_PREVIOUS_SEEKER') == '1':
        rust.seeker_functions=lambda library,update,slew:(update,slew)
    if os.environ.get('WT_BENCH_NO_CYTHON') == '1':
        import missile_backend
        missile_backend._activate_cython=lambda:'python'
    import missile_worker
    request=dict(missile='us_aim9l_sidewinder',duration=10,
                 launcher=dict(position=[0,5000,0],velocity=[300,0,0],angles=[0,0,0]),
                 target=dict(position=[4000,5000,0],velocity=[200,0,0],angles=[0,0,0]))
    times=[]
    for _ in range(7):
        started=time.perf_counter();result=missile_worker.simulate(request)
        times.append(time.perf_counter()-started)
    digest=hashlib.sha256(json.dumps(result,sort_keys=True).encode()).hexdigest()
    print(json.dumps(dict(median_s=statistics.median(times),times_s=times,digest=digest,backend=missile_worker.BACKEND)))


def benchmark_flight():
    results=[]
    for mode in ('python','fast-python','previous-rust','previous-vector-rust','previous-controller-rust','previous-seeker-rust','rust'):
        env=dict(os.environ,WT_MISSILE_BACKEND='python' if mode=='python' else 'auto',
                 WT_NUMERIC_BACKEND='rust' if 'rust' in mode else 'python',
                 WT_BENCH_PREVIOUS_RUST='1' if mode=='previous-rust' else '0',
                 WT_BENCH_PREVIOUS_VECTOR='1' if mode in ('python','fast-python','previous-rust','previous-vector-rust') else '0',
                 WT_BENCH_PREVIOUS_CONTROLLER='0' if mode in ('rust','previous-seeker-rust') else '1',
                 WT_BENCH_PREVIOUS_SEEKER='0' if mode=='rust' else '1')
        run=subprocess.run([sys.executable,__file__,'--flight-worker'],env=env,
                           capture_output=True,text=True,check=True,timeout=120)
        result=json.loads(run.stdout);results.append(result)
        print(f"10-second AIM-9L flight, {mode}: median {result['median_s']:.6f} s (7 runs; excludes startup)")
    if any(result['digest']!=results[0]['digest'] for result in results):raise AssertionError('Flight output changed')
    print(f"Whole-flight speedup: {results[0]['median_s']/results[-1]['median_s']:.3f}x; output matches exactly")


def benchmark_cython():
    results={}
    for mode in ('python','rust-only','cython','hybrid'):
        env=dict(os.environ,WT_MISSILE_BACKEND='python' if mode=='python' else 'compiled' if mode in ('cython','hybrid') else 'auto',
                 WT_NUMERIC_BACKEND='rust' if mode in ('rust-only','hybrid') else 'python',
                 WT_BENCH_NO_CYTHON='1' if mode=='rust-only' else '0')
        for key in ('WT_BENCH_PREVIOUS_RUST','WT_BENCH_PREVIOUS_VECTOR','WT_BENCH_PREVIOUS_CONTROLLER','WT_BENCH_PREVIOUS_SEEKER'):env[key]='0'
        run=subprocess.run([sys.executable,__file__,'--flight-worker'],env=env,capture_output=True,text=True,check=True,timeout=120)
        results[mode]=json.loads(run.stdout)
        if mode in ('cython','hybrid') and results[mode]['backend']!='compiled':raise AssertionError('Cython not active')
    if len({r['digest'] for r in results.values()})!=1:raise AssertionError('Cython flight mismatch')
    print('CYTHON_FLIGHT_BENCHMARK '+json.dumps(results))
    run=subprocess.run([sys.executable,__file__,'--em-cython-worker'],capture_output=True,text=True,check=True,timeout=300)
    print(run.stdout.strip())


def em_cython_worker():
    os.environ['WT_NUMERIC_BACKEND']='rust'
    library=rust.load(__file__)
    forces={name:[1e4,-2e4,3e4] for name in (*assembly.NAMES,'parasite')}
    positions={name:[1.,-2.,3.] for name in assembly.NAMES};cog=[0.,0.,0.]
    packed_force=[x for name in (*assembly.NAMES,'parasite') for x in forces[name]]
    packed_moment=[x for name in assembly.NAMES for x in forces[name]]+[x for name in assembly.NAMES for x in positions[name]]+cog
    groups={
        'polar':(lambda:REF_POLAR.calc_c(POLAR,37.,12.,.2,.9),lambda:polar.calc_c(POLAR,37.,12.,.2,.9),lambda:rust.polar(library,POLAR,37.,12.,.2,.9)),
        'force':(lambda:REF_ASSEMBLY.assemble_force(forces),lambda:assembly.assemble_force(forces),lambda:rust.assembly(library,(x for name in (*assembly.NAMES,'parasite') for x in forces[name]))),
        'moment':(lambda:REF_ASSEMBLY.assemble_moment(forces,positions,cog),lambda:assembly.assemble_moment(forces,positions,cog),lambda:rust.assembly(library,[x for name in assembly.NAMES for x in forces[name]]+[x for name in assembly.NAMES for x in positions[name]]+cog,moment=True))}
    assert Path(assembly.__file__).suffix in ('.so','.pyd'),assembly.__file__
    result={}
    for name,functions in groups.items():
        expected=functions[0]()
        for fn in functions[1:]:
            assert [struct.pack('<d',x) for x in fn()]==[struct.pack('<d',x) for x in expected]
        result[name]=dict(zip(('python_us','cython_us','rust_us'),[min(timeit.repeat(fn,number=10000,repeat=3))*100 for fn in functions]))
    print('CYTHON_EM_BENCHMARK '+json.dumps(result))


if __name__ == '__main__':
    if '--em-cython-worker' in sys.argv:
        em_cython_worker();raise SystemExit(0)
    parser=argparse.ArgumentParser()
    parser.add_argument('--benchmark',action='store_true')
    parser.add_argument('--benchmark-cython',action='store_true')
    parser.add_argument('--benchmark-flight',action='store_true')
    parser.add_argument('--flight-worker',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.flight_worker:
        flight_worker();raise SystemExit(0)
    result=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromTestCase(Parity))
    if not result.wasSuccessful():raise SystemExit(1)
    if args.benchmark:benchmark()
    if args.benchmark_cython:benchmark_cython()
    if args.benchmark_flight:benchmark_flight()
