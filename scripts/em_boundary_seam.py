def check_boundary(task):
    from em_sampling import worker_solver, sample_boundary_column
    from em_accuracy import speed_tolerance, turn_tolerance
    from em_branch_limit import fixed_alpha, ROLL_LEVELING_PROBE_OFFSET
    from em_pitch_response import PHYSICAL_REASONS
    name,config_json,_,pair=task
    solver=worker_solver(name,config_json)
    if not all(c.get('boundary_status')=='verified limit' and c.get('boundary')
               and c['boundary']['valid'] and c['boundary'].get('envelope_limit') for c in pair):return None
    left,right=[c['boundary'] for c in pair]
    lo,hi=left['speed_kmh'],right['speed_kmh']
    from em_mach_events import solver_regions,overlaps
    if overlaps(solver_regions(solver),lo,hi):return None
    if not 0.<hi-lo<=2.*speed_tolerance(solver.config):return None
    tolerance=turn_tolerance(solver.config['sep_tolerance_mps'])

    def sample(speed,a,b):
        column=sample_boundary_column((name,config_json,speed,[a,b]))
        point=column.get('boundary')
        return point if column['boundary_status']=='verified limit' and point and point['valid'] else None

    def continuous(a,b):
        if a['speed_kmh']==b['speed_kmh']:return [a]
        points=[a]
        for fraction in (1./3.,2./3.):
            speed=a['speed_kmh']+(b['speed_kmh']-a['speed_kmh'])*fraction
            point=sample(speed,a,b)
            if point is None or point.get('roll_leveling_branch')!=a.get('roll_leveling_branch'):return None
            predicted=a['turn_dps']+(b['turn_dps']-a['turn_dps'])*fraction
            if abs(point['turn_dps']-predicted)>tolerance:return None
            points.append(point)
        return [*points,b]

    common=dict(speed_interval_kmh=[lo,hi])
    from em_mach_events import catalog,crossings,certificate
    if crossings(catalog(solver)['events'],left,right):
        return certificate(solver,pair,sample)
    if left.get('roll_leveling_branch')==right.get('roll_leveling_branch'):
        points=continuous(left,right)
        if points is None:return None
        return dict(common,points=points,method='linear boundary interpolation checked at one-third and two-thirds speeds')

    branches={left.get('roll_leveling_branch'),right.get('roll_leveling_branch')}
    if (not solver.fm.get('RollLeveling',True) or branches not in ({0,1},{-1,0})
            or any(p.get('sideslip_attitude_deg',0.) for p in (left,right))):return None
    sign=1. if 1 in branches else -1.
    alpha=sign*(12.-ROLL_LEVELING_PROBE_OFFSET)

    def incoming(speed,seed):
        point=(fixed_alpha(solver,speed,alpha,seed) or
               fixed_alpha(solver,speed,alpha,seed,canonical=True))
        if (point is None or not point['converged'] or point.get('roll_leveling_branch')!=0
                or point['load_g']<=1. or not set(point['reasons'])<=PHYSICAL_REASONS):return None
        return point

    a,b=left,right
    pa,pb=incoming(lo,a),incoming(hi,b)
    if pa is None or pb is None or pa['valid']==pb['valid']:return None


    if any(p['valid']!=(edge['roll_leveling_branch']!=0) for p,edge in ((pa,a),(pb,b))):return None
    original_sides=pa,pb
    target=min(.01,.05*speed_tolerance(solver.config))
    for _ in range(12):
        if b['speed_kmh']-a['speed_kmh']<=target:break
        speed=(a['speed_kmh']+b['speed_kmh'])*.5
        point=incoming(speed,pa)
        if point is None:return None
        if point['valid']==pa['valid']:a=pa=point
        else:b=pb=point
    else:return None
    a=sample(a['speed_kmh'],left,right) if a is not left else left
    b=sample(b['speed_kmh'],left,right) if b is not right else right
    if (a is None or b is None or a.get('roll_leveling_branch')!=left.get('roll_leveling_branch')
            or b.get('roll_leveling_branch')!=right.get('roll_leveling_branch')):


        if hi-lo>speed_tolerance(solver.config):return None
        a,b=left,right;pa,pb=original_sides
    rejected=pb if pa['valid'] else pa
    outgoing=fixed_alpha(solver,rejected['speed_kmh'],sign*(12.+ROLL_LEVELING_PROBE_OFFSET),rejected)
    if (outgoing is None or not outgoing['valid'] or outgoing.get('roll_leveling_branch')!=int(sign)
            or outgoing['load_g']<=rejected['load_g']):return None
    before,after=continuous(left,a),continuous(b,right)
    if before is None or after is None:return None
    def evidence(point):
        return {k:point[k] for k in ('speed_kmh','load_g','alpha_deg','valid','reasons',
                                    'force_error_g','angular_error_rad_s2','roll_leveling_branch')}
    return dict(common,points=before+after,
        method='balanced boundary limits across a bracketed native roll-leveling transition',
        transition_speed_interval_kmh=[a['speed_kmh'],b['speed_kmh']],
        native_alpha_deg=sign*12.,incoming_checks=[evidence(pa),evidence(pb)],
        outgoing_check=evidence(outgoing))


def boundary_intervals(intervals,max_width):
    merged=[]
    for lo,hi in sorted(set(map(tuple,intervals))):
        if merged and lo==merged[-1][1] and hi-merged[-1][0]<=max_width:
            merged[-1]=(merged[-1][0],hi)
        else:merged.append((lo,hi))
    return merged


def apply_outline(boundary,certificates):
    for certificate in certificates:
        lo,hi=certificate['speed_interval_kmh']
        transition=certificate.get('transition_speed_interval_kmh')
        points=[]
        for sample in certificate['points']:
            point=dict(speed_kmh=sample['speed_kmh'],turn_dps=sample['turn_dps'],at_plot_ceiling=False)
            if certificate.get('mach_event'):
                point.update(edge_kind='Mach-polar boundary transition',
                    mach_event=certificate['mach_event'])
                if certificate.get('native_mach_uncertainty'):
                    point.update(edge_kind='Mach transition; sampled numerical uncertainty',
                        native_mach_uncertainty=certificate['native_mach_uncertainty'])
            if transition and sample['speed_kmh'] in transition and not certificate.get('mach_event'):
                point.update(speed_kmh=sum(transition)*.5,sample_speed_kmh=sample['speed_kmh'],
                             vertical_edge=True,edge_kind=certificate.get('transition_kind','Roll-leveling boundary transition'))
            points.append(point)
        if transition and certificate.get('mach_event'):
            points.append(dict(speed_kmh=sum(transition)*.5,turn_dps=None,
                               at_plot_ceiling=False,edge_kind='Native Mach discontinuity'))
        boundary=[p for p in boundary if not lo<=p['speed_kmh']<=hi]+points
        boundary.sort(key=lambda p:p['speed_kmh'])
    return boundary
