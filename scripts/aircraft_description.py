def description(config):
    return dict(mode='discrete', revision='native-step-trim',
        equilibrium='Discrete aircraft step with settled aerodynamic histories',
        ps='Discrete energy change over one airborne step',
        reporting_hz=config['timestep_hz'], force_reference_hz=config['timestep_hz'],
        propulsion='Stationary propulsion')
