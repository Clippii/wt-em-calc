def afterburner_command(requested, boost_type, controllable):
    return bool(requested and ((boost_type > 0 and controllable) or boost_type in (4, 8, 10)))
