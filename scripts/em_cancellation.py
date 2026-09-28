_event=None


def initialize(event):
    global _event
    _event=event


def check():
    if _event is not None and _event.is_set():raise InterruptedError('Calculation cancelled')
    from em_trim_work import check_deadline
    check_deadline()
