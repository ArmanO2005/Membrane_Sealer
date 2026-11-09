import threading

# Shared stop flag for every controller. Blocking loops and sleeps go through
# sleep()/check() so a stop request unwinds whatever operation is running.
_event = threading.Event()


class AbortedError(Exception):
    pass


def request():
    _event.set()


def clear():
    _event.clear()


def is_requested():
    return _event.is_set()


def check():
    if _event.is_set():
        raise AbortedError("Operation aborted")


def sleep(seconds):
    """time.sleep() that raises AbortedError as soon as a stop is requested."""
    if _event.wait(max(seconds, 0)):
        raise AbortedError("Operation aborted")
