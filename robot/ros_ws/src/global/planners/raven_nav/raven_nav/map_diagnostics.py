"""Optional wall-time measurements; never change map acceptance or freshness."""
import functools
import os
import time


def profile_map_stage(function):
    @functools.wraps(function)
    def measured(self, *args, **kwargs):
        if os.getenv('RAVEN_MAP_PROFILE', 'false').lower() != 'true':
            return function(self, *args, **kwargs)
        started = time.monotonic()
        try:
            return function(self, *args, **kwargs)
        finally:
            elapsed = time.monotonic() - started
            self.get_logger().info(
                f'[map-profile] stage={function.__name__} wall_s={elapsed:.6f}')
    return measured
