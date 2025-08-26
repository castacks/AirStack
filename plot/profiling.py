import time
import numpy as np

class TicToc:
    def __init__(self):
        self.dct = {}

    def tic(self, name):
        if name not in self.dct:
            self.dct[name] = {'elapsed': [], 'start': None}
        self.dct[name]['start'] = time.time()

    def toc(self, name):
        elapsed = time.time() - self.dct[name]['start']
        self.dct[name]['elapsed'].append(elapsed)
        self.dct[name]['start'] = None
        return elapsed

    def print_stats(self):
        s = ''
        for key, d in self.dct.items():
            s += key + '\n'
            s += '\tcalls: %d' % len(d['elapsed']) + '\n'
            s += '\tavg: %0.2f' % np.mean(d['elapsed']) + '\n'
            s += '\ttotal: %0.2f' % np.sum(d['elapsed']) + '\n'
            s += '\tmin: %0.2f' % np.min(d['elapsed']) + '\n'
            s += '\tmax: %0.2f' % np.max(d['elapsed']) + '\n'
        print(s)

    
tic_toc = TicToc()

def tic(name):
    tic_toc.tic(name)

def toc(name):
    tic_toc.toc(name)

def print_stats():
    tic_toc.print_stats()
