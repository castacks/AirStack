"""Offline test child: journal two records, exit abruptly without terminal.

Never used for simulator/runtime acquisition. Output constrained by recorder.
"""
import os
import sys
from reliable_recorder import DurableRecorder
if __name__=='__main__':
    recorder=DurableRecorder(sys.argv[1],'synthetic-crash','a'*64)
    os._exit(23)
