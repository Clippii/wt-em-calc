import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
if __name__=='__main__':
    os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
    os.environ.setdefault('OMP_NUM_THREADS','1')
    os.environ.setdefault('WT_ALTITUDE_OUTPUT_DIR',str(ROOT/'outputs/altitude'))
    os.execv(sys.executable,[sys.executable,str(ROOT/'scripts/altitude_server.py'),*sys.argv[1:]])
