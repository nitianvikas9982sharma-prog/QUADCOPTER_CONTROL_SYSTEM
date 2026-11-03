import sys
import os
# ============================================================
# PROJECT PATH
# ============================================================

sys.path.append(
    os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..')
    )
)

# ========
# IMPORTS
# ========
from PLOTS_PID_LQR import plot_flight_states as plo
from Visulation_compare_3D import CompareVisualizer as DV
import numpy as np
import matplotlib.pyplot as plt
from main_LQR import lqrmain
from main_PID import pidmain

#================
#INITAL_POSITION
#================
inital_position=[float(x) for x in input("Enter the inital location of drone :").split()]
final_position=[float(x) for x in input("Enter the final location of drone :").split()]
total_time=20

spid=pidmain(inital_position,final_position,total_time)
slqr=lqrmain(inital_position,final_position,total_time)

X_histpid=spid.X_hist
X_histlqr=slqr.X_hist
time_vec=spid.time_vec

plo(X_histpid,X_histlqr, time_vec)
DV(X_histpid,X_histlqr, time_vec).run()

