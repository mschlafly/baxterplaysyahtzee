

import numpy as np

def form_T(R,p):
    T=np.identity(4)
    T[0:3,0:3]=R
    p=np.array(p)
    try:
        T[0:3,3:4]=p[0:3,0]
    except:
        for i in range(3):
            T[i,3]=p[i]
    return T

def get_Rp_from_T(T):
    R=T[0:3,0:3]
    p=T[0:3,3:4]
    return (R,p)   

