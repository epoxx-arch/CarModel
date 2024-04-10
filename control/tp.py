"""


"""

@dataclass
class State:
    s = 0 
    v = 0 
    a = 0 
    j = 0 


"""
s = vt + 1/2at^2  --> s(k+1) = s(k) + v(k)T + 1/2a(k)T^2 
v = v0 + at --> v(k+1) = v(k) + a(k)T
a = a  --> a(k+1) = (1-Ts/Tp) a(k) + 
"""
def model(state,a_des):
    