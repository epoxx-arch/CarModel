

class Position():
    def __init__(self,x = 0,y = 0) -> None:
        self.x = x
        self.y = y


class Trajectory

class Obstacle():
    
    def __init__(self, x= 0,y=0,init_speed = 1,init_omega = 0.3,width  = 1,length = 3, is_random_theta_speed = False,is_ramdom_speed = False):
        self.width = width
        self.length = length
        self.init_speed = init_speed
        self.init_omega = init_omega
        self.init_position = Position(x,y)
        self.if_random_speed = is_ramdom_speed 
        self.if_random_theta_speed = is_random_theta_speed 
        self.tra = [] 



