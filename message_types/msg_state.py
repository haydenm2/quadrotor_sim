class msg_state:
    def __init__(self):
        self.pn = 0.      # inertial north position in meters
        self.pe = 0.      # inertial east position in meters
        self.pd = 0.      # inertial down position in meters
        self.u = 0.       # body x velocity in meters/sec
        self.v = 0.       # body y velocity in meters/sec
        self.w = 0.       # body z velocity in meters/sec
        self.phi = 0.     # roll angle in radians
        self.theta = 0.   # pitch angle in radians
        self.psi = 0.     # yaw angle in radians
        self.p = 0.       # roll rate in radians/sec
        self.q = 0.       # pitch rate in radians/sec
        self.r = 0.       # yaw rate in radians/sec
