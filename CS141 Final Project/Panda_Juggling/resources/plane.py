import pybullet as p
import os

class Plane:
    def __init__(self, client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'simpleplane.urdf')
        self.plane = p.loadURDF(fileName=f_name,
                   basePosition=[0, 0, 0],
                   physicsClientId=client)
        # SET UP SO BOUNCING IS ENABLED --> restitution of 1 might be too much
        p.changeDynamics(self.plane, -1, restitution=1.)

    def get_ids(self):
        return self.plane, self.client
