'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import SimpleXMLRPCServer
from collections import OrderedDict
import threading
import numpy as np
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ServerAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        server = SimpleXMLRPCServer.SimpleXMLRPCServer(('localhost', 9000), logRequests=True, allow_none=True)
        server.register_instance(self)
        server.register_introspection_functions()
        server.register_multicall_functions()

        thread = threading.Thread(target=server.serve_forever)
        thread.start()
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        self.recognize_posture(self.perception)
        return self.posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes
        self.angle_interpolation(keyframes, self.perception)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        self.forward_kinematics(self.perception.joint)
        
        Te = np.array([self.from_trans(self.transforms[self.chains[name][-1]])])
        return Te

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        keyframes = self.set_transform(self, effector_name, transform)
        self.execute_keyframes(keyframes)

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

