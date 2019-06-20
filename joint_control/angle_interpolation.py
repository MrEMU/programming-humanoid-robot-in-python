'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import rightBackToStand, hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        
        #Press k in Simspark to start the Keyframes, cause its set to the game time here
        time = perception.game_state.time
        names = keyframes[0]
        timelist = keyframes[1]
        output = 0.0
        
        #Iterating over the joints
        for i in range(len(names)):
            times = timelist[i]
            keys = keyframes[2][i]
            current = []
            last = []
            lefthandle = 0.0
            leftpoint = 0.0
            righthandle = 0.0
            rightpoint = 0.0
            t = 0.0
            
            #Iterating over the time frames to find the actual point in the sequence
            for j in range(len(times)):
                if j == 0 and time < times[j]: 
                    t = time/times[j]
                    current = keys[j]
                    leftpoint = 0.0
                    rightpoint = current[0]
                    lefthandle = 0.0
                    righthandle = rightpoint + current[1][2]
                    break
                elif time < times[j]:
                    t = (time-times[j-1])/(times[j]-times[j-1])
                    current = keys[j]
                    last = keys[j-1]
                    leftpoint = last[0]
                    rightpoint = current[0]
                    lefthandle = leftpoint + last[2][2]
                    righthandle = rightpoint + current[1][2]
                    break
                    
            #Computing the Beziere Point for current time
            output = (1-t)**3*leftpoint + 3*(1-t)**2*t*lefthandle + 3*(1-t)*t**2*righthandle + t**3*rightpoint
            target_joints[names[i]] = output
        #self.keyframes = ([], [], [])
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    #agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = hello()
    agent.run()
