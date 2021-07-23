# Choregraphe bezier export in Python.
from naoqi import ALProxy
def Ready(self):
    """准备接杆"""

    names = list()
    times = list()
    keys = list()

    names.append("LAnklePitch")
    times.append([0.88])
    keys.append([[0.0942478, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LAnkleRoll")
    times.append([0.88])
    keys.append([[-0.10821, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([0.88])
    keys.append([[-0.741765, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([0.88])
    keys.append([[-1.59174, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([0.88])
    keys.append([[1, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LHipPitch")
    times.append([0.88])
    keys.append([[0.1309, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LHipRoll")
    times.append([0.88])
    keys.append([[0.109956, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LHipYawPitch")
    times.append([0.88])
    keys.append([[-0.171042, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LKneePitch")
    times.append([0.88])
    keys.append([[-0.0925025, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([0.88])
    keys.append([[0.579449, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([0.88])
    keys.append([[-0.0610865, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([0.88])
    keys.append([[0, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RAnklePitch")
    times.append([0.88])
    keys.append([[0.0820305, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RAnkleRoll")
    times.append([0.88])
    keys.append([[0.106465, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([0.88])
    keys.append([[0.879646, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([0.88])
    keys.append([[1.44862, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([0.88])
    keys.append([[1, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RHipPitch")
    times.append([0.88])
    keys.append([[0.124914, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RHipRoll")
    times.append([0.88])
    keys.append([[-0.115192, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RHipYawPitch")
    times.append([0.88])
    keys.append([[-0.171042, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RKneePitch")
    times.append([0.88])
    keys.append([[-0.0925025, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([0.88])
    keys.append([[0.654498, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([0.88])
    keys.append([[0.0244346, [3, -0.306667, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([0.88])
    keys.append([[0, [3, -0.306667, 0], [3, 0, 0]]])

    try:
        # uncomment the following line and modify the IP if you use this script outside Choregraphe.
        # motion = ALProxy("ALMotion", IP, 9559)
        motion = ALProxy("ALMotion")
        motion.angleInterpolationBezier(names, times, keys)
    except BaseException, err:
        print err
    def HoldingPole(self):
        """收杆"""
        names = list()
        times = list()
        keys = list()

        names.append("LAnklePitch")
        times.append([6.88])
        keys.append([[0.0942478, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("LAnkleRoll")
        times.append([6.88])
        keys.append([[-0.10821, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("LElbowRoll")
        times.append([2.68, 3.76, 4.64, 5.56, 6.88])
        keys.append([[-0.74002, [3, -0.906667, 0], [3, 0.36, 0]], [-0.733038, [3, -0.36, 0], [3, 0.293333, 0]],
                     [-0.733038, [3, -0.293333, 0], [3, 0.306667, 0]], [-0.733038, [3, -0.306667, 0], [3, 0.44, 0]],
                     [-1.40324, [3, -0.44, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([5.56, 6.88])
        keys.append([[-1.61617, [3, -1.86667, 0], [3, 0.44, 0]], [-1.56731, [3, -0.44, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([2.68, 5.56, 6.88])
        keys.append(
            [[0, [3, -0.906667, 0], [3, 0.96, 0]], [0, [3, -0.96, 0], [3, 0.44, 0]], [0, [3, -0.44, 0], [3, 0, 0]]])

        names.append("LHipPitch")
        times.append([6.88])
        keys.append([[0.1309, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("LHipRoll")
        times.append([6.88])
        keys.append([[0.109956, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("LHipYawPitch")
        times.append([6.88])
        keys.append([[-0.171042, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("LKneePitch")
        times.append([6.88])
        keys.append([[-0.0923279, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([3.76, 4.64, 5.56, 6.88])
        keys.append([[0.680678, [3, -1.26667, 0], [3, 0.293333, 0]], [0.680678, [3, -0.293333, 0], [3, 0.306667, 0]],
                     [0.680678, [3, -0.306667, 0], [3, 0.44, 0]], [1.494, [3, -0.44, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([5.56, 6.88])
        keys.append([[0.00523599, [3, -1.86667, 0], [3, 0.44, 0]], [-0.0610865, [3, -0.44, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([2.68, 3.76, 5.56, 6.88])
        keys.append([[0, [3, -0.906667, 0], [3, 0.36, 0]], [-1.5708, [3, -0.36, 0], [3, 0.6, 0]],
                     [-1.5708, [3, -0.6, 0], [3, 0.44, 0]], [-1.5708, [3, -0.44, 0], [3, 0, 0]]])

        names.append("RAnklePitch")
        times.append([6.88])
        keys.append([[0.0820305, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("RAnkleRoll")
        times.append([6.88])
        keys.append([[0.106465, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([2.68, 3.76, 4.64, 5.56, 6.88])
        keys.append([[0.879646, [3, -0.906667, 0], [3, 0.36, 0]], [0.74351, [3, -0.36, 0], [3, 0.293333, 0]],
                     [0.747001, [3, -0.293333, -0.00349067], [3, 0.306667, 0.00364934]],
                     [0.968658, [3, -0.306667, -0.109197], [3, 0.44, 0.156674]], [1.54462, [3, -0.44, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([3.76, 4.64, 5.56, 6.88])
        keys.append([[1.56731, [3, -1.26667, 0], [3, 0.293333, 0]],
                     [1.57989, [3, -0.293333, -0.00131381], [3, 0.306667, 0.00137353]],
                     [1.58127, [3, -0.306667, 0], [3, 0.44, 0]], [1.58127, [3, -0.44, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([2.68, 3.76, 4.64, 5.56, 6.88])
        keys.append([[1, [3, -0.906667, 0], [3, 0.36, 0]], [1, [3, -0.36, 0], [3, 0.293333, 0]],
                     [0, [3, -0.293333, 0], [3, 0.306667, 0]], [0, [3, -0.306667, 0], [3, 0.44, 0]],
                     [0, [3, -0.44, 0], [3, 0, 0]]])

        names.append("RHipPitch")
        times.append([6.88])
        keys.append([[0.124914, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("RHipRoll")
        times.append([6.88])
        keys.append([[-0.115192, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("RHipYawPitch")
        times.append([6.88])
        keys.append([[-0.171042, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("RKneePitch")
        times.append([6.88])
        keys.append([[-0.0923279, [3, -2.30667, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([3.76, 4.64, 5.56, 6.88])
        keys.append([[0.802851, [3, -1.26667, 0], [3, 0.293333, 0]], [0.837758, [3, -0.293333, 0], [3, 0.306667, 0]],
                     [0.603884, [3, -0.306667, 0], [3, 0.44, 0]], [1.34565, [3, -0.44, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([4.64, 5.56, 6.88])
        keys.append([[0.0395692, [3, -1.56, 0], [3, 0.306667, 0]], [0.0418879, [3, -0.306667, 0], [3, 0.44, 0]],
                     [0.0418879, [3, -0.44, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([2.68, 3.76, 4.64, 5.56, 6.88])
        keys.append([[0, [3, -0.906667, 0], [3, 0.36, 0]], [-1.5708, [3, -0.36, 0], [3, 0.293333, 0]],
                     [-1.5708, [3, -0.293333, 0], [3, 0.306667, 0]], [-1.5708, [3, -0.306667, 0], [3, 0.44, 0]],
                     [-1.5708, [3, -0.44, 0], [3, 0, 0]]])

        try:
            # uncomment the following line and modify the IP if you use this script outside Choregraphe.
            # motion = ALProxy("ALMotion", IP, 9559)
            motion = ALProxy("ALMotion")
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err
    def AddressingTheBall(self):
        """准备击球"""
        names = list()
        times = list()
        keys = list()

        names.append("LAnklePitch")
        times.append([1.16])
        keys.append([[0.101229, [3, -0.4, 0], [3, 0, 0]]])

        names.append("LAnkleRoll")
        times.append([1.16])
        keys.append([[-0.109956, [3, -0.4, 0], [3, 0, 0]]])

        names.append("LHipPitch")
        times.append([1.16])
        keys.append([[0.132645, [3, -0.4, 0], [3, 0, 0]]])

        names.append("LHipRoll")
        times.append([1.16])
        keys.append([[0.113446, [3, -0.4, 0], [3, 0, 0]]])

        names.append("LHipYawPitch")
        times.append([1.16])
        keys.append([[-0.178024, [3, -0.4, 0], [3, 0, 0]]])

        names.append("LKneePitch")
        times.append([1.16])
        keys.append([[-0.0907571, [3, -0.4, 0], [3, 0, 0]]])

        names.append("RAnklePitch")
        times.append([1.16])
        keys.append([[0.0925025, [3, -0.4, 0], [3, 0, 0]]])

        names.append("RAnkleRoll")
        times.append([1.16])
        keys.append([[0.10821, [3, -0.4, 0], [3, 0, 0]]])

        names.append("RHipPitch")
        times.append([1.16])
        keys.append([[0.13439, [3, -0.4, 0], [3, 0, 0]]])

        names.append("RHipRoll")
        times.append([1.16])
        keys.append([[-0.115192, [3, -0.4, 0], [3, 0, 0]]])

        names.append("RHipYawPitch")
        times.append([1.16])
        keys.append([[-0.178024, [3, -0.4, 0], [3, 0, 0]]])

        names.append("RKneePitch")
        times.append([1.16])
        keys.append([[-0.0890118, [3, -0.4, 0], [3, 0, 0]]])

        try:
            # uncomment the following line and modify the IP if you use this script outside Choregraphe.
            # motion = ALProxy("ALMotion", IP, 9559)
            motion = ALProxy("ALMotion")
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err
    def Batting(self, speed):
        """击球"""
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([3.28])
        keys.append([[0.0628318, [3, -1.10667, 0], [3, 0, 0]]])

        names.append("HeadYaw")
        times.append([3.28])
        keys.append([[0.178024, [3, -1.10667, 0], [3, 0, 0]]])

        names.append("LAnklePitch")
        times.append([0.8])
        keys.append([[0.113446, [3, -0.28, 0], [3, 0, 0]]])

        names.append("LAnkleRoll")
        times.append([0.8])
        keys.append([[-0.109956, [3, -0.28, 0], [3, 0, 0]]])

        names.append("LElbowRoll")
        times.append([0.8, 1.68, 2.6, 3.28])
        keys.append([[-1.37183, [3, -0.28, 0], [3, 0.293333, 0]], [-0.925025, [3, -0.293333, 0], [3, 0.306667, 0]],
                     [-0.925025, [3, -0.306667, 0], [3, 0.226667, 0]], [-0.872665, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([0.8, 1.68, 2.6, 3.28])
        keys.append([[-1.57429, [3, -0.28, 0], [3, 0.293333, 0]], [-1.61617, [3, -0.293333, 0], [3, 0.306667, 0]],
                     [-1.5708, [3, -0.306667, 0], [3, 0.226667, 0]], [-1.57254, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([0.8, 1.68, 2.6, 3.28])
        keys.append([[0.03, [3, -0.28, 0], [3, 0.293333, 0]], [0.15, [3, -0.293333, 0], [3, 0.306667, 0]],
                     [0.15, [3, -0.306667, 0], [3, 0.226667, 0]], [0.15, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("LHipPitch")
        times.append([0.8])
        keys.append([[0.137881, [3, -0.28, 0], [3, 0, 0]]])

        names.append("LHipRoll")
        times.append([0.8])
        keys.append([[0.115192, [3, -0.28, 0], [3, 0, 0]]])

        names.append("LHipYawPitch")
        times.append([0.8])
        keys.append([[-0.178024, [3, -0.28, 0], [3, 0, 0]]])

        names.append("LKneePitch")
        times.append([0.8])
        keys.append([[-0.0802851, [3, -0.28, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([0.8, 1.68, 2.6, 3.28])
        keys.append([[1.44339, [3, -0.28, 0], [3, 0.293333, 0]], [0.680678, [3, -0.293333, 0], [3, 0.306667, 0]],
                     [0.680678, [3, -0.306667, 0], [3, 0.226667, 0]], [0.671952, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([0.8, 1.68, 2.6, 3.28])
        keys.append([[-0.0174533, [3, -0.28, 0], [3, 0.293333, 0]], [0.0331613, [3, -0.293333, 0], [3, 0.306667, 0]],
                     [0.0331613, [3, -0.306667, 0], [3, 0.226667, 0]], [0.0331613, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([0.8, 1.68, 2.6, 3.28])
        keys.append([[-1.54811, [3, -0.28, 0], [3, 0.293333, 0]],
                     [-1.65108, [3, -0.293333, 0.0200334], [3, 0.306667, -0.020944]],
                     [-1.67203, [3, -0.306667, 0], [2, 2.63974, -0.00180929]],
                     [0.918043, [2, -0.191221, -1.40694], [3, 0, 0]]])

        names.append("RAnklePitch")
        times.append([0.8])
        keys.append([[0.10472, [3, -0.28, 0], [3, 0, 0]]])

        names.append("RAnkleRoll")
        times.append([0.8])
        keys.append([[0.10821, [3, -0.28, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([0.8, 1.68, 2.6, 3.28])
        keys.append([[1.51495, [3, -0.28, 0], [3, 0.293333, 0]],
                     [1.06116, [3, -0.293333, 0.00166942], [3, 0.306667, -0.0017453]],
                     [1.05941, [3, -0.306667, 0], [3, 0.226667, 0]], [1.06116, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([0.8, 1.68, 3.28])
        keys.append([[1.56207, [3, -0.28, 0], [3, 0.293333, 0]],
                     [1.46782, [3, -0.293333, 0.0152763], [3, 0.533333, -0.0277751]],
                     [1.43292, [3, -0.533333, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([0.8, 1.68, 2.6, 3.28])
        keys.append([[0.04, [3, -0.28, 0], [3, 0.293333, 0]], [0, [3, -0.293333, 0], [3, 0.306667, 0]],
                     [1, [3, -0.306667, 0], [3, 0.226667, 0]], [1, [3, -0.226667, 0], [3, 0, 0]]])

        names.append("RHipPitch")
        times.append([0.8])
        keys.append([[0.136136, [3, -0.28, 0], [3, 0, 0]]])

        names.append("RHipRoll")
        times.append([0.8])
        keys.append([[-0.115192, [3, -0.28, 0], [3, 0, 0]]])

        names.append("RHipYawPitch")
        times.append([0.8])
        keys.append([[-0.178024, [3, -0.28, 0], [3, 0, 0]]])

        names.append("RKneePitch")
        times.append([0.8])
        keys.append([[-0.0785398, [3, -0.28, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([0.8, 1.68, 3.28])
        keys.append([[1.34914, [3, -0.28, 0], [3, 0.293333, 0]], [0.603884, [3, -0.293333, 0], [3, 0.533333, 0]],
                     [0.671952, [3, -0.533333, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([0.8, 1.68, 3.28])
        keys.append([[0.0174533, [3, -0.28, 0], [3, 0.293333, 0]], [-0.0331613, [3, -0.293333, 0], [3, 0.533333, 0]],
                     [-0.0331613, [3, -0.533333, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([0.8, 1.68, 2.6, 3.28])
        keys.append([[-1.52367, [3, -0.28, 0], [3, 0.293333, 0]], [-1.47829, [3, -0.293333, 0], [3, 0.306667, 0]],
                     [-1.5708, [3, -0.306667, 0], [3, 0.226667, 0]], [-1.5708, [3, -0.226667, 0], [3, 0, 0]]])

        try:
            # uncomment the following line and modify the IP if you use this script outside Choregraphe.
            # motion = ALProxy("ALMotion", IP, 9559)
            motion = ALProxy("ALMotion")
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err
    def ReceivingPole(self):
        """击球后收杆"""
        names = list()
        times = list()
        keys = list()

        names.append("LElbowRoll")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[-0.872665, [3, -0.253333, 0], [3, 0.28, 0]], [-0.872665, [3, -0.28, 0], [3, 0.293333, 0]],
                     [-0.872665, [3, -0.293333, 0], [3, 0.306667, 0]], [-1.40324, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[-1.5708, [3, -0.253333, 0], [3, 0.28, 0]], [-1.5708, [3, -0.28, 0], [3, 0.293333, 0]],
                     [-1.5708, [3, -0.293333, 0], [3, 0.306667, 0]], [-1.59174, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[0.15, [3, -0.253333, 0], [3, 0.28, 0]], [0.15, [3, -0.28, 0], [3, 0.293333, 0]],
                     [0.15, [3, -0.293333, 0], [3, 0.306667, 0]], [0, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[0.687597, [3, -0.253333, 0], [3, 0.28, 0]],
                     [0.696701, [3, -0.28, -0.00199909], [3, 0.293333, 0.00209428]],
                     [0.699877, [3, -0.293333, -0.00317649], [3, 0.306667, 0.00332087]],
                     [1.494, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[0.0388224, [3, -0.253333, 0], [3, 0.28, 0]],
                     [0.0462708, [3, -0.28, -0.00163553], [3, 0.293333, 0.00171341]],
                     [0.0488692, [3, -0.293333, 0], [3, 0.306667, 0]], [-0.0610865, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[-1.5708, [3, -0.253333, 0], [3, 0.28, 0]], [-1.5708, [3, -0.28, 0], [3, 0.293333, 0]],
                     [-1.5708, [3, -0.293333, 0], [3, 0.306667, 0]], [-1.5708, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[0.167552, [3, -0.253333, 0], [3, 0.28, 0]], [0.109956, [3, -0.28, 0], [3, 0.293333, 0]],
                     [1.29503, [3, -0.293333, -0.233797], [3, 0.306667, 0.244424]],
                     [1.54462, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[1.43292, [3, -0.253333, 0], [3, 0.28, 0]], [1.43292, [3, -0.28, 0], [3, 0.293333, 0]],
                     [1.51844, [3, -0.293333, 0], [3, 0.306667, 0]], [1.39452, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[1, [3, -0.253333, 0], [3, 0.28, 0]], [0.08, [3, -0.28, 0.0763636], [3, 0.293333, -0.08]],
                     [0, [3, -0.293333, 0], [3, 0.306667, 0]], [0, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[0.457276, [3, -0.253333, 0], [3, 0.28, 0]], [0.457276, [3, -0.28, 0], [3, 0.293333, 0]],
                     [0.776672, [3, -0.293333, -0.136524], [3, 0.306667, 0.142729]],
                     [1.29503, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[-0.0331613, [3, -0.253333, 0], [3, 0.28, 0]], [-0.0331613, [3, -0.28, 0], [3, 0.293333, 0]],
                     [-0.0331613, [3, -0.293333, 0], [3, 0.306667, 0]], [0.0418879, [3, -0.306667, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([0.72, 1.56, 2.44, 3.36])
        keys.append([[-1.5708, [3, -0.253333, 0], [3, 0.28, 0]], [-1.5708, [3, -0.28, 0], [3, 0.293333, 0]],
                     [-1.5708, [3, -0.293333, 0], [3, 0.306667, 0]], [-1.5708, [3, -0.306667, 0], [3, 0, 0]]])

        try:
            # uncomment the following line and modify the IP if you use this script outside Choregraphe.
            # motion = ALProxy("ALMotion", IP, 9559)
            motion = ALProxy("ALMotion")
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err
    def MoveStart(self):
        """击球结束后，姿势调整成移动准备动作"""
        names = list()
        times = list()
        keys = list()

        names.append("LAnklePitch")
        times.append([1.36])
        keys.append([[-0.350811, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LAnkleRoll")
        times.append([1.36])
        keys.append([[-0.00523599, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LElbowRoll")
        times.append([1.36])
        keys.append([[-1.40324, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LElbowYaw")
        times.append([1.36])
        keys.append([[-1.59174, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LHand")
        times.append([1.36])
        keys.append([[0, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LHipPitch")
        times.append([1.36])
        keys.append([[-0.450295, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LHipRoll")
        times.append([1.36])
        keys.append([[0, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LHipYawPitch")
        times.append([1.36])
        keys.append([[-0.00523599, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LKneePitch")
        times.append([1.36])
        keys.append([[0.706858, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LShoulderPitch")
        times.append([1.36])
        keys.append([[1.494, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LShoulderRoll")
        times.append([1.36])
        keys.append([[-0.0610865, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("LWristYaw")
        times.append([1.36])
        keys.append([[-1.5708, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RAnklePitch")
        times.append([1.36])
        keys.append([[-0.359538, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RAnkleRoll")
        times.append([1.36])
        keys.append([[0.00698132, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RElbowRoll")
        times.append([1.36])
        keys.append([[1.54462, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RElbowYaw")
        times.append([1.36])
        keys.append([[1.39452, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RHand")
        times.append([1.36])
        keys.append([[0, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RHipPitch")
        times.append([1.36])
        keys.append([[-0.462512, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RHipRoll")
        times.append([1.36])
        keys.append([[-0.0383972, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RHipYawPitch")
        times.append([1.36])
        keys.append([[-0.00523599, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RKneePitch")
        times.append([1.36])
        keys.append([[0.703368, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RShoulderPitch")
        times.append([1.36])
        keys.append([[1.29503, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RShoulderRoll")
        times.append([1.36])
        keys.append([[0.0418879, [3, -0.466667, 0], [3, 0, 0]]])

        names.append("RWristYaw")
        times.append([1.36])
        keys.append([[-1.5708, [3, -0.466667, 0], [3, 0, 0]]])

        try:
            # uncomment the following line and modify the IP if you use this script outside Choregraphe.
            # motion = ALProxy("ALMotion", IP, 9559)
            motion = ALProxy("ALMotion")
            motion.angleInterpolationBezier(names, times, keys)
        except BaseException, err:
            print err
