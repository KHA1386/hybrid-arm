from splib3.numerics import Quat, vsub, vadd
import numpy as np
import stlib3
from stlib3.scene import MainHeader

from scripts.Camera import createCamera
from scripts.NeedleProbe import createNeedleProbe, addProbe
from scripts.Robot import createRobot
from scripts.RobotGUI import RobotGUI
from scripts.Body import createBody, addCollisionToBody, addLesionToBody
from scripts.Header import addHeader
from importlib.machinery import SourceFileLoader
import os
import Sofa.Core

filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scripts', 'SceneParams.py')
params = SourceFileLoader('params', filename).load_module()

# Simulation Options
ROBOT_CONTROL = True
INVERSE = True

# Precomputed position
if ROBOT_CONTROL:
    robotBasePosition = params.robotBasePosition
    needleMecaPosition = params.initSoftExtremityPosition
    needleMecaOrientation = params.initSoftExtremityOrientation
    needleMecaPosition = vadd(needleMecaPosition, robotBasePosition)
    initAngles = params.initAngles
    initEffectorPosition = params.initEffectorPosition + params.initEffectorOrientation
    robotInitConfiguration = [initAngles, initEffectorPosition]
else:
    needleMecaPosition = [0, -70, -220]
    needleMecaOrientation = [-0.7, -0.1, -0.7, -0.1]

class KeyboardArmController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.target = kwargs.get("target")

    def onKeypressedEvent(self, e):
        key = e['key']
        pos = list(self.target.dofs.position.value[0])
        step = 5.0

        # pos[0]: X, pos[1]: Y, pos[2]: Z (4 giá trị phía sau là Quaternion xoay, giữ nguyên)
        if key == 'Up':          # Mũi tên lên: Di chuyển Y+
            pos[1] += step
        elif key == 'Down':      # Mũi tên xuống: Di chuyển Y-
            pos[1] -= step
        elif key == 'Right':     # Mũi tên phải: Di chuyển X+
            pos[0] += step
        elif key == 'Left':      # Mũi tên trái: Di chuyển X-
            pos[0] -= step
        elif key == 'I':         # Phím I: Tiến sâu trục Z
            pos[2] += step
        elif key == 'K':         # Phím K: Lùi trục Z
            pos[2] -= step

        self.target.dofs.position = [pos]


def createScene(rootNode):
    addHeader(rootNode)
    # Hệ màu RGBA, giá trị chạy từ 0.0 đến 1.0
    rootNode.addObject('RequiredPlugin', pluginName='ArticulatedSystemPlugin')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')
    rootNode.addObject('RequiredPlugin', name='ExternalPlugins',
                       pluginName=['BeamAdapter', 'SoftRobots'])
    if INVERSE:
        rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver' if INVERSE else 'BlockGaussSeidelConstraintSolver', printLog=0, tolerance=1e-8,
                       maxIterations=1000, epsilon=0.001)
    rootNode.addObject('CollisionResponse', response='FrictionContactConstraint', responseParams='mu=0.0')
    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('NewProximityIntersection', alarmDistance=3, contactDistance=0.1)

    ###############################
    # Needle Probe
    ###############################
    needleProbe = createNeedleProbe(rootNode, 'NeedleProbe',
                                    withProbe=False if ROBOT_CONTROL else True,
                                    withNeedle=True,
                                    position=[0, 0, 0] + needleMecaOrientation)

    ###############################
    # Needle & Probe Rest Shape
    ###############################
    needleRestShape = rootNode.addChild('NeedleRestShape')
    needleRestShape.addObject('MechanicalObject', name='dofs', template='Rigid3',
                              position=[list(np.copy(needleMecaPosition + needleMecaOrientation)) for i in range(3)],
                              showObject=False, showObjectScale=10, drawMode=1, showColor=[0, 255, 0, 255])

    softPartExtremity = None
    if ROBOT_CONTROL:
        ###################################################################
        # Robot with User Interface
        ###################################################################
        robot = createRobot(rootNode, translation=robotBasePosition, initConfiguration=robotInitConfiguration,
                            inverse=INVERSE, withEffector=False)
        robot.Articulations.joint0.maxAngle = 0
        robot.Articulations.joint0.minAngle = 0
        softPartExtremity = robot.SoftExtremity
        addProbe(softPartExtremity, 0)
        if INVERSE:
            robot.SoftExtremity.ProbeColli.addObject('PositionEffector', name='effector', template='Rigid3', indices=0,
                                                     effectorGoal=rootNode.EffectorTarget.dofs.findData(
                                                         'position').getLinkPath(),
                                                     useDirections=[1, 1, 1, 1, 1, 1])
            q = Quat(-0.5, -0.5, -0.3, 0.3)
            q.normalize()
            rootNode.EffectorTarget.dofs.position = [[10.743, -29.625, 17] + list(q)]
        needleProbe.addObject('RestShapeSpringsForceField', template='Rigid3', name='attachAll',
                      points=[0, 1, 2],
                      external_points=[0, 0, 0],
                      external_rest_shape=robot.SoftExtremity.dofs.getLinkPath(),
                      stiffness=1e6, angularStiffness=1e12)
        needleProbe.EulerImplicitSolver.rayleighMass = 0.0
        needleProbe.EulerImplicitSolver.rayleighStiffness = 0.0
        if not INVERSE:
            RobotGUI(robot)
    else:
        for i in range(3):
            needleProbe.addObject('RestShapeSpringsForceField', template='Rigid3', name='attach' + str(i), points=i,
                                  external_points=i, external_rest_shape=needleRestShape.dofs.getLinkPath(),
                                  stiffness=1e6, angularStiffness=1e12,
                                  drawSpring=False)

    needleProbe.init()  # BeamInterpolation acts weird when we translate the beam at init. TODO: find out why
    needleProbe.dofs.translation = needleMecaPosition

    ###############################
    # Camera
    ###############################
    camera = createCamera(needleProbe, [210, 0, 0, 0, 0, 0, 1], 0)

    ###############################
    # Body Mechanical
    ###############################
    tissue = createBody(rootNode, params)
    addCollisionToBody(tissue, params.prostateColliPath, 'ProstateCollision', params.rotation, params.translation)
    for i in range(4):
        addLesionToBody(tissue, params.lesionsPath[i], 'Lesion' + str(i), params.rotation, params.translation)
    if INVERSE:
        rootNode.addObject(KeyboardArmController(target=rootNode.EffectorTarget))
    return rootNode


