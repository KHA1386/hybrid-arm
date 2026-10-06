def addHeader(rootNode):
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels hideForceFields hideWireframe')

    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('BackgroundSetting', color=[0.53, 0.81, 0.92, 1])
    rootNode.findData('dt').value = 0.02
    rootNode.gravity = [0, -9810, 0]


def createScene(rootNode):
    addHeader(rootNode)
    rootNode.addObject('RequiredPlugin', name='ExternalPlugins',
                       pluginName=['BeamAdapter', 'SoftRobots'])
