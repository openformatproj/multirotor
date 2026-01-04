import pybullet_data
import simulator.conf as conf

def generate_world(engine):
    engine.setGravity(0, 0, conf.G)
    engine.setAdditionalSearchPath(pybullet_data.getDataPath())
    engine.loadURDF("plane.urdf")
    # terrain_shape = engine.createCollisionShape(shapeType=engine.GEOM_HEIGHTFIELD, 
    #                                             meshScale=[0.05, 0.05, 1],
    #                                             fileName=pybullet_data.getDataPath() + "/heightmaps/wm_height_out.png")
    # terrain = engine.createMultiBody(0, terrain_shape)
    # engine.resetBasePositionAndOrientation(terrain, [0, 0, 0], [0, 0, 0, 1])