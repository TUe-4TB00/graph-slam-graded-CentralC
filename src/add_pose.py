
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)
d=gtsam.Rot2.fromDegrees(90)

degToRad = np.pi/180


def add_pose(graph, initial_estimate):
    # TODO: Add the odometry factor between X(3) and X(4) to the graph (BetweenFactorPose2)
    step1 = gtsam.Pose2(0.0, 0.0, 45.0*degToRad)
    step2 = gtsam.Pose2(2.0, 0.0, 0.0)
    step3 = gtsam.Pose2(0.0, 0.0, 45.0*degToRad)

    totalD = step1.compose(step2).compose(step3)

    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), totalD, ODOMETRY_NOISE))
    # TODO: Based on the odometry, find the initial estimate for the pose of X(4) and add it to the graph
    initial_estimate.insert(X(4), gtsam.Pose2(5.5, 1.5, 90.10*degToRad))

    return graph, initial_estimate