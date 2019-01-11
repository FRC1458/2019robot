package frc.team1458.lib.odom

import frc.team1458.lib.util.maths.*

class Pose2D(val x: Double, val y: Double, theta: Double = 0.0) {
    
    val theta = TurtleMaths.constrainAngle(theta)

    companion object {
        fun interp(pose1: Pose2D, pose2: Pose2D, ratio: Double): Pose2D {
            // TODO - important replae linear with arc interpolation
            return Pose2D(pose1.x + ((pose2.x - pose1.x) * ratio), pose1.y + ((pose2.y - pose1.y) * ratio), TurtleMaths.constrainAngle(pose1.theta + ((pose2.theta - pose1.theta) * ratio)))
        }
    }
}
