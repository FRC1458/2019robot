package frc.team1458.lib.odom

import frc.team1458.lib.sensor.interfaces.*
import frc.team1458.lib.util.maths.TurtleMaths
import java.lang.Math.cos
import java.lang.Math.sin

class EncoderOdom(val left: DistanceSensor, val right: DistanceSensor, val gyro: AngleSensor) {

    // TODO add interp and vision integration

    var pose: Pose2D = Pose2D(0.0, 0.0, 0.0)

    var lastLeft = 0.0
    var lastRight = 0.0

    init {
        println("turtwig is gud")
    }

    fun setup() {
        lastLeft = left.distanceFeet
        lastRight = right.distanceFeet
    }

    fun update() {
        val dl = left.distanceFeet - lastLeft
        val dr = right.distanceFeet - lastRight
        lastLeft = left.distanceFeet
        lastRight = right.distanceFeet

        val fwd = (dl + dr).toDouble() / 2.0

        // avg approximation, see 2004 update http://rossum.sourceforge.net/papers/DiffSteer/#d7
        val theta = TurtleMaths.constrainAngle((pose.theta + gyro.radians) / 2.0)

        // TODO - use proper differential arc approximation
        pose = Pose2D(pose.x + fwd * cos(theta), pose.y + fwd * sin(theta), gyro.radians)
    }
}
