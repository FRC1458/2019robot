package frc.team1458.lib.odom

import frc.team1458.lib.sensor.interfaces.*
import frc.team1458.lib.util.maths.TurtleMaths
import java.lang.Math.*

class EncoderOdom(val left: DistanceSensor, val right: DistanceSensor, val gyro: AngleSensor) {

    // TODO add interp and vision integration

    var pose: Pose2D = Pose2D(0.0, 0.0, 0.0)

    var lastLeft = 0.0
    var lastRight = 0.0

    init {
        println("Odometry Created")
    }

    fun clear() {
        lastLeft = 0.0
        lastRight = 0.0
        pose = Pose2D(0.0, 0.0, 0.0)

    }

    fun setup() {
        lastLeft = left.distanceFeet
        lastRight = right.distanceFeet
    }

    fun update() {
        val dl = left.distanceFeet - lastLeft
        val dr = right.distanceFeet - lastRight
        val gyroRads = gyro.radians

        lastLeft = left.distanceFeet
        lastRight = right.distanceFeet

        val fwd = (dl + dr).toDouble() / 2.0

        // TODO - use proper differential arc approximation
        // avg approximation, see 2004 update http://rossum.sourceforge.net/papers/DiffSteer/#d7
        val theta = TurtleMaths.constrainAngle((/*pose.theta +*/ gyroRads) / /*2.0*/ 1.0)

        println("Gyro Angle: " + toDegrees(gyroRads))
        println("fwd: $fwd")
        println("theta: " + toDegrees(theta))

        pose = Pose2D(pose.x + fwd * cos(theta), pose.y + fwd * sin(theta), gyroRads)
    }
}
