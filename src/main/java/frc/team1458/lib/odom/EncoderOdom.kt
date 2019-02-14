package frc.team1458.lib.odom

import frc.team1458.lib.sensor.interfaces.*
import frc.team1458.lib.util.maths.TurtleMaths
import frc.team1458.lib.util.flow.systemTimeMillis
import javafx.geometry.Pos
import java.lang.Math.* // TODO switch to kotlin math libs sometime
import java.util.*

class EncoderOdom(
    private val left: DistanceSensor,
    private val right: DistanceSensor,
    val gyro: AngleSensor,
    val latencyCompensation: Boolean
) {

    // TODO add interp and vision integration

    var pose: Pose2D = Pose2D(0.0, 0.0, 0.0)

    var lastLeft = 0.0
    var lastRight = 0.0

    private val bufferSize = 256
    private val latencyPositionBuffer : Queue<Pair<Double, Pose2D>> = LinkedList()

    init {
        if (latencyCompensation) {
            println("Odometry Latency Compensation Enabled")
        }
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

    fun getPoseAtTime(deltaTimeMs: Double): Pose2D? {
        val now = systemTimeMillis
        val timeAt = (now - deltaTimeMs)

        if (latencyPositionBuffer.size >= 2) {
            for (tuple in latencyPositionBuffer) {
                if (tuple.first > timeAt) {
                    val lastTime = tuple
                }

                else {
                    val nextTime = tuple
                    break
                }
            }
        }

        return null

    }

    fun update() {
        val dl = left.distanceFeet - lastLeft
        val dr = right.distanceFeet - lastRight

        val gyroRads = gyro.radians

        lastLeft = left.distanceFeet
        lastRight = right.distanceFeet

        val fwd = (dl + dr) / 2.0

        // TODO - use proper differential arc approximation
        // avg approximation, see 2004 update http://rossum.sourceforge.net/papers/DiffSteer/#d7
        val theta = TurtleMaths.constrainAngle((/*pose.theta +*/ gyroRads) / /*2.0*/ 1.0)

        pose = Pose2D(pose.x + fwd * cos(theta), pose.y + fwd * sin(theta), gyroRads)

        if (latencyCompensation) {
            latencyPositionBuffer.add(Pair(systemTimeMillis, pose)) // TODO maybe not system time? match time?

            if (latencyPositionBuffer.size > bufferSize) {
                latencyPositionBuffer.remove()
            }
        }
    }
}
