package frc.team1458.robot

import frc.team1458.lib.drive.ClosedLoopTank
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sin

class Auto(drivetrain: ClosedLoopTank) {
    val maximumVelocity = 2.0 // feet/second - Absolute maximum velocity the robot can spin the wheels
    val wheelbaseDistance = 1.96 // feet - Distance between wheels; could change as a tuning parameter possibly


    fun lineUpToTargetComputed(targetPositionFromSensor: Pair<Double, Double>, targetAngle: Double) {


    }

    fun lineUpToTargetUncomputed(
        distanceToTarget1: Double,
        distanceToTarget2: Double,
        angleToTarget: Double,
        sensorOffset: Pair<Double, Double> = Pair(0.0, 0.0)
    ) {

        val x1: Double = (cos(angleToTarget) * distanceToTarget1) + sensorOffset.first
        val y1: Double = (sin(angleToTarget) * distanceToTarget1) + sensorOffset.second

        val x2: Double = (cos(angleToTarget) * distanceToTarget2) + sensorOffset.first
        val y2: Double = (sin(angleToTarget) * distanceToTarget2) + sensorOffset.second

        val xmid: Double = (x1 + x2) / 2.0
        val ymid: Double = (y2 + y1) / 2.0

        if (x1 > xmid) {
            val targetAngle: Double = atan((x1 - ymid)/(y1 - xmid)) // TODO maybe y/x
        }

        else if (x2 > xmid) {
            val targetAngle: Double = atan((x2 - ymid)/(y2 - xmid)) // TODO maybe y/x
        }



        // val targetPositionFromSensor = Pair(x, y)


    }
}