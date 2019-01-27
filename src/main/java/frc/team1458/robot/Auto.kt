package frc.team1458.robot

import frc.team1458.lib.util.maths.TurtleMaths
import frc.team1458.lib.drive.ClosedLoopTank
import frc.team1458.lib.odom.EncoderOdom
import frc.team1458.lib.pathfinding.PurePursuitFollower
import frc.team1458.lib.sensor.interfaces.AngleSensor
import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.flow.systemTimeSeconds
import kotlin.math.*

class Auto(
    private val robot: Robot,
    private val drivetrain: ClosedLoopTank,
    private val odometry: EncoderOdom,
    private val gyroscope: AngleSensor = odometry.gyro,
    private val automaticSpeedControl: Boolean = false, // TODO Implement
    private val putToDashboard: Boolean = false,
    clearSensorsOnInit: Boolean = true
) {
    private val absoluteMaximumVelocity = 8.00 // feet/second - Absolute maximum velocity the robot can spin the wheels
    private val setVelocity = 4.00 // feet/second - Overall speed (this would be speed if going perfectly straight)
    private val purePursuitScaling = 1.00 // Scaling factor
    private val purePursuitTargetTolerance = 0.60 // feet - Tolerance to reaching the point being followed
    private val wheelbaseDistance = 1.96 // feet - Distance between wheels; could change as a tuning parameter possibly
    private val lookaheadDistance = 1.80 // feet - Higher values make smoother, easier-to-follow path but less precise

     init {
         if (clearSensorsOnInit) {
             resetAutonomousSensors()
         }
    }

    fun startBasicPurePursuit(pathPoints: Array<Pair<Double, Double>>, rampDownTime: Double = 0.0, clearSensorsOnStart: Boolean = true) {
        var driveVelocity: Pair<Double, Double> = Pair(0.0, 0.0)
        val purePursuitFollower = PurePursuitFollower(
            points = pathPoints,
            lookahead = lookaheadDistance,
            scaling = purePursuitScaling,
            wheelbase = wheelbaseDistance,
            targetTolerance = purePursuitTargetTolerance
        )

        drivetrain.setDriveVelocity(0.0, 0.0)

        if (clearSensorsOnStart) {
            resetAutonomousSensors()
        }

        while (robot.isAutonomous && robot.isEnabled && !purePursuitFollower.finished(
                Pair(odometry.pose.x, odometry.pose.y))
        ) {
            odometry.update()

            if (putToDashboard) {
                LiveDashboard.putOdom(odometry.pose)
            }

            driveVelocity =
                purePursuitFollower.getControl(Pair(odometry.pose.x, odometry.pose.y), odometry.pose.theta, setVelocity, putToDashboard)

            driveVelocity = Pair(TurtleMaths.constrain(driveVelocity.first, -1.0 * absoluteMaximumVelocity, absoluteMaximumVelocity),
                TurtleMaths.constrain(driveVelocity.second, -1.0 * absoluteMaximumVelocity, absoluteMaximumVelocity))

            drivetrain.setDriveVelocity(driveVelocity.first, driveVelocity.second)

            delay(5) // TODO Maybe try maximum throughput so disable?
        }

        // TODO Make sure ramp down velocity works / needs tweaks
        if (robot.isAutonomous && robot.isEnabled && rampDownTime != 0.0 && rampDownTime > 0.0) {
            var timeLeft: Double
            var scalingFactor: Double
            val endTime = systemTimeSeconds + rampDownTime

            while (endTime >= systemTimeSeconds) {
                timeLeft = endTime - systemTimeSeconds
                scalingFactor = (timeLeft / rampDownTime)

                if (scalingFactor > 1.01) {
                    println("Warning: Velocity Ramp-Down Scaling Issue Detected! - $scalingFactor")
                    break
                }

                drivetrain.setDriveVelocity(driveVelocity.first * scalingFactor, driveVelocity.second * scalingFactor)
            }
        }

        drivetrain.setDriveVelocity(0.0, 0.0)
    }

    fun resetAutonomousSensors(zeroEncoders: Boolean = true,
                               zeroGyroscope: Boolean = true,
                               resetOdometry: Boolean = true,
                               clearDashboard: Boolean = true)
    {
        if (putToDashboard && clearDashboard) {
            LiveDashboard.endPath()
        }

        if (zeroEncoders) {
            drivetrain.leftMaster.connectedEncoder.zero()
            drivetrain.rightMaster.connectedEncoder.zero()
        }

        if (zeroGyroscope) {
            gyroscope.zero()
        }

        if (resetOdometry) {
            odometry.clear()
            odometry.setup()
            odometry.update()
        }
    }

    fun turnInPlaceToAngleOdometry(angleDegree: Double,
                                   maxAngularVelocity: Double,
                                   maxAngularAcceleration: Double,
                                   angleToleranceDegree: Double = 5.0)
    {
        drivetrain.setDriveVelocity(0.0, 0.0)
        // TODO Finish
    }

    fun turnInPlaceToAngle(angleDeltaDegree: Double,
                           maxAngularVelocity: Double = 1.0,
                           angleToleranceDegree: Double = 5.0)
    {
        drivetrain.setDriveVelocity(0.0, 0.0)

        val startRads = odometry.pose.theta
        val halfRads = (angleDeltaDegree * 0.0174533) / 2.0

        val highTolerance = (angleDeltaDegree + angleToleranceDegree) * 0.0174533
        val lowTolerance = (angleDeltaDegree - angleToleranceDegree) * 0.0174533

        var deltaRads = 0.0
        var velocityScale: Double
        var newTheta: Double
        var oldTheta = 0.0

        while (robot.isAutonomous && robot.isEnabled && deltaRads > highTolerance || deltaRads < lowTolerance) {
            newTheta = odometry.pose.theta

            if (newTheta != oldTheta) {
                velocityScale = (-1.0 * abs(((1/halfRads) * deltaRads) - ((deltaRads) * (1/deltaRads)))) + 1.0

                drivetrain.setDriveVelocity(maxAngularVelocity * velocityScale, (-1.0 * maxAngularVelocity) * velocityScale)

                deltaRads = odometry.pose.theta - startRads
                oldTheta = newTheta
            }

            delay(5) // TODO Maybe try maximum throughput so disable?
        }

        drivetrain.setDriveVelocity(0.0, 0.0)
    }


    fun lineUpToTargetComputed(targetPositionFromSensor: Pair<Double, Double>, targetAngle: Double) {
        // TODO Finish
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
            val targetAngle: Double = atan((x1 - ymid) / (y1 - xmid)) // TODO maybe y/x
        } else if (x2 > xmid) {
            val targetAngle: Double = atan((x2 - ymid) / (y2 - xmid)) // TODO maybe y/x
        }

        // val targetPositionFromSensor = Pair(x, y)
        // TODO Finish
    }
}