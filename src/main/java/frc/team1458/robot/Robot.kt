package frc.team1458.robot

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.sensor.*
import frc.team1458.lib.sensor.interfaces.*
import frc.team1458.lib.core.BaseRobot
import frc.team1458.lib.input.interfaces.Switch
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.odom.EncoderOdom
import frc.team1458.lib.drive.ClosedLoopTank
import frc.team1458.lib.pathfinding.*
import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.maths.TurtleMaths
import java.io.PrintWriter
import kotlin.math.min
import kotlin.math.sqrt

class Robot : BaseRobot() {

    val oi: OI = OI()
    val dt: ClosedLoopTank = ClosedLoopTank(
        leftMaster = SmartMotor.CANtalonSRX(16),
        rightMaster = SmartMotor.CANtalonSRX(22).inverted,
        leftMotors = arrayOf(SmartMotor.CANtalonSRX(25)),
        rightMotors = arrayOf(SmartMotor.CANtalonSRX(23).inverted),
        closedLoopControl = true,
        wheelDiameter = 0.5,
        closedLoopScaling = 6.0, // TODO: determine

        // TODO Don't forget PID I is disabled for autonomous testing as it introduces errors: maybe 0.0 for I
        pidConstantsLeft = PIDConstants(0.6, kI = 0.001/*3*/, kD = 0.00, kF = 1.0 / 1798.8), // These are decent
        pidConstantsRight = PIDConstants(0.750, kI = 0.001/*5*/, kD = 0.05, kF = 1.0 / 1806.4)// These are decent2
    )

    val drivetrainInverted: Boolean = false

    // Intake stuff
    val intakeEnabled: Boolean = false
    val intake1 = SmartMotor.CANtalonSRX(17).inverted
    val intake2 = SmartMotor.CANtalonSRX(19)

    // Autonomous stuff
    val gyro: AngleSensor = NavX.MXP_I2C().yaw.inverted
    val odom = EncoderOdom(dt.leftEnc, dt.rightEnc, gyro, latencyCompensation = true)
    val startPose = Pair(13.0, 13.0) // (x, y) of start coordinates

    var velocities: Array<Double> = arrayOf()
    var times: Array<Double> = arrayOf()

    //smooth accelerating
    var currInputValue: Double = 0.0
    val threshhold = 0.03 //To Be Determined

    var previousJoystick = oi.throttleAxis.value

    override fun robotSetup() {
        print("Robot Starting...")

        // Don't zero later now
        dt.leftMaster.connectedEncoder.zero()
        dt.rightMaster.connectedEncoder.zero()

        gyro.zero()

        odom.setup()
        odom.update()

        LiveDashboard.setup(startPose.first, startPose.second)
        LiveDashboard.endPath()
        println(" Done!")
    }

    override fun runAuto() {
        println("Warning: Sandstorm")

        // Square path
        // val path = PathUtils.generateLinearPath(arrayOf(Pair(0.0, 0.0), Pair(10.0, 0.0), Pair(6.0, 6.0), Pair(0.0, 6.0), Pair(0.0, 0.0)), 250)

        // TODO change number of points: has significant effect on any sort of driving , Pair(20.0, 10.0), Pair(20.0, -10.0)
        // val turnPath = PathUtils.interpolateTurnArcWithAngle(90.0, 0.0, Pair(15.0, 0.0), "left", 15, 5.0)
        val twoPointTurnPath = PathUtils.interpolateArcBetweenTwoPoints(Pose2DPathfinding(10.0, 0.0, 0.0), Pose2DPathfinding(15.0, 5.0, 90.0), 30, "left")
        val pathCurvature = PathUtils.generateLinearPath(arrayOf(Triple(0.0, 0.0, 100000.0), *twoPointTurnPath, Triple(15.0, 20.0, 100000.0)/*, Pair(15.0, 15.0), Pair(0.0, 15.0), Pair(0.0, 5.0)/*Pair(0.0, 0.0), Pair(6.0, 0.0), Pair(7.0, 0.085), Pair(8.0, 0.35), Pair(9.0, 0.8), Pair(10.0, 1.53), Pair(11.0, 2.68), Pair(12.0, 6.0),Pair(12.0, 15.0)*/ */), 400)
        val pathNoCurvature = PathUtils.removeCurvature(pathCurvature)


        val MAX_LINVEL = 5.0
        val MAX_LINACCEL = 3.5
        val MAX_CENTRIPITAL = 3.5


        val length = pathNoCurvature.toList()
            .zipWithNext { a, b -> TurtleMaths.distance(Pair(a.first, a.second), Pair(b.first, b.second)) }.sum()

        val maxvel = pathCurvature.map { min(MAX_LINVEL, sqrt(it.third * MAX_CENTRIPITAL)) }


        val (x, v) = PathUtils.generateVXGraph(length, maxvel.toTypedArray(), MAX_LINACCEL, 0.0, 0.0)
        val (t, vz) = PathUtils.xvTotv(x, v)

        for (i in 0 until t.size) {
            println("TVZ ${t[i]}, ${vz[i]}")

        }

        val vTimed = PathUtils.consistentTime(t, vz, 0.02)

        val auto = Auto(this, dt, odom, gyro, automaticSpeedControl = false, putToDashboard = true, velocities = v, distances = x) // v

        auto.startBasicPurePursuit(pathNoCurvature, 3.0)

    }
    override fun teleopInit() {
        // likely nothing here
        println("Robot Enabled!")
        println("let's think about what these ions are doing")
    }

    override fun teleopPeriodic() {
        odom.update()
        LiveDashboard.putOdom(odom.pose)

        // drive code - runs around 50hz
        var steerInput = oi.throttleAxis.value

        if (steerInput - previousJoystick > 0.2){
            steerInput = TurtleMaths.constrain(steerInput + 0.005, -1.0, 1.0)
        }

        else if (steerInput - previousJoystick < -0.2)
        {
            steerInput = TurtleMaths.constrain(steerInput - 0.005, -1.0, 1.0)
        }

        dt.arcadeDrive(
            when {
                drivetrainInverted -> -0.5 * (steerInput)
                oi.slowDownButton.triggered -> 0.5 * steerInput

                else -> oi.throttleAxis.value
            },
            if (drivetrainInverted) {
                (steerInput)
            } else {
                steerInput
            }
        )

        previousJoystick = oi.throttleAxis.value

        // Intake control code
        if (intakeEnabled) {
            when {
                oi.intakeIn.triggered -> {
                    intake1.speed = 1.0
                    intake2.speed = 1.0
                }
                oi.intakeOut.triggered -> {
                    intake1.speed = -1.0
                    intake2.speed = -1.0
                }
                else -> {
                    intake1.speed = 0.0
                    intake2.speed = 0.0
                }
            }
        }
    }

    override fun runTest() {

    }

    override fun robotDisabled() {
        dt.setDriveVelocity(0.0, 0.0)
        LiveDashboard.putOdom(odom.pose)
        println("Disabled")
    }

    override fun disabledPeriodic() {
        LiveDashboard.putOdom(odom.pose)
    }
}

