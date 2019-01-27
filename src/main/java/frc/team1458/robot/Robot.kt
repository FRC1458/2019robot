package frc.team1458.robot

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.sensor.*
import frc.team1458.lib.sensor.interfaces.*
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.core.BaseRobot
import frc.team1458.lib.input.interfaces.Switch
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.odom.EncoderOdom
import frc.team1458.lib.drive.ClosedLoopTank
import frc.team1458.lib.pathfinding.PathUtils
import frc.team1458.lib.pathfinding.Pose2DPathfinding
import frc.team1458.lib.pathfinding.PurePursuitFollower
import frc.team1458.lib.util.LiveDashboard
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

    // Elevator stuff
    val elevatorEnabled: Boolean = true
    val mag1 = Switch.fromDIO(8).inverted
    val mag2 = Switch.fromDIO(9).inverted
    val elev1 = SmartMotor.CANtalonSRX(20).inverted
    val elev2 = SmartMotor.CANtalonSRX(21).inverted
    var elevatorSpeed =  0.0

    // Autonomous stuff
    val gyro: AngleSensor = NavX.MXP_I2C().yaw.inverted
    val odom = EncoderOdom(dt.leftEnc, dt.rightEnc, gyro)
    var startTime: Double = 0.0
    // val coprocessor: Coprocessor = frc.team1458.lib.util.Coprocessor("http://10.0.1.70/telemetry")

    override fun robotSetup() {
        // println("Setup")

        // Don't zero later now
        dt.leftMaster.connectedEncoder.zero()
        dt.rightMaster.connectedEncoder.zero()
        gyro.zero()

        odom.setup()
        odom.update()

        LiveDashboard.setup(13.0, 13.0)
        LiveDashboard.endPath()
        // coprocessor.sendRequest()

    }

    override fun runAuto() {
        println("Warning: Sandstorm")

        // Clear the visualizer before we start auto
        LiveDashboard.endPath()

        dt.leftMaster.connectedEncoder.zero()
        dt.rightMaster.connectedEncoder.zero()
        gyro.zero()

        odom.clear()
        odom.setup()
        odom.update()

        // Square path
        // val path = PathUtils.generateLinearPath(arrayOf(Pair(0.0, 0.0), Pair(10.0, 0.0), Pair(6.0, 6.0), Pair(0.0, 6.0), Pair(0.0, 0.0)), 250)

        // TODO change number of points: has significant effect on any sort of driving , Pair(20.0, 10.0), Pair(20.0, -10.0)
        // val turnPath = PathUtils.interpolateTurnArcWithAngle(90.0, 0.0, Pair(15.0, 0.0), "left", 15, 5.0)
        val twoPointTurnPath = PathUtils.interpolateArcBetweenTwoPoints(Pose2DPathfinding(10.0, 0.0, 0.0), Pose2DPathfinding(15.0, 5.0, 90.0), 30, "left")
        val path_curvature = PathUtils.generateLinearPath(arrayOf(Triple(0.0, 0.0, 100000.0), *twoPointTurnPath, Triple(15.0, 20.0, 100000.0)/*, Pair(15.0, 15.0), Pair(0.0, 15.0), Pair(0.0, 5.0)/*Pair(0.0, 0.0), Pair(6.0, 0.0), Pair(7.0, 0.085), Pair(8.0, 0.35), Pair(9.0, 0.8), Pair(10.0, 1.53), Pair(11.0, 2.68), Pair(12.0, 6.0),Pair(12.0, 15.0)*/ */), 400)
        val path = PathUtils.removeCurvature(path_curvature)


        // TODO Play with lookahead as it greatly affects stability of PP algorithm
        val LOOKAHEAD = 1.80 // 1.8 // higher values make smoother, easier-to-follow path but less precise following, measured in FEET measured in FEET
        val SCALING = 0.98 // arbitrary(ish) factor
        val VELOCITY = 5.0 // feet per second overall speed (this would be speed if going perfectly straight)
        val WHEELBASE = 1.96 // feet - distance between wheels - could change as a tuning parameter possibly
        val pp = PurePursuitFollower(path, LOOKAHEAD, SCALING, WHEELBASE, 0.65)
        val returnHome = false

        val MAX_LINVEL = 5.0
        val MAX_LINACCEL = 3.5
        val MAX_CENTRIPITAL = 3.5

        val maxvel = path_curvature.map { min(MAX_LINVEL, sqrt(it.third * MAX_CENTRIPITAL)) }


        // println("\nEncoder Start Data - left_enc: " + dt.leftEnc.distanceInches + " right_enc: " + dt.rightEnc.distanceInches)

        while (isAutonomous && isEnabled && !pp.finished(Pair(odom.pose.x, odom.pose.y))) {
            odom.update()
            LiveDashboard.putOdom(odom.pose)

            var (l, r) = pp.getControl(Pair(odom.pose.x, odom.pose.y), odom.pose.theta, VELOCITY)
            // println("XY : (${odom.pose.x}, ${odom.pose.y})")

            // Precautionary velocity limit enforcement
            /*
            if (l > MAXVEL) {
                //l = MAXVEL
                // println("Warning: Velocity Limits Enforced!")
            } else if (l < (MAXVEL * -1.0)) {
                //l = (MAXVEL * -1.0)
                // println("Warning: Velocity Limits Enforced!")
            }
            if (r > MAXVEL) {
                //r = MAXVEL
                // println("Warning: Velocity Limits Enforced!")
            } else if (r < (MAXVEL * -1.0)) {
                //r = (MAXVEL * -1.0)
                // println("Warning: Velocity Limits Enforced!")
            }
            */
            dt.setDriveVelocity(l, r)

            delay(5)
        }


        /*val homePath = PathUtils.generateLinearPath(arrayOf(Pair(odom.pose.x, odom.pose.y) , Pair(0.0, 0.0)), 400)
        val ppReturnHome = PurePursuitFollower(homePath, LOOKAHEAD, SCALING, WHEELBASE, 0.55)

        delay(2000)

        while (returnHome && isAutonomous && isEnabled && !ppReturnHome.finished(Pair(odom.pose.x, odom.pose.y))) {
            odom.update()
            LiveDashboard.putOdom(odom.pose)

            var (l, r) = ppReturnHome.getControl(Pair(odom.pose.x, odom.pose.y), odom.pose.theta, VELOCITY)

            dt.setDriveVelocity(l, r)

            delay(3)
        }*/
    }

    override fun teleopInit() {
        // likely nothing here
        println("Start Data - left_enc: " + dt.leftEnc.distanceInches + " right_enc: " + dt.rightEnc.distanceInches)
    }

    override fun teleopPeriodic() {
        odom.update()
        LiveDashboard.putOdom(odom.pose)
        // SmartDashboard.putNumber("GyroAngle", gyro.heading)

        // println("left_enc: " + dt.leftEnc.distanceInches + " right_enc: " + dt.rightEnc.distanceInches)

        // drive code - runs around 50hz
        dt.arcadeDrive(
            if (drivetrainInverted) {
                -0.5 * (oi.throttleAxis.value)
            } else if (oi.slowDownButton.triggered) {
                0.5 * oi.throttleAxis.value
            } else {
                oi.throttleAxis.value
            },
            if (drivetrainInverted) {
                (oi.steerAxis.value)
            } else {
                oi.steerAxis.value
            }
        )

        // Elevator control code
        if (elevatorEnabled) {
            if (oi.elevatorUp.triggered && !mag1.triggered) {
                elevatorSpeed = 0.8
            }
            else if (oi.elevatorDown.triggered && !mag2.triggered) {
                elevatorSpeed = -0.8
            }
            else {
                elevatorSpeed =  0.0
            }

            elev1.speed = elevatorSpeed
            elev2.speed = elevatorSpeed
            println("angle " + elev2.connectedEncoder.angle)
        }

        // Intake control code
        if (intakeEnabled) {
            if (oi.intakeIn.triggered) {
                intake1.speed = 1.0
                intake2.speed = 1.0
            } else if (oi.intakeOut.triggered) {
                intake1.speed = -1.0
                intake2.speed = -1.0
            } else {
                intake1.speed = 0.0
                intake2.speed = 0.0
            }
        }
    }

    override fun runTest() {
        // rewind mechanism, run compressor, etc

        val turnPath = PathUtils.interpolateArcBetweenTwoPoints(Pose2DPathfinding(10.0, 0.0, 0.0), Pose2DPathfinding(15.0, 5.0, 90.0), 15, "left")

        for (i in 0..turnPath.size-1) {
            println(turnPath[i])
        }

        delay(20000)
    }

    override fun robotDisabled(
    ) {
        println("\nEncoder End Data - left_enc: " + dt.leftEnc.distanceInches + " right_enc: " + dt.rightEnc.distanceInches)
        LiveDashboard.putOdom(odom.pose)
        // coprocessor.sendRequest()
    }

    override fun disabledPeriodic() {
        LiveDashboard.putOdom(odom.pose)
        // coprocessor.sendRequest()
    }
}
