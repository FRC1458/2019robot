package frc.team1458.robot

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.sensor.*
import frc.team1458.lib.sensor.interfaces.*
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.core.BaseRobot
import frc.team1458.lib.drive.TankDrive
import frc.team1458.lib.input.interfaces.Switch
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.odom.EncoderOdom

import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.pathfinding.PathUtils
import frc.team1458.lib.pathfinding.PurePursuitFollower
import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.flow.systemTimeMillis
import frc.team1458.lib.util.flow.systemTimeSeconds

class Robot : BaseRobot() {

    val oi: OI = OI()
    val dt: TankDrive = TankDrive(
        leftMaster = SmartMotor.CANtalonSRX(16),
        rightMaster = SmartMotor.CANtalonSRX(22).inverted,
        leftMotors = arrayOf(SmartMotor.CANtalonSRX(25)),
        rightMotors = arrayOf(SmartMotor.CANtalonSRX(23).inverted),
        closedLoopControl = true,
        wheelDiameter = 0.5,
        closedLoopScaling = 6.0, // TODO: determine

        // TODO Don't forget PID I is disabled for autonomous testing as it introduces errors
        pidConstantsLowGearLeft = PIDConstants(0.5, kI = 0.0, kD = 0.05, kF = 1.0 / 1798.8), // These are decent
        pidConstantsLowGearRight = PIDConstants(0.5, kI = 0.0, kD = 0.05, kF = 1.0 / 1806.4), // These are decent2

        shifter = Solenoid.doubleSolenoid(extendChannel = 1, retractChannel = 0)
                + Solenoid.doubleSolenoid(extendChannel = 2, retractChannel = 3),

        pidConstantsHighGearLeft = PIDConstants(0.15, kI = 0.0, kD = 0.01, kF = 1.0 / 6530.5), // TODO: determine
        pidConstantsHighGearRight = PIDConstants(0.15, kI = 0.0, kD = 0.01, kF = 1.0 / 6877.7), // TODO: determine
        autoShift = false,
        shiftUpSpeed = 10.0, // TODO: determine
        shiftDownSpeed = 12.0, // TODO: determine
        shiftCooldown = 3.0,

        accelLimit = 1000000.0 // feet/sec^2
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
        println("Setup")

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
        val path = PathUtils.generateLinearPath(arrayOf(Pair(0.0, 0.0), Pair(6.0, 0.0), Pair(8.646, 1.0), Pair(9.464, 2.0), Pair(9.873, 3.0), Pair(10.0, 4.0),Pair(10.0, 15.0)), 400)

        // TODO Play with lookahead as it greatly affects stability of PP algorithm
        val LOOKAHEAD = 1.50 // higher values make smoother, easier-to-follow path but less precise following, measured in FEET
        val SCALING = 1.0 // arbitrary(ish) factor
        val VELOCITY = 3.0 // feet per second overall speed (this would be speed if going perfectly straight)
        val MAXVEL = 2.0 // Absolute maximum velocity the robot can spin the wheels
        val WHEELBASE = 1.96 // feet - distance between wheels - could change as a tuning parameter possibly
        val pp = PurePursuitFollower(path, LOOKAHEAD, SCALING, WHEELBASE, 0.5)

        println("\nEncoder Start Data - left_enc: " + dt.leftEnc.distanceInches + " right_enc: " + dt.rightEnc.distanceInches)

        while (isAutonomous && isEnabled && !pp.finished(Pair(odom.pose.x, odom.pose.y))) {
            odom.update()
            LiveDashboard.putOdom(odom.pose)

            var (l, r) = pp.getControl(Pair(odom.pose.x, odom.pose.y), odom.pose.theta, VELOCITY)

            // Precautionary velocity limit enforcement
            if (l > MAXVEL) {
                //l = MAXVEL
                println("Warning: Velocity Limits Enforced!")
            } else if (l < (MAXVEL * -1.0)) {
                //l = (MAXVEL * -1.0)
                println("Warning: Velocity Limits Enforced!")
            }
            if (r > MAXVEL) {
                //r = MAXVEL
                println("Warning: Velocity Limits Enforced!")
            } else if (r < (MAXVEL * -1.0)) {
                //r = (MAXVEL * -1.0)
                println("Warning: Velocity Limits Enforced!")
            }
            dt.setDriveVelocity(l, r)

            delay(5)
        }
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
