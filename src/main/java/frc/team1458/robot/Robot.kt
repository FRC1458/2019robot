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

        pidConstantsLowGearLeft =  PIDConstants(0.5, kI = 0.001, kD = 0.05, kF = 1.0/1798.8), // These are decent
        pidConstantsLowGearRight = PIDConstants(0.5, kI = 0.001, kD = 0.05, kF = 1.0/1806.4), // These are decent2

        shifter = Solenoid.doubleSolenoid(extendChannel = 1, retractChannel = 0)
                + Solenoid.doubleSolenoid(extendChannel = 2, retractChannel = 3),

        pidConstantsHighGearLeft = PIDConstants(0.15, kI = 0.001, kD = 0.01, kF = 1.0/6530.5), // TODO: determine
        pidConstantsHighGearRight = PIDConstants(0.15, kI = 0.001, kD = 0.01, kF = 1.0/6877.7), // TODO: determine
        autoShift = false,
        shiftUpSpeed = 10.0, // TODO: determine
        shiftDownSpeed = 12.0, // TODO: determine
        shiftCooldown = 3.0,

        accelLimit = 1000000.0 // feet/sec^2
    )

    val drivetrainInverted: Boolean = false

    val gyro: AngleSensor = NavX.MXP_I2C().yaw

    // Elevator stuff
    val mag1 = Switch.fromDIO(8).inverted
    val mag2 = Switch.fromDIO(9).inverted
    val elev1 = SmartMotor.CANtalonSRX(20).inverted
    val elev2 = SmartMotor.CANtalonSRX(21).inverted

    // val table = NetworkTableInstance.getDefault().getTable("Live_Dashboard")
    // val odom = EncoderOdom(dt.leftEnc, dt.rightEnc, gyro)

    override fun robotSetup() {
        println("Setup")

        dt.leftMaster.connectedEncoder.zero()
        dt.rightMaster.connectedEncoder.zero()
        gyro.zero()
        // odom.update()
    }

    override fun runAuto() {
        println("Warning: Sandstorm")
        dt.tankDrive(0.4, 0.1)
        delay(400)
        dt.tankDrive(0.4, 0.0)
    }

    override fun teleopInit() {
        // likely nothing here


    }

    override fun teleopPeriodic() {

        /* some logging code - don't mess with this rn
        odom.update()
        SmartDashboard.putNumber("GyroAngle", gyro.heading)

        // should work with falcondash
        table.getEntry("robotX").setDouble(odom.pose.x)
        table.getEntry("robotY").setDouble(odom.pose.y)
        table.getEntry("robotHeading").setDouble(odom.pose.theta)

        */

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

        val speed = if(oi.elevatorUp.triggered && !mag1.triggered) { 0.8 }
        else if(oi.elevatorDown.triggered && !mag2.triggered) { -0.8 }
        else { 0.0 }
        elev1.speed = speed
        elev2.speed = speed

        if(oi.intakeIn.triggered){
            SmartMotor.CANtalonSRX(17).inverted.speed = 1.0
            SmartMotor.CANtalonSRX(19).speed = 1.0
        }
        else if(oi.intakeOut.triggered) {
            SmartMotor.CANtalonSRX(17).inverted.speed = -1.0
            SmartMotor.CANtalonSRX(19).speed = -1.0
        }
        else{
            SmartMotor.CANtalonSRX(17).inverted.speed = 0.0
            SmartMotor.CANtalonSRX(19).speed = 0.0
        }
    }

    override fun runTest() {
        // rewind mechanism, run compressor, etc
    }

    override fun robotDisabled() {}
    override fun disabledPeriodic() {}
}
