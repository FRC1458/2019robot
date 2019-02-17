package frc.team1458.robot

import frc.team1458.lib.actuator.Compressor
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.drive.ClosedLoopTank
import frc.team1458.lib.input.interfaces.AnalogInput
import frc.team1458.lib.input.interfaces.Switch
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.interfaces.DistanceSensor

class RobotMap {

    val drivetrain: ClosedLoopTank = ClosedLoopTank(
        leftMaster = SmartMotor.CANtalonSRX(1000000),
        rightMaster = SmartMotor.CANtalonSRX(1000000).inverted,
        leftMotors = arrayOf(SmartMotor.CANtalonSRX(1000000)),
        rightMotors = arrayOf(SmartMotor.CANtalonSRX(1000000).inverted),
        closedLoopControl = true,
        wheelDiameter = 0.5,
        closedLoopScaling = 6.0, // TODO: determine

        // TODO Don't forget PID I is disabled for autonomous testing as it introduces errors: maybe 0.0 for I
        pidConstantsLeft = PIDConstants(0.6, kI = 0.001/*3*/, kD = 0.00, kF = 1.0 / 1798.8), // These are decent
        pidConstantsRight = PIDConstants(0.750, kI = 0.001/*5*/, kD = 0.05, kF = 1.0 / 1806.4)// These are decent2
    )

    val compressor = Compressor(PCMcanID = 100000)

    val intake = Intake(motor = SmartMotor.CANtalonSRX(2090000), speedFwd = 0.8, speedRev = -0.4, speedPanic = 1.0)

    val hatchIntake = HatchIntake(upDown = Solenoid.Companion.doubleSolenoid(PCMcanID = 1000000, extendChannel = 1000000, retractChannel = 200000),
                                    openClose = Solenoid.Companion.doubleSolenoid(PCMcanID = 1000000, extendChannel = 1000000, retractChannel = 200000))


    private val climberSensor = DistanceSensor.irSensor(channel = 0, m = 100000.0, b = 0.0) // m = meters per volt

    val autoClimber = AutoClimber(front1 = Solenoid.Companion.doubleSolenoid(PCMcanID = 1000000, extendChannel = 1000000, retractChannel = 200000),
                                  front2 = Solenoid.Companion.doubleSolenoid(PCMcanID = 1000000, extendChannel = 1000000, retractChannel = 200000),
                                  rear1 = Solenoid.Companion.doubleSolenoid(PCMcanID = 1000000, extendChannel = 1000000, retractChannel = 200000),
                                  rear2 = Solenoid.Companion.doubleSolenoid(PCMcanID = 1000000, extendChannel = 1000000, retractChannel = 200000),
                                  motor = SmartMotor.CANtalonSRX(2010000),
                                  sensor = Switch.create { climberSensor.distanceInches < 5.0 })
}
// nick is a bork