package frc.team1458.robot

import frc.team1458.lib.actuator.Compressor
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.drive.ClosedLoopTank
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.AnalogPressureSensor
import frc.team1458.lib.sensor.interfaces.DistanceSensor

class RobotMap {

    val drivetrain: ClosedLoopTank = ClosedLoopTank(
        leftMaster = SmartMotor.CANtalonSRX(6),
        rightMaster = SmartMotor.CANtalonSRX(12).inverted,
        leftMotors = arrayOf(SmartMotor.CANtalonSRX(10)),
        rightMotors = arrayOf(SmartMotor.CANtalonSRX(18).inverted),
        closedLoopControl = false,
        wheelDiameter = 0.5,
        closedLoopScaling = 6.0, // TODO: determine

        // TODO Don't forget PID I is disabled for autonomous testing as it introduces errors: maybe 0.0 for I
        pidConstantsLeft = PIDConstants(0.6, kI = 0.001/*3*/, kD = 0.00, kF = 1.0 / 1798.8), // These are decent
        pidConstantsRight = PIDConstants(0.750, kI = 0.001/*5*/, kD = 0.05, kF = 1.0 / 1806.4)// These are decent2
    )

    val compressor = Compressor(PCMcanID = 0)
    val pressureSensor = AnalogPressureSensor(1)

    val intake = Intake(motor = SmartMotor.CANtalonSRX(11), speedFwd = 0.86, speedRev = -0.4, speedPanic = 1.0)

    val hatchIntake = HatchIntake(upDown = Solenoid.doubleSolenoid(PCMcanID = 0, extendChannel = 1, retractChannel = 0),
                                     openClose = Solenoid.doubleSolenoid(PCMcanID = 0, extendChannel = 4, retractChannel = 5))

    private val climberSensor = DistanceSensor.irSensor(channel = 0, m = 1.0, b = 0.0) // m = meters per volt

    val climber = ManualClimber(regulator = Solenoid.doubleSolenoid(PCMcanID = 1, extendChannel = 0, retractChannel = 1),
                                front = Solenoid.doubleSolenoid(PCMcanID = 1, extendChannel = 5, retractChannel = 4),
                                  rear = Solenoid.doubleSolenoid(PCMcanID = 1, extendChannel = 3, retractChannel = 2),
                                  motor = SmartMotor.CANtalonSRX(4).inverted) // arbitrary weird noncoherent value but works
}
