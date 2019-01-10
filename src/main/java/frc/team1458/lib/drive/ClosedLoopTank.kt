package frc.team1458.lib.drive

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.drive.util.AutoshiftHelper
import frc.team1458.lib.drive.util.CheesyDriveHelper
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.interfaces.AngleSensor
import frc.team1458.lib.sensor.interfaces.DistanceSensor
import frc.team1458.lib.util.flow.systemTimeMillis
import frc.team1458.lib.util.flow.systemTimeSeconds
import java.lang.Math.abs
import kotlin.math.abs


class ClosedLoopTank(
                val leftMaster: SmartMotor,
                val rightMaster: SmartMotor,
                val leftMotors: Array<SmartMotor> = arrayOf(),
                val rightMotors: Array<SmartMotor> = arrayOf(),

                // Closed Loop Control
                var closedLoopControl: Boolean = false,
                wheelDiameter: Double? = null,
                val closedLoopScaling: Double? = null,
                val pidConstantsLeft: PIDConstants? = null,
                val pidConstantsRight: PIDConstants? = null,

                var gyro: AngleSensor? = null) {

    val wheelCircumference = wheelDiameter?.times(Math.PI)

    var leftTarget = 0.0
    var rightTarget = 0.0

    /**
     * Returns true if closed loop mode is ready or false if open-loop should be used as a fallback
     */
    val closedLoopReady : Boolean
        get() = closedLoopControl &&
                (wheelCircumference != null) &&
                (closedLoopScaling != null) &&
                (pidConstantsLeft != null) && (pidConstantsRight != null) &&
                leftMaster.isEncoderWorking &&
                rightMaster.isEncoderWorking

    val leftEnc : DistanceSensor = object : DistanceSensor {
        override val distanceMeters: Double
            get() = leftMaster.connectedEncoder.angle * (wheelCircumference ?: 0.0) * 0.3048 / 360.0

        override val velocity: Double
            get() = leftMaster.connectedEncoder.rate * (wheelCircumference ?: 0.0) * 0.3048 / 360.0

        override fun zero() {
            leftMaster.connectedEncoder.zero()
        }
    }

    val rightEnc : DistanceSensor = object : DistanceSensor {
        override val distanceMeters: Double
            get() = rightMaster.connectedEncoder.angle * (wheelCircumference ?: 0.0) * 0.3048 / 360.0

        override val velocity: Double
            get() = rightMaster.connectedEncoder.rate * (wheelCircumference ?: 0.0) * 0.3048 / 360.0

        override fun zero() {
            rightMaster.connectedEncoder.zero()
        }
    }

    init {
        leftMaster.brakeMode = SmartMotor.BrakeMode.BRAKE
        rightMaster.brakeMode = SmartMotor.BrakeMode.BRAKE

        // Set both motors to drive
        for(motor in leftMotors) {
            motor.follow(leftMaster)
            motor.brakeMode = SmartMotor.BrakeMode.BRAKE
        }

        for(motor in rightMotors) {
            motor.follow(rightMaster)
            motor.brakeMode = SmartMotor.BrakeMode.BRAKE
        }

        leftMaster.PIDconstants = pidConstantsLeft ?: PIDConstants.DISABLE
        rightMaster.PIDconstants = pidConstantsRight ?: PIDConstants.DISABLE
    }

    fun setDriveVelocity(left: Double, right: Double) {
        if(wheelCircumference != null) {
            leftTarget = left * (360.0 / wheelCircumference)
            rightTarget = right * (360.0 / wheelCircumference)
        } else {
            leftTarget = left
            rightTarget = right
        }

        leftMaster.PIDsetpoint = leftTarget
        rightMaster.PIDsetpoint = rightTarget
    }

    fun setRawDrive(left: Double, right: Double) {
        leftTarget = left
        rightTarget = right

        leftMaster.speed = leftTarget
        rightMaster.speed = rightTarget
    }

    fun tankDrive(left: Double, right: Double) {
        if(closedLoopControl && closedLoopReady && closedLoopScaling != null) {
            setDriveVelocity(left * closedLoopScaling, right * closedLoopScaling)
        } else {
            setRawDrive(left, right)
        }
    }

    fun arcadeDrive(forward: Double, turn: Double) {
        if(closedLoopControl && closedLoopReady && closedLoopScaling != null) {
            setDriveVelocity((forward + turn) * closedLoopScaling, (forward - turn) * closedLoopScaling)
        } else {
            setRawDrive(forward + turn, forward - turn)
        }
    }

}
