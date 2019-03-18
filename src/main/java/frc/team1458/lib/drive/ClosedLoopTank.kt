package frc.team1458.lib.drive

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.interfaces.AngleSensor
import frc.team1458.lib.sensor.interfaces.DistanceSensor


class ClosedLoopTank(
                val leftMaster: SmartMotor,
                val rightMaster: SmartMotor,
                val leftMotors: Array<SmartMotor> = arrayOf(),
                val rightMotors: Array<SmartMotor> = arrayOf(),

                // Closed Loop Control
                val closedLoopControl: Boolean = false,
                wheelDiameter: Double? = null,
                val closedLoopScaling: Double? = null,
                val pidConstantsLeft: PIDConstants? = null,
                val pidConstantsRight: PIDConstants? = null,
                val gyro: AngleSensor? = null,

                // Current Limiting
                private var currentLimitingEnabled: Boolean = false,
                private val currentLimitMax: Int = 20,
                private val currentLimitContinuous: Int = currentLimitMax,
                private val currentLimitTimeMs: Int = 100) {

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

        // Current Limiting
        if (currentLimitingEnabled) {
            leftMaster._talonInstance!!.configContinuousCurrentLimit(currentLimitContinuous, 0)
            leftMaster._talonInstance!!.configPeakCurrentLimit(currentLimitMax, 0)
            leftMaster._talonInstance!!.configPeakCurrentDuration(currentLimitTimeMs, 0)
            leftMaster._talonInstance!!.enableCurrentLimit(true)

            rightMaster._talonInstance!!.configContinuousCurrentLimit(currentLimitContinuous, 0)
            rightMaster._talonInstance!!.configPeakCurrentLimit(currentLimitMax, 0)
            rightMaster._talonInstance!!.configPeakCurrentDuration(currentLimitTimeMs, 0)
            rightMaster._talonInstance!!.enableCurrentLimit(true)

            for (motor: SmartMotor in leftMotors) {
                motor._talonInstance!!.configContinuousCurrentLimit(currentLimitContinuous, 0)
                motor._talonInstance!!.configPeakCurrentLimit(currentLimitMax, 0)
                motor._talonInstance!!.configPeakCurrentDuration(currentLimitTimeMs, 0)
                motor._talonInstance!!.enableCurrentLimit(true)
            }

            for (motor: SmartMotor in rightMotors) {
                motor._talonInstance!!.configContinuousCurrentLimit(currentLimitContinuous, 0)
                motor._talonInstance!!.configPeakCurrentLimit(currentLimitMax, 0)
                motor._talonInstance!!.configPeakCurrentDuration(currentLimitTimeMs, 0)
                motor._talonInstance!!.enableCurrentLimit(true)
            }
        }

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
