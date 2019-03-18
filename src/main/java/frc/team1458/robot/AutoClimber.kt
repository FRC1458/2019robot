package frc.team1458.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.input.interfaces.Switch
import frc.team1458.lib.util.flow.systemTimeMillis

class AutoClimber(val front: Solenoid, val rear: Solenoid, val motor: SmartMotor, val sensor: Switch) {
    var frontExtended = false
    var rearExtended = false

    var oneShot = false

    var lastClimb = 0.0

    init {
        front.retract()
        rear.retract()

        // TODO uncomment this if you see smoke coming out of the bot
        /*motor._talonInstance!!.configContinuousCurrentLimit(30, 0)
        motor._talonInstance!!.configPeakCurrentLimit(50, 0)
        motor._talonInstance!!.configPeakCurrentDuration(300, 0)
        motor._talonInstance!!.enableCurrentLimit(true)*/
    }

    fun update(speed: Double) {
        if(rearExtended) {
            motor.speed = speed * 0.50

            if(sensor.triggered && (systemTimeMillis - lastClimb) > 1500) {
                front.retract()
                frontExtended = false
            }
        } else {
            motor.speed = 0.0
        }

        SmartDashboard.putNumber("front",
            if(front.state == Solenoid.State.EXTENDING) { 1.0 } else if(front.state == Solenoid.State.RETRACTING) { -1.0 } else { 0.0} )

        SmartDashboard.putNumber("rear",
            if(rear.state == Solenoid.State.EXTENDING) { 1.0 } else if(rear.state == Solenoid.State.RETRACTING) { -1.0 } else { 0.0} )
    }

    fun extend() {
        if(oneShot) {
            return
        }

        frontExtended = true
        rearExtended = true
        oneShot = true

        front.extend()
        rear.extend()

        lastClimb = systemTimeMillis

        println("Climber Extending...")
    }

    fun retract() {
        front.retract()
        rear.retract()

        frontExtended = false
        rearExtended = false
    }

    // DO NOT CALL THIS
    fun reset() {
        oneShot = false
    }
}