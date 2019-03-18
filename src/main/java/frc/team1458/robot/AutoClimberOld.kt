package frc.team1458.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.input.interfaces.Switch
import frc.team1458.lib.util.flow.systemTimeMillis

class AutoClimberOld(val front1: Solenoid, val front2: Solenoid, val rear1: Solenoid, val rear2: Solenoid, val motor: SmartMotor, val sensor: Switch) {
    var frontExtended = false
    var rearExtended = false

    var oneShot = false

    var lastClimb = 0.0

    init {
        front1.retract()
        front2.retract()
        rear1.retract()
        rear2.retract()

        // TODO uncomment this if you see smoke coming out of the bot
        /*motor._talonInstance!!.configContinuousCurrentLimit(30, 0)
        motor._talonInstance!!.configPeakCurrentLimit(50, 0)
        motor._talonInstance!!.configPeakCurrentDuration(300, 0)
        motor._talonInstance!!.enableCurrentLimit(true)*/
    }

    fun update(speed: Double) {
        if(rearExtended) {
            motor.speed = speed

            if(sensor.triggered && (systemTimeMillis - lastClimb) > 1500) {
                front1.retract()
                front2.retract()
                frontExtended = false
            }
        } else {
            motor.speed = 0.0
        }

        SmartDashboard.putNumber("front1",
            if(front1.state == Solenoid.State.EXTENDING) { 1.0 } else if(front1.state == Solenoid.State.RETRACTING) { -1.0 } else { 0.0} )

        SmartDashboard.putNumber("front2",
            if(front2.state == Solenoid.State.EXTENDING) { 1.0 } else if(front2.state == Solenoid.State.RETRACTING) { -1.0 } else { 0.0} )

        SmartDashboard.putNumber("rear1",
            if(rear1.state == Solenoid.State.EXTENDING) { 1.0 } else if(rear1.state == Solenoid.State.RETRACTING) { -1.0 } else { 0.0} )

        SmartDashboard.putNumber("rear2",
            if(rear2.state == Solenoid.State.EXTENDING) { 1.0 } else if(rear2.state == Solenoid.State.RETRACTING) { -1.0 } else { 0.0} )
    }

    fun extend() {
        if(oneShot) {
            return
        }

        frontExtended = true
        rearExtended = true
        oneShot = true

        front1.extend()
        front2.extend()
        rear1.extend()
        rear2.extend()

        lastClimb = systemTimeMillis

        println("Climber Extending...")
    }

    fun retract() {
        front1.retract()
        front2.retract()
        rear1.retract()
        rear2.retract()

        frontExtended = false
        rearExtended = false
    }

    // DO NOT CALL THIS
    fun reset() {
        oneShot = false
    }
}