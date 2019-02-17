package frc.team1458.robot

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.input.interfaces.Switch

class AutoClimber(val front1: Solenoid, val front2: Solenoid, val rear1: Solenoid, val rear2: Solenoid, val motor: SmartMotor, val sensor: Switch) {
    var frontExtended = false
    var rearExtended = false

    var oneShot = false

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
        } else {
            motor.speed = 0.0
        }

        if(sensor.triggered) {
            front1.retract()
            front2.retract()
            frontExtended = false
        }
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

        println("Them ions be goin")
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