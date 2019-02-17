package frc.team1458.robot

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.input.interfaces.Switch

class ManualClimber(val front1: Solenoid, val front2: Solenoid, val rear1: Solenoid, val rear2: Solenoid, val motor: SmartMotor) {

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

    fun update(speed: Double, front: Boolean, rear: Boolean) {
        if(rear) {
            motor.speed = speed
        } else {
            motor.speed = 0.0
        }

        if(rear && front) {
            front1.extend()
            front2.extend()
        } else {
            front1.retract()
            front2.retract()
        }

        if(rear) {
            rear1.extend()
            rear2.extend()
        } else {
            rear1.retract()
            rear2.retract()
        }
    }
}