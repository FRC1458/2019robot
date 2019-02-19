package frc.team1458.robot

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.input.interfaces.Switch
import frc.team1458.lib.util.flow.delay

class ManualClimber(val regulator: Solenoid, val front: Solenoid, val rear: Solenoid, val motor: SmartMotor) {

    var rearExtended = false

    init {
        front.retract()
        rear.retract()
        delay(250)
        regulator.extend()

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
    }

    fun raise() {
        regulator.retract()
        delay(500)
        front.extend()
        rear.extend()
        delay(500)
        regulator.extend()
        rearExtended = true
    }

    fun frontDown() {
        front.retract()
    }

    fun rearDown() {
        rear.retract()
        rearExtended = false
    }
}