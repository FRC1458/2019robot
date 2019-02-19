package frc.team1458.robot

import frc.team1458.lib.actuator.Solenoid

class HatchIntake(val upDown: Solenoid, val openClose: Solenoid) {

    fun up() {
        upDown.extend()
    }

    fun down() {
        upDown.retract()
    }

    fun grab() {
        openClose.extend()
        println("Intake Gripping...")
    }

    fun release() {
        openClose.retract()
        println("Intake Releasing...")
    }
}
