package frc.team1458.robot

import frc.team1458.lib.actuator.Motor
import frc.team1458.lib.actuator.SmartMotor

class Intake(val motor: Motor, val speedFwd: Double, val speedRev: Double, val speedPanic: Double) {
    var speed = 0.0

    fun update() {
        motor.speed = speed
    }

    fun forward() {
        speed = speedFwd
        update() // dont remove this
    }

    fun panic() {
        speed = speedPanic
        update() // dont remove this
    }

    fun reverse() {
        speed = speedRev
        update() // dont remove this
    }
}