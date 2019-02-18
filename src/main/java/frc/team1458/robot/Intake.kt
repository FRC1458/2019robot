package frc.team1458.robot

import frc.team1458.lib.actuator.Motor
import frc.team1458.lib.actuator.SmartMotor

class Intake(val motor: Motor, val speedFwd: Double, val speedRev: Double, val speedPanic: Double) {
    var speed = 0.0

    fun stop() {
        motor.speed = 0.0
    }

    fun forward() {
        motor.speed = speedFwd
    }

    fun panic() {
        motor.speed = speedPanic
    }

    fun reverse() {
        motor.speed = speedRev
    }
}