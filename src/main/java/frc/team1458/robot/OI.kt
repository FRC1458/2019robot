package frc.team1458.robot

import frc.team1458.lib.input.FlightStick
import frc.team1458.lib.input.Gamepad
import frc.team1458.lib.input.interfaces.Switch

class OI {
    val leftStick: FlightStick = FlightStick.flightStick(1)
    val rightStick: FlightStick = FlightStick.flightStick(0)

    // TODO check if inversion/scaleador
    var steerAxis = leftStick.rollAxis.scale(0.5)
    var throttleAxis = rightStick.pitchAxis.inverted

    val forwardButton = rightStick.getButton(1)
    val forwardLineButton = rightStick.getButton(2)
    val reverseButton = rightStick.getButton(3)

    val visionFollowButton = rightStick.trigger

    val controlBoard = Gamepad.xboxController(2) // not really an xbox controller

    val intakeForwardButton = controlBoard.getButton(1000)
    val intakeReverseButton = controlBoard.getButton(1000)
    val intakePanicButton = controlBoard.getButton(1000)

    val hatchUpDownSwitch = Switch.toggleSwitch(controlBoard.getButton(1000)) // make this ONLY toggle switch if you use xbox controller, else not toggle switch
    val hatchGrab = controlBoard.getButton(1000)
    val hatchRelease = controlBoard.getButton(1000)

    val climbSwitch = controlBoard.getButton(1000)

}