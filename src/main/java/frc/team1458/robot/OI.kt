package frc.team1458.robot

import frc.team1458.lib.input.FlightStick
import frc.team1458.lib.input.Gamepad
import frc.team1458.lib.input.interfaces.Switch

class OI {
    private val leftStick: FlightStick = FlightStick.flightStick(1)
    private val rightStick: FlightStick = FlightStick.flightStick(0)

    // TODO check if inversion/scaleador
    var steerAxis = leftStick.rollAxis.scale(0.5)
    var throttleAxis = rightStick.pitchAxis.inverted

    val forwardButton = rightStick.getButton(3)
    val forwardLineButton = rightStick.getButton(2)
    val reverseButton = rightStick.getButton(4)

    val visionEnableButton = rightStick.trigger.or(leftStick.trigger) // prepares / starts vision
    val visionFollowButton = rightStick.trigger // actually follows drivetrain

    private val controlBoard = Gamepad.xboxController(2) // not really an xbox controller - the new button panel

    // TODO unbork pls
    val intakeForwardButton = leftStick.getButton(11)
    val intakeReverseButton = leftStick.getButton(12)
    val intakePanicButton = leftStick.getButton(13)

    val hatchUpDownSwitch = Switch.toggleSwitch(leftStick.getButton(16)) // make this ONLY toggle switch if you use xbox controller, else not toggle switch
    val hatchGrab = leftStick.getButton(15)
    val hatchRelease = leftStick.getButton(14)

    val climbSwitch = Switch.ALWAYS_OFF // Switch.toggleSwitch(leftStick.getButton(7)) // make this ONLY toggle switch if you use xbox controller, else not toggle switch
}