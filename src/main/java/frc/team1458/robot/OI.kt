package frc.team1458.robot

import frc.team1458.lib.input.FlightStick
import frc.team1458.lib.input.Gamepad
import frc.team1458.lib.input.interfaces.POV
import frc.team1458.lib.input.interfaces.Switch

class OI {
    private val leftStick: FlightStick = FlightStick.flightStick(1)
    private val rightStick: FlightStick = FlightStick.flightStick(0)

    var steerAxis = leftStick.rollAxis.scale(0.35)
    var throttleAxis = rightStick.pitchAxis.inverted.scale(0.6)

    val forwardButton = rightStick.getButton(3)
    val forwardLineButton = Switch.ALWAYS_OFF // rightStick.getButton(2)
    val reverseButton = rightStick.getButton(4)

    val defenseButton = Switch.ALWAYS_OFF

    val pushingButton = leftStick.getButton(2) // limit acceleration but no torque limit

    val disableSafetyButton = leftStick.getButton(4).or(pushingButton) // no speed and no torque limit

    val visionEnableButton = rightStick.trigger.or(leftStick.trigger) // prepares / starts vision
    val visionFollowButton = rightStick.trigger // actually follows drivetrain

    private val controlBoard = Gamepad.xboxController(2)

    val intakeForwardButton = controlBoard.getButton(Gamepad.Button.RBUMP)
    val intakeReverseButton = controlBoard.getButton(Gamepad.Button.LBUMP)
    val intakePanicButton = Switch.ALWAYS_OFF

    val hatchUpDownSwitch = Switch.doubleToggle(Switch.fromPOV(controlBoard.getPOV(), POV.Direction.SOUTH),
            Switch.fromPOV(controlBoard.getPOV(), POV.Direction.NORTH))

    val hatchGrab = controlBoard.getButton(Gamepad.Button.A)
    val hatchRelease = controlBoard.getButton(Gamepad.Button.B)

    val climb1 = rightStick.getButton(10) 
    val climb2 = Switch.fromPOV(controlBoard.getPOV(), POV.Direction.WEST)
    val climb3 = Switch.fromPOV(controlBoard.getPOV(), POV.Direction.EAST)
}
