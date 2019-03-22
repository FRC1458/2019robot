package frc.team1458.robot

import frc.team1458.lib.input.FlightStick
import frc.team1458.lib.input.Gamepad
import frc.team1458.lib.input.interfaces.POV
import frc.team1458.lib.input.interfaces.Switch

class OI {
    private val leftStick: FlightStick = FlightStick.flightStick(1)
    private val rightStick: FlightStick = FlightStick.flightStick(0)

    var steerAxis = leftStick.rollAxis.scale(0.5)
    var throttleAxis = rightStick.pitchAxis.inverted.scale(0.6)

    val forwardButton = rightStick.getButton(3)
    val forwardLineButton = rightStick.getButton(2)
    val reverseButton = rightStick.getButton(4)

    val defenseButton = leftStick.getButton(2)
    val disableSafetyButton = leftStick.getButton(4)

    val visionEnableButton = rightStick.trigger.or(leftStick.trigger) // prepares / starts vision
    val visionFollowButton = rightStick.trigger // actually follows drivetrain

    private val controlBoard = Gamepad.xboxController(2)

    val intakeForwardButton = controlBoard.getButton(Gamepad.Button.RBUMP)
    val intakeReverseButton = controlBoard.getButton(Gamepad.Button.LBUMP)
    val intakePanicButton = Switch.ALWAYS_OFF // leftStick.getButton(13)

    val hatchUpDownSwitch = Switch.doubleToggle(Switch.fromPOV(controlBoard.getPOV(), POV.Direction.SOUTH),
            Switch.fromPOV(controlBoard.getPOV(), POV.Direction.NORTH)) // TODO - IF THIS ONE IS BACKWARDS, SWAP THE SOUTH AND THE NORTH ----- DAVIS DAY 1

    val hatchGrab = controlBoard.getButton(Gamepad.Button.A)
    val hatchRelease = controlBoard.getButton(Gamepad.Button.B)

    val climb1 = rightStick.getButton(10) // TODO CHANGE THIS TO ONE OF THE BUTTONS ON THE BASE OF THE STICK ----- DAVIS DAY 1
    val climb2 = Switch.fromPOV(controlBoard.getPOV(), POV.Direction.WEST)
    val climb3 = Switch.fromPOV(controlBoard.getPOV(), POV.Direction.EAST)
}
