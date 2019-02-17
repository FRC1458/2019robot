package frc.team1458.robot

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.sensor.*
import frc.team1458.lib.sensor.interfaces.*
import frc.team1458.lib.core.BaseRobot
import frc.team1458.lib.input.interfaces.Switch
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.odom.EncoderOdom
import frc.team1458.lib.drive.ClosedLoopTank
import frc.team1458.lib.pathfinding.*
import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.TelemetryLogger
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.maths.TurtleMaths
import java.io.PrintWriter
import kotlin.math.min
import kotlin.math.sqrt

class Robot : BaseRobot() {

    private val oi: OI = OI()
    val robot = RobotMap()

    private var drivetrainReversed = false


    override fun robotSetup() {
        robot.drivetrain.leftMaster.connectedEncoder.zero()
        robot.drivetrain.rightMaster.connectedEncoder.zero()

        VisionTable.setup()
        TelemetryLogger.setup(
            arrayOf(
                "leftEncoderDistance",
                "rightEncoderDistance"
            )
        )
    }

    override fun runAuto() {
        println("Warning: Sandstorm")

        teleopInit()
        while (super.isAutonomous() && super.isEnabled()) {
            teleopPeriodic()
            delay(1)
        }

    }
    override fun teleopInit() {
        println("let's think about what these ions are doing")

        robot.compressor.start()
    }

    override fun teleopPeriodic() {
        TelemetryLogger.startIteration()

        // check if need to reverse DT
        var camera = -1

        if(oi.forwardButton.triggered) {
            drivetrainReversed = false
            camera = 0
        } else if(oi.forwardLineButton.triggered) {
            drivetrainReversed = false
            camera = 1
        } else if(oi.reverseButton.triggered) {
            drivetrainReversed = true
            camera = 2
        }


        // Ball intake
        when {
            oi.intakeForwardButton.triggered -> robot.intake.forward()
            oi.intakeReverseButton.triggered -> robot.intake.reverse()
            oi.intakePanicButton.triggered -> robot.intake.panic()
            else -> robot.intake.update()
        }


        // hatch intake
        if(oi.hatchUpDownSwitch.triggered) {
            robot.hatchIntake.up()
        } else {
            robot.hatchIntake.down()
        }

        if(oi.hatchGrab.triggered) {
            robot.hatchIntake.grab()
        } else if(oi.hatchRelease.triggered) {
            robot.hatchIntake.release()
        }


        // climby climby
        if(oi.climbSwitch.triggered) {
            robot.autoClimber.extend()
        } else {
            robot.autoClimber.retract()
        }

        robot.autoClimber.update(oi.throttleAxis.value)


        // change camera only if camera changed
        if(camera != -1) {
            VisionTable.camera!!.setDouble(camera.toDouble())
        }

        // TODO unbork vision stuff
        robot.drivetrain.arcadeDrive(if(drivetrainReversed)
                                        { -oi.throttleAxis.value }
                                    else
                                        { oi.throttleAxis.value },
                                    oi.steerAxis.value)

        TelemetryLogger.putValue("leftEncoderDistance", robot.drivetrain.leftMaster.connectedEncoder.angle)
        TelemetryLogger.putValue("rightEncoderDistance", robot.drivetrain.rightMaster.connectedEncoder.angle)

        TelemetryLogger.endIteration()
    }

    override fun runTest() {

    }

    override fun robotDisabled() {
        robot.drivetrain.setDriveVelocity(0.0, 0.0)
        println("Disabled")
    }

    override fun disabledPeriodic() {
        // communism
    }
}

