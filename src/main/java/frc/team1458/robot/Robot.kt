package frc.team1458.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.core.BaseRobot
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.logging.ThreadLogger

class Robot : BaseRobot() {

    private val oi: OI = OI()
    val robot = RobotMap()

    var camera = 0

    private var drivetrainReversed = false

    private val logging = ThreadLogger(5000)


    override fun robotSetup() {
        // robot.drivetrain.leftMaster.connectedEncoder.zero()
        // robot.drivetrain.rightMaster.connectedEncoder.zero()

        VisionTable.setup()
        logging.setup(logDirectory = "/tmp/logs/", keys = arrayOf("psi"))
        logging.start()

        /*TelemetryLogger.setup(
            arrayOf(
                "leftEncoderDistance",
                "rightEncoderDistance"
            )
        )*/
    }

    override fun runAuto() {
        println("My battery is low and it's getting dark")


        // 12 - right, inverted
        // 18 - right, inverted
        // 6  - left/forward
        // 10 - left/forward

        /*var m = SmartMotor.CANtalonSRX(12)
        println("running 12")
        m.speed = 0.5
        delay(5000)
        m.speed = 0.0

        m = SmartMotor.CANtalonSRX(18)
        println("running 18")
        m.speed = 0.5
        delay(5000)
        m.speed = 0.0

        m = SmartMotor.CANtalonSRX(6)
        println("running 6")
        m.speed = 0.5
        delay(5000)
        m.speed = 0.0

        m = SmartMotor.CANtalonSRX(10)
        println("running 10")
        m.speed = 0.5
        delay(5000)
        m.speed = 0.0

        return*/


        teleopInit()

        while (super.isAutonomous() && super.isEnabled()) {
            teleopPeriodic()
            delay(1)
        }

    }

    override fun teleopInit() {
        println("Let's think about what these ions are doing")

        robot.compressor.start()
    }

    override fun teleopPeriodic() {
        // check if need to reverse DT

        if (oi.forwardButton.triggered) { // front vision camera
            drivetrainReversed = false
            camera = 0
            VisionTable.camera!!.setDouble(camera.toDouble())
            println(camera)
        } else if (oi.forwardLineButton.triggered) { // front downward-facing
            drivetrainReversed = false
            camera = 1
            VisionTable.camera!!.setDouble(camera.toDouble())
            println(camera)
        } else if (oi.reverseButton.triggered) { // rear (cargo target)
            drivetrainReversed = true
            camera = 2
            VisionTable.camera!!.setDouble(camera.toDouble())
            println(camera)
        }


        // Ball intake
        when {
            oi.intakeForwardButton.triggered -> robot.intake.forward()
            oi.intakeReverseButton.triggered -> robot.intake.reverse()
            oi.intakePanicButton.triggered -> robot.intake.panic()
            else -> robot.intake.stop()
        }


        // hatch intake
        if (oi.hatchUpDownSwitch.triggered) {
            robot.hatchIntake.up()
        } else {
            robot.hatchIntake.down()
        }

        if (oi.hatchGrab.triggered) {
            robot.hatchIntake.grab()
        } else if (oi.hatchRelease.triggered) {
            robot.hatchIntake.release()
        }


        // climby climby
        if (oi.climbSwitch.triggered) {
            robot.autoClimber.extend()
        } else {
            robot.autoClimber.retract()
        }

        robot.autoClimber.update(oi.throttleAxis.value)

        if (oi.visionEnableButton.triggered) {
            VisionTable.visionReady!!.setBoolean(false)
            VisionTable.visionEnable!!.setBoolean(true)
        } else {
            VisionTable.visionReady!!.setBoolean(false)
        }

        if (oi.visionFollowButton.triggered && (VisionTable.visionReady!!.getBoolean(false) == true)) {
            val (kP, kD) = arrayOf(
                Pair(1.2, 0.0), // front vision camera
                Pair(1.2, 0.3), // front downward-facing
                Pair(1.2, 0.0) // rear (cargo target)
            )[camera]

            val steer = kP * VisionTable.horizOffset!!.getDouble(0.0) +
                    kD * VisionTable.angleOffset!!.getDouble(0.0)

            val speed = 0.75 * (if (drivetrainReversed) {
                -oi.throttleAxis.value
            } else {
                oi.throttleAxis.value
            })

            robot.drivetrain.arcadeDrive(speed, steer)

        } else if (oi.climbSwitch.triggered) { // climbing driving
            robot.drivetrain.arcadeDrive(oi.throttleAxis.value * 0.25, 0.0)
        } else { // normal driving
            robot.drivetrain.arcadeDrive(
                if (drivetrainReversed) {
                    -oi.throttleAxis.value
                } else {
                    oi.throttleAxis.value
                },
                oi.steerAxis.value
            )
        }

        // Logging
        SmartDashboard.putNumber("pressure", robot.pressureSensor.pressure)
        logging.update("psi", robot.pressureSensor.pressure)
    }

    override fun runTest() {
        logging.update("psi", robot.pressureSensor.pressure)
    }

    override fun robotDisabled() {
        robot.drivetrain.setDriveVelocity(0.0, 0.0)

        println("Disabled")
    }

    override fun disabledPeriodic() {
        // Communism
        SmartDashboard.putNumber("pressureeeeee", robot.pressureSensor.pressure)
        logging.update("psi", robot.pressureSensor.pressure)
    }
}

