package frc.team1458.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.core.BaseRobot
import frc.team1458.lib.sensor.PDP
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.flow.systemTimeMillis
import frc.team1458.lib.util.logging.ThreadLogger

class Robot : BaseRobot() {

    private val oi: OI = OI()
    val robot = RobotMap()

    var camera = 0

    private var drivetrainReversed = false
    private val logging = ThreadLogger(5000)


    override fun robotSetup() {
        VisionTable.setup()
        
        VisionTable.ll_mode!!.putNumber(0)
        VisionTable.ll_pipeline!!.putNumber(1)
        VisionTable.ll_stream!!.putNumber(1)

        // Logging
        SmartDashboard.putNumber("Pressure (psi)", robot.pressureSensor.pressure)
        SmartDashboard.putData("PDP", PDP.pdp)

        SmartDashboard.putNumber("kP", 0.55)
        SmartDashboard.putNumber("kD", -0.3)

        val LIMIT_MAX = 20
        val LIMIT_CONT = LIMIT_MAX
        val TIME = 100

        robot.drivetrain.leftMaster._talonInstance!!.configContinuousCurrentLimit(LIMIT_CONT, 0)
        robot.drivetrain.leftMaster._talonInstance!!.configPeakCurrentLimit(LIMIT_MAX, 0)
        robot.drivetrain.leftMaster._talonInstance!!.configPeakCurrentDuration(TIME, 0)
        robot.drivetrain.leftMaster._talonInstance!!.enableCurrentLimit(true)

        robot.drivetrain.rightMaster._talonInstance!!.configContinuousCurrentLimit(LIMIT_CONT, 0)
        robot.drivetrain.rightMaster._talonInstance!!.configPeakCurrentLimit(LIMIT_MAX, 0)
        robot.drivetrain.rightMaster._talonInstance!!.configPeakCurrentDuration(TIME, 0)
        robot.drivetrain.rightMaster._talonInstance!!.enableCurrentLimit(true)

        robot.drivetrain.leftMotors[0]._talonInstance!!.configContinuousCurrentLimit(LIMIT_CONT, 0)
        robot.drivetrain.leftMotors[0]._talonInstance!!.configPeakCurrentLimit(LIMIT_MAX, 0)
        robot.drivetrain.leftMotors[0]._talonInstance!!.configPeakCurrentDuration(TIME, 0)
        robot.drivetrain.leftMotors[0]._talonInstance!!.enableCurrentLimit(true)

        robot.drivetrain.rightMotors[0]._talonInstance!!.configContinuousCurrentLimit(LIMIT_CONT, 0)
        robot.drivetrain.rightMotors[0]._talonInstance!!.configPeakCurrentLimit(LIMIT_MAX, 0)
        robot.drivetrain.rightMotors[0]._talonInstance!!.configPeakCurrentDuration(TIME, 0)
        robot.drivetrain.rightMotors[0]._talonInstance!!.enableCurrentLimit(true)

        try {
            logging.setup(logDirectory = "/home/lvuser/logs/", keys = arrayOf("psi"))
            logging.start()
        }
        catch (e: Exception) {
            println("BIG BORK LOGGING BORKED!")
            e.printStackTrace()
        }

    }

    override fun runAuto() {
        println("Auto Enabled")

        robot.hatchIntake.grab()
        robot.hatchIntake.down()

        teleopInit()

        while (super.isAutonomous() && super.isEnabled()) {
            teleopPeriodic()
            delay(1)
        }

    }

    override fun teleopInit() {
        println("Teleoperated Enabled")

        robot.compressor.start()
    }

    var last : Double = 0.0
    var lastTime : Double = 0.0
    var climbStarted : Boolean = false

    override fun teleopPeriodic() {

        // check if need to reverse DT
            drivetrainReversed = false
            camera = 0
            VisionTable.ll_stream!!.putNumber(1)
            println(camera)
        } else if (oi.forwardLineButton.triggered) { // front downward-facing
            drivetrainReversed = false
            camera = 1
            VisionTable.ll_stream!!.putNumber(1)
            println(camera)
        } else if (oi.reverseButton.triggered) { // rear (cargo target)
            drivetrainReversed = true
            camera = 2
            VisionTable.ll_stream!!.putNumber(2)
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
            robot.hatchIntake.down()
        } else {
            robot.hatchIntake.up()
        }

        if (oi.hatchGrab.triggered) {
            robot.hatchIntake.grab()
        } else if (oi.hatchRelease.triggered) {
            robot.hatchIntake.release()
        }

        // used for "yeeting" the robot to the second level HAB
        if (oi.disableSafetyButton.triggered) {
            robot.drivetrain.disableCurrentLimit()
        } else {
            robot.drivetrain.enableCurrentLimit()
        }

        if (oi.pushingButton.triggered) {
            robot.drivetrain.enableRamp()
        } else {
            robot.drivetrain.disableRamp()
        }

        // climby climby
        if (oi.climb1.triggered) {
            println("climb 1")
            robot.climber.raise()
        }
        if(oi.climb2.triggered) {
            println("climb 2")
            robot.climber.frontDown()
        }
        if(oi.climb3.triggered) {
            println("climb 3")
            robot.climber.rearDown()
        }

        robot.climber.update(oi.throttleAxis.value)


        // switch the pipeline big bork in the cloud
        if(oi.pipeLoadingStation.triggered) {
            VisionTable.ll_mode!!.putNumber(0)
            VisionTable.ll_pipeline!!.putNumber(0)
        }

        if(oi.pipeLeft.triggered) {
            VisionTable.ll_mode!!.putNumber(0)
            VisionTable.ll_pipeline!!.putNumber(1)
        }

        if(oi.pipeRight.triggered) {
            VisionTable.ll_mode!!.putNumber(0)
            VisionTable.ll_pipeline!!.putNumber(2)
        }

        if(oi.pipeNone.triggered) {
            VisionTable.ll_mode!!.putNumber(1)
        }


        if (oi.visionFollowButton.triggered && (VisionTable.ll_tv!!.getNumber(0) == 1)) {

            val kP = SmartDashboard.getNumber("kP", 0.55)
            val kD = SmartDashboard.getNumber("kD", -0.3)

            // TODO, switch to "tx" if using limelight (divide by 29.8) only use "tx" if "tv" = 1, else there is no targets

            val offset = VisionTable.ll_tx!!.getDouble(0.0) / 29.8

            val steer = kP * offset - (kD * (last - offset) / (0.001 * (lastTime - systemTimeMillis)))

            last = offset
            lastTime = systemTimeMillis

            val speed = 0.75 * (if (drivetrainReversed) {
                -oi.throttleAxis.value
            } else {
                oi.throttleAxis.value
            })

            robot.drivetrain.arcadeDrive(speed, ((0.8*speed)+0.2) * steer)

        } else if (climbStarted) { // climbing driving
            robot.drivetrain.arcadeDrive(oi.throttleAxis.value * 0.15, 0.0)
        } else { // normal driving
            robot.drivetrain.arcadeDrive(
                if (drivetrainReversed) {
                    -oi.throttleAxis.value
                } else {
                    if(oi.disableSafetyButton.triggered) { 1.6 * oi.throttleAxis.value } else { oi.throttleAxis.value }
                },
                oi.steerAxis.value
            )
        }

        VisionTable.pressure!!.setDouble(robot.pressureSensor.pressure)

        // Logging
        SmartDashboard.putNumber("Pressure (psi)", robot.pressureSensor.pressure)
        SmartDashboard.putData("PDP", PDP.pdp)


    }

    override fun runTest() {
        robot.climber.resetClimb()
        robot.compressor.start()
        
    }

    override fun robotDisabled() {
        robot.drivetrain.setDriveVelocity(0.0, 0.0)
        println("Robot Disabled")

        // Logging
        SmartDashboard.putNumber("Pressure (psi)", robot.pressureSensor.pressure)
        SmartDashboard.putData("PDP", PDP.pdp)


    }

    override fun disabledPeriodic() {
        // Communism

        // Logging
        SmartDashboard.putNumber("Pressure (psi)", robot.pressureSensor.pressure)
        SmartDashboard.putData("PDP", PDP.pdp)


    }
}

