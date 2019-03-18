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

        // Logging
        SmartDashboard.putNumber("Pressure (psi)", robot.pressureSensor.pressure)
        SmartDashboard.putData("PDP", PDP.pdp)

        SmartDashboard.putNumber("kP", 0.35)
        SmartDashboard.putNumber("kD", -0.2)

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



        if (oi.visionEnableButton.triggered) {
            VisionTable.visionEnable!!.setBoolean(true)
        } else {
            VisionTable.visionReady!!.setBoolean(false)
            VisionTable.visionEnable!!.setBoolean(false)
        }

        if (oi.visionFollowButton.triggered && (VisionTable.visionReady!!.getBoolean(false) == true)) {

            val x = SmartDashboard.getNumber("kP", 0.35)
            val y = SmartDashboard.getNumber("kD", -0.2)

            val (kP, kD) = arrayOf(
                Pair(x, y), // front vision camera
                Pair(x, y), // front downward-facing
                Pair(x, y) // rear (cargo target)
            )[camera]

            val offset = VisionTable.horizOffset!!.getDouble(0.0)

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
                    oi.throttleAxis.value
                },
                oi.steerAxis.value
            )
        }

        // Logging
        SmartDashboard.putNumber("Pressure (psi)", robot.pressureSensor.pressure)
        SmartDashboard.putData("PDP", PDP.pdp)

        try {
            logging.update("psi", robot.pressureSensor.pressure.toString())
        }
        catch (e: Exception) {
            println("BIG BORK LOGGING BORKED!")
            e.printStackTrace()
        }
    }

    override fun runTest() {
        // logging.update("psi", robot.pressureSensor.pressure)

        while (this.isEnabled) {
            /*if (oi.controlBoard.getButton(5).triggered) {
                println("front extend")
                robot.autoClimber.front.extend()
            } else {
                println("front retract")
                robot.autoClimber.front.retract()
            }

            if (oi.controlBoard.getButton(6).triggered) {
                println("rear extend")
                robot.autoClimber.rear.extend()
            } else {
                println("rear retract")
                robot.autoClimber.rear.retract()
            }*/

            /*if (oi.controlBoard.getButton(1).triggered) { // front1
                if (oi.controlBoard.getButton(5).triggered) {
                    println("front1 extending")
                    robot.autoClimber.front1.extend()
                } else if (oi.controlBoard.getButton(6).triggered) {
                    println("rear2 retracting")
                    robot.autoClimber.front1.retract()
                }

            }

            if (oi.controlBoard.getButton(2).triggered) { // front2
                if (oi.controlBoard.getButton(5).triggered) {
                    println("front2 extending")
                    robot.autoClimber.front2.extend()
                } else if (oi.controlBoard.getButton(6).triggered) {
                    println("rear2 retracting")
                    robot.autoClimber.front2.retract()
                }
            }

            if (oi.controlBoard.getButton(3).triggered) { // rear1
                if (oi.controlBoard.getButton(5).triggered) {
                    println("rear1 extending")
                    robot.autoClimber.rear1.extend()
                } else if (oi.controlBoard.getButton(6).triggered) {
                    println("rear2 retracting")
                    robot.autoClimber.rear1.retract()
                }
            }

            if (oi.controlBoard.getButton(4).triggered) { // rear2
                if (oi.controlBoard.getButton(5).triggered) {
                    println("rear2 extending")
                    robot.autoClimber.rear2.extend()
                } else if (oi.controlBoard.getButton(6).triggered) {
                    println("rear2 retracting")
                    robot.autoClimber.rear2.retract()
                }
            }*/

            // Logging
            SmartDashboard.putNumber("Pressure (psi)", robot.pressureSensor.pressure)
            SmartDashboard.putData("PDP", PDP.pdp)

            /*try {
                logging.update("psi", robot.pressureSensor.pressure.toString())
            }
            catch (e: Exception) {
                println("BIG BORK LOGGING BORKED!")
                e.printStackTrace()
            }*/
        }
    }

    override fun robotDisabled() {
        robot.drivetrain.setDriveVelocity(0.0, 0.0)
        println("Robot Disabled")

        // Logging
        SmartDashboard.putNumber("Pressure (psi)", robot.pressureSensor.pressure)
        SmartDashboard.putData("PDP", PDP.pdp)

        try {
            logging.update("psi", robot.pressureSensor.pressure.toString())
        }
        catch (e: Exception) {
            println("BIG BORK LOGGING BORKED!")
            e.printStackTrace()
        }
    }

    override fun disabledPeriodic() {
        // Communism

        // Logging
        SmartDashboard.putNumber("Pressure (psi)", robot.pressureSensor.pressure)
        SmartDashboard.putData("PDP", PDP.pdp)

        try {
            logging.update("psi", robot.pressureSensor.pressure.toString())
        }
        catch (e: Exception) {
            println("BIG BORK LOGGING BORKED!")
            e.printStackTrace()
        }
    }
}

