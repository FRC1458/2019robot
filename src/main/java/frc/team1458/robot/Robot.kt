package frc.team1458.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.drive.TankDrive
import frc.team1458.lib.pid.PIDConstants
import frc.team1458.lib.sensor.NavX
import frc.team1458.lib.util.flow.delay


class Robot : TimedRobot() {
    private var m_autoSelected: String? = null
    private val m_chooser = SendableChooser<String>()


    // Stolen from Anish's code
    val opInt = OI()
    val robot = RobotMapFinalChassis(opInt)

    var drivetrainInverted = false
    var lastTriggered = false


    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    override fun robotInit() {
        m_chooser.setName("Default Auto", kDefaultAuto)
        m_chooser.setName("My Auto", kCustomAuto)
        SmartDashboard.putData("Auto choices", m_chooser)

        lastTriggered = false
        // Hah we only have 1 gear so is it safe to remove? :thonk:
        robot.drivetrain.lowGear()
        robot.drivetrain.leftMaster.connectedEncoder.zero()
        robot.drivetrain.rightMaster.connectedEncoder.zero()
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {}

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     *
     * You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    override fun autonomousInit() {
        m_autoSelected = m_chooser.selected
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        println("Auto selected: " + m_autoSelected!!)

        // Test autonomous script
        robot.drivetrain.tankDrive(0.1, 0.1)
        delay(500)
        robot.drivetrain.tankDrive(0.0, 0.0)
    }

    /**
     * This function is called periodically during autonomous.
     */
    override fun autonomousPeriodic() {
        when (m_autoSelected) {
            kCustomAuto -> {
            }
            kDefaultAuto ->{
            }
            else -> {
            }
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    override fun teleopPeriodic() {
        // Stolen from Anish's code:
        // Press button to invert drivetrain
        if(opInt.invertButton.triggered && !lastTriggered) {
            drivetrainInverted = !drivetrainInverted

            /* Switch camera if drivetrain inverted
            if(drivetrainInverted) {
                cameraServer.source = rearCamera
            } else {
                cameraServer.source = frontCamera
            } */
        }

        lastTriggered = opInt.invertButton.triggered

        robot.drivetrain.arcadeDrive(
            if (drivetrainInverted) { -0.5 * (opInt.throttleAxis.value) }
            else if (opInt.slowDownButton.triggered) { 0.5 * opInt.throttleAxis.value }
            else { opInt.throttleAxis.value },
            if (drivetrainInverted) { (opInt.steerAxis.value) } else { opInt.steerAxis.value }
        )
    }
    override fun testPeriodic() {}

    companion object {
        private val kDefaultAuto = "Default"
        private val kCustomAuto = "My Auto"
    }
}