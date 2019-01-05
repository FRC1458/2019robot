package frc.team1458.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.I2C

import jaci.pathfinder.Trajectory
import com.kauailabs.navx.frc.AHRS
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

// var address = 0x00
// val port = I2C(I2C.Port.kOnboard, address)

val tal = TalonSRX(1);
val traj = Trajectory(1)
val jk = AHRS(I2C.Port.kOnboard)


class Robot : TimedRobot() {
    private var m_autoSelected: String? = null
    private val m_chooser = SendableChooser<String>()

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    override fun robotInit() {
        m_chooser.setName("Default Auto", kDefaultAuto)
        m_chooser.setName("My Auto", kCustomAuto)
        SmartDashboard.putData("Auto choices", m_chooser)
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
    }

    /**
     * This function is called periodically during autonomous.
     */
    override fun autonomousPeriodic() {
        when (m_autoSelected) {
            kCustomAuto -> {
            }
            kDefaultAuto -> {
            }
            else -> {
            }
        }// Put custom auto code here
        // Put default auto code here
    }

    /**
     * This function is called periodically during operator control.
     */
    override fun teleopPeriodic() {}

    /**
     * This function is called periodically during test mode.
     */
    override fun testPeriodic() {}

    companion object {
        private val kDefaultAuto = "Default"
        private val kCustomAuto = "My Auto"
    }
}