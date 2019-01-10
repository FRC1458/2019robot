package frc.team1458.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.core.BaseRobot


class Robot : BaseRobot() {

    override fun robotSetup() {
        println("Hello World!")
    }

    override fun runAuto() {
        println("Warning: Sandstorm")
    }

    override fun teleopInit() {
        // likely nothing here
    }

    override fun teleopPeriodic() {
        // drive code - runs around 50hz
    }

    override fun runTest() {
        // rewind mechanism, run compressor, etc
    }

    override fun robotDisabled() {}
    override fun disabledPeriodic() {}
}
