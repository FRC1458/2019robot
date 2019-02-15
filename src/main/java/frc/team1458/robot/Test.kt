package frc.team1458.robot

import edu.wpi.first.wpilibj.RobotBase
import frc.team1458.lib.pathfinding.PathUtils
import frc.team1458.lib.util.maths.TurtleMaths
import java.util.*






object Test {
    // Main method for testing
    @JvmStatic
    fun main(args : Array<String>) {
        RobotBase.startRobot(::TestRobot)
    }
}