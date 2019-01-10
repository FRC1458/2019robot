package frc.team1458.robot

import edu.wpi.first.wpilibj.RobotBase

object Main {
    // Main method for starting the robot
    @JvmStatic
    fun main(args : Array<String>) {
        RobotBase.startRobot(::Robot)
    }
}