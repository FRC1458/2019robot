package frc.team1458.robot

import edu.wpi.first.wpilibj.RobotBase
import frc.team1458.lib.pathfinding.PathUtils
import frc.team1458.lib.util.maths.TurtleMaths
import java.util.*

object Test {
    // Main method for testing
    @JvmStatic
    fun main(args : Array<String>) {
        println("turtwig is best")

        println(TurtleMaths.distance(Pair(3.0, 0.0), Pair(0.0, 4.0)))
        val path = PathUtils.generateLinearPath(
                arrayOf(Pair(0.0, 0.0), Pair(6.0, 0.0), Pair(6.0, 6.0), Pair(0.0, 6.0), Pair(0.0, 0.0)), 250)
        print(Arrays.toString(path))
    }
}