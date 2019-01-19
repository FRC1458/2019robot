package frc.team1458.lib.pathfinding

import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.maths.TurtleMaths
import kotlin.math.* // TODO check if  messed with: changed from java math to kotlin math


class PurePursuitFollower(
    val points: Array<Pair<Double, Double>>,
    val lookahead: Double,
    val scaling: Double,
    val wheelbase: Double, // Chassis width
    val targetTolerance: Double = 0.3
) {

    var lastLookahead = 0

    private fun getLookahead(robotPos: Pair<Double, Double>): Pair<Double, Double> {
        for (i in lastLookahead until points.size) {
            if (TurtleMaths.distance(robotPos, points[i]) > lookahead) {
                lastLookahead = i
                println("Next Look Ahead Point: " + points[i])
                return points[i]
            }
        }

        lastLookahead = points.size - 1
        return points.last()
    }

    // Get the closest point to the robot on the path, does not affect lastLookahead variable
    private fun getClosestPathPoint(robotPos: Pair<Double, Double>): Pair<Double, Double> {
        var closestIndex = 0
        var closestDist = 0.0
        var dist: Double

        // TODO Maybe make more efficient, possibly based on position in path
        for (i in 0 until points.size) { // TODO Maybe out of bounds error, maybe inclusive
            dist = TurtleMaths.distance(robotPos, points[i])

            if (i == 0) {
                closestIndex = i
                closestDist = dist
            } else if (dist < closestDist) {
                closestIndex = i
                closestDist = dist
            }

        }

        return points[closestIndex]
    }

    // Applies translation as well as rotation matrix to convert the points to robot coordinate space
    private fun poseToPoint(
        pos: Pair<Double, Double>,
        angle: Double,
        newPoint: Pair<Double, Double>
    ): Pair<Double, Double> {
        val deltax = newPoint.first - pos.first
        val deltay = newPoint.second - pos.second

        val angleRad = -1 * angle // + (3.1415926 / 2.0)

        return Pair(deltax * cos(angleRad) - deltay * sin(angleRad), deltax * sin(angleRad) + deltay * cos(angleRad))
    }

    private fun arcToTank(invcurve: Double, vel: Double): Pair<Double, Double> {
        if (abs(invcurve) < 0.01) {
            return Pair(scaling * vel, scaling * vel)
        } else {
            val left = scaling * vel * (1.0 - invcurve * wheelbase / 2.0)
            //val left = scaling * vel * invcurve * ((1.0 / invcurve) - (wheelbase / 2.0))
            val right = scaling * vel * (1.0 + invcurve * wheelbase / 2.0)
            //val right = scaling * vel * invcurve * ((1.0 / invcurve) + (wheelbase / 2.0))

            println("left: $left = $scaling * $vel * (1.0 - $invcurve * 0.98)")
            println("right: $right = $scaling * $vel * (1.0 + $invcurve * 0.98)")

            return Pair(left, right)
        }
    }

    // Checks if the path following has finished using the target tolerance distance to the robot
    fun finished(pos: Pair<Double, Double>): Boolean = (TurtleMaths.distance(pos, points.last()) < targetTolerance)

    // Takes the position and returns the velocity values for the drivetrain
    fun getControl(pos: Pair<Double, Double>, angle: Double, speed: Double): Pair<Double, Double> {
        if (finished(pos)) {
            return Pair(0.0, 0.0) // Stops robot if within the target tolerance
        }

        // val lookaheadpt = getLookahead(pos) TODO Compare new lookahead function
        val lookaheadpt = getClosestPathPoint(pos)
        LiveDashboard.putPath(lookaheadpt.first, lookaheadpt.second, 0.0)

        val pt = poseToPoint(pos, angle, lookaheadpt)
        println("Robot Frame Point: (" + pt.first + ", " + pt.second + ")")
        val invcurve: Double =
            (2.0 * pt.second) / (lookahead * lookahead/*pt.first * pt.first + pt.second * pt.second*/)
        println("InvCurvature: $invcurve")
        println("Curvature: ${1.0 / invcurve}")

        return arcToTank(invcurve, speed)

    }
}