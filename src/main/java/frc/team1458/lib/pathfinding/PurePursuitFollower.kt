package frc.team1458.lib.pathfinding

import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.maths.TurtleMaths
import kotlin.math.* // TODO check if  messed with: changed from java math to kotlin math


class PurePursuitFollower(
    val points: Array<Pair<Double, Double>>,
    val lookahead: Double,
    val scaling: Double,
    val wheelbase: Double,
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

    private fun poseToPoint(
        pos: Pair<Double, Double>,
        angle: Double,
        newPoint: Pair<Double, Double>
    ): Pair<Double, Double> {
        val deltax = newPoint.first - pos.first
        val deltay = newPoint.second - pos.second

        val angleRad = -1 * angle // + (3.1415926 / 2.0)
        //converts to robot local coordinate frame
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

    fun finished(pos: Pair<Double, Double>): Boolean = (TurtleMaths.distance(pos, points.last()) < targetTolerance)

    fun getControl(pos: Pair<Double, Double>, angle: Double, speed: Double): Pair<Double, Double> {
        if (finished(pos)) {
            return Pair(0.0, 0.0) // Stops robot if within the target tolerance
        }

        val lookaheadpt = getLookahead(pos)
        LiveDashboard.putPath(lookaheadpt.first, lookaheadpt.second, 0.0)

        val pt = poseToPoint(pos, angle, lookaheadpt)
        println("Robot Frame Point: (" + pt.first + ", " + pt.second + ")")
        val invcurve: Double =
            2.0 * pt.second / (lookahead * lookahead/*pt.first * pt.first + pt.second * pt.second*/)
        println("InvCurvature: $invcurve")
        println("Curvature: ${1.0 / invcurve}")

        return arcToTank(invcurve, speed)

    }
}