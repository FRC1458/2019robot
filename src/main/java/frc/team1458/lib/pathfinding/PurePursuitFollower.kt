package frc.team1458.lib.pathfinding

import frc.team1458.lib.util.LiveDashboard
import frc.team1458.lib.util.maths.TurtleMaths
import java.lang.Math.*


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

    private fun arcToTank(curve: Double, vel: Double): Pair<Double, Double> {
        if (abs(curve) < 0.01) {
            return Pair(scaling * vel, scaling * vel)
        } else {
            // may need to swap signs here idk
            val left = scaling * vel * curve * ((1.0 / curve) - (wheelbase / 2.0))
            val right = scaling * vel * curve * ((1.0 / curve) + (wheelbase / 2.0))
            //TODO: maybe flip values

            return Pair(left, right)
        }
    }

    fun finished(pos: Pair<Double, Double>): Boolean = (TurtleMaths.distance(pos, points.last()) < targetTolerance)

    fun getControl(pos: Pair<Double, Double>, angle: Double, speed: Double): Pair<Double, Double> {
        if (finished(pos)) {
            return Pair(0.0, 0.0)
        }

        val lookahead = getLookahead(pos)
        LiveDashboard.putPath(lookahead.first, lookahead.second, 0.0)

        val pt = poseToPoint(pos, angle, lookahead)
        println("Robot Frame Point: (" + pt.first + ", " + pt.second + ")")
        val curvature = -2.0 * pt.first / (pt.first * pt.first + pt.second * pt.second) //TODO: maybe remofe negative sign
        println("Curvature: " + curvature)

        return arcToTank(curvature, speed)

    }
}