package frc.team1458.lib.pathfinding

import frc.team1458.lib.util.maths.TurtleMaths

object PathUtils {
    fun generateLinearPath(points: Array<Pair<Double, Double>>, numPoints: Int): Array<Pair<Double, Double>> {
        val out: ArrayList<Pair<Double, Double>> = ArrayList(numPoints)
        var length = points.toList().zipWithNext { a, b -> TurtleMaths.distance(a, b) }.sum()
        var gap = length / (numPoints.toDouble() - 1.0)

        for((a, b) in points.toList().zipWithNext()) {
            val numPts = (TurtleMaths.distance(a, b) / gap).toInt()

            for(i in 0..numPts) {
                val x = i.toDouble() / numPts.toDouble()
                val xo = 1.0 - x
                out.add(Pair(a.first * xo + b.first * x, a.second * xo + b.second * x))
            }
        }

        return out.toTypedArray()
    }
}