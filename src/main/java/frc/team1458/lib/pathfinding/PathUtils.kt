package frc.team1458.lib.pathfinding

import frc.team1458.lib.util.maths.TurtleMaths
import kotlin.math.cos
import kotlin.math.sin

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

    //angle is in degrees, currentAngle in degrees. only call this function once.
    fun interpolateTurnArcWithAngle(turnAngle: Double, currentAngle: Double,
                                    currentPosition: Pair<Double, Double>,
                                    turnMode: String, numberPoints: Int,
                                    radius: Double): Array<Pair<Double, Double>>
    {

        var turnAngleRad = turnAngle * (Math.PI / 180.0)
        var currentAngleRad = currentAngle * (Math.PI / 180.0)

        //dTheta is in
        var array: Array<Pair<Double, Double>> = Array<Pair<Double, Double>>(numberPoints + 1) { i -> currentPosition}
        var dTheta = turnAngleRad / numberPoints.toDouble()

        dTheta = if (turnMode == "left") dTheta else if (turnMode == "right") -dTheta else 0.0

        for (i in 1..numberPoints)
        {

            var rawPoints: Pair<Double, Double> = Pair(radius * cos(currentAngle + (dTheta * i)),
                radius * sin(currentAngle + (dTheta * i))
            )

            println(rawPoints) //point 1 should be somewhere on circle centered on origin

            //rotate point right if turning left
            if (turnMode == "left") {
                rawPoints = Pair(rawPoints.second, -rawPoints.first)
            }
            else if (turnMode == "right")
            {
                rawPoints = Pair(-rawPoints.second, rawPoints.first)
            }

            println(rawPoints) //after rotation should be

            var pointAdjusted: Pair<Double, Double> = Pair(rawPoints.first - radius * cos(currentAngleRad) + currentPosition.first,
                (rawPoints.second - radius * sin(currentAngleRad) + currentPosition.second)* -1.0)

            array[(numberPoints) - i] = pointAdjusted
        }

        return array
    }
}