package frc.team1458.lib.pathfinding

import frc.team1458.lib.util.maths.TurtleMaths
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator
import kotlin.math.*

object PathUtils {
    fun generateLinearPath(
        points: Array<Triple<Double, Double, Double>>,
        numPoints: Int
    ): Array<Triple<Double, Double, Double>> {
        val out: ArrayList<Triple<Double, Double, Double>> = ArrayList(numPoints)
        val length = points.toList()
            .zipWithNext { a, b -> TurtleMaths.distance(Pair(a.first, a.second), Pair(b.first, b.second)) }.sum()
        val gap = length / (numPoints.toDouble() - 1.0)

        for ((a, b) in points.toList().zipWithNext()) {
            val numPts = (TurtleMaths.distance(Pair(a.first, a.second), Pair(b.first, b.second)) / gap).toInt()

            for (i in 0..numPts) {
                val x = i.toDouble() / numPts.toDouble()
                val xo = 1.0 - x
                out.add(Triple(a.first * xo + b.first * x, a.second * xo + b.second * x, a.third))
            }
        }

        return out.toTypedArray()
    }

    fun removeCurvature(points: Array<Triple<Double, Double, Double>>): Array<Pair<Double, Double>> {
        return points.map { Pair(it.first, it.second) }.toTypedArray()
    }

    //angle is in degrees, currentAngle in degrees. only call this function once.
    fun interpolateTurnArcWithAngle(
        turnAngle: Double, currentAngle: Double,
        currentPosition: Pair<Double, Double>,
        turnMode: String, numberPoints: Int,
        radius: Double
    ): Array<Pair<Double, Double>> {

        val turnAngleRad = turnAngle * (Math.PI / 180.0)
        val currentAngleRad = currentAngle * (Math.PI / 180.0)

        //dTheta is in
        val array: Array<Pair<Double, Double>> = Array(numberPoints + 1) { i -> currentPosition }
        var dTheta = turnAngleRad / numberPoints.toDouble()

        dTheta = if (turnMode == "left") dTheta else if (turnMode == "right") -dTheta else 0.0

        for (i in 0..numberPoints) {

            var rawPoints: Pair<Double, Double> = Pair(
                radius * cos(currentAngle + (dTheta * i)) - radius * cos(currentAngleRad),
                (radius * (sin(currentAngle + (dTheta * i)))) - (radius * sin(currentAngleRad))
            )


            // println(rawPoints) //point 1 should be somewhere on circle centered on origin

            //rotate point right if turning left
            if (turnMode == "left") {
                rawPoints = Pair(rawPoints.second, -1.0 * rawPoints.first)
            } else if (turnMode == "right") {
                rawPoints = Pair(-1.0 * rawPoints.second, rawPoints.first)
            }

            // println(rawPoints) //after rotation should be

            val pointAdjusted: Pair<Double, Double> = Pair(
                rawPoints.first + currentPosition.first,
                (rawPoints.second + currentPosition.second)
            )

            array[i] = pointAdjusted
        }

        return array
    }

    fun interpolateArcBetweenTwoPoints(
        startPoint: Pose2DPathfinding,
        endPoint: Pose2DPathfinding,
        numberPoints: Int,
        turnMode: String
    ): Array<Triple<Double, Double, Double>> {
        var newX: Double
        var newY: Double

        //convert angles of points to degrees
        val vector1Angle = (startPoint.theta * (Math.PI / 180.0)) + 0.000001
        val vector2Angle = (endPoint.theta * (Math.PI / 180.0)) + 0.000001

        //create direction vectors for dot product
        val vector1 = Pair(Math.cos(vector1Angle), Math.sin(vector1Angle))

        val vector2 = Pair(Math.cos(vector2Angle), Math.sin(vector2Angle))

        val dotProduct: Double = (vector1.first * vector2.first) + (vector1.second * vector2.second)

        // cos(theta) = vector1 . vector2 / (||vector1|| ||vector2||)
        var angleBetweenVectors = Math.acos(dotProduct) //both vectors have magnitude 1

        //finding external angle that's formed when vectors extend outward
        angleBetweenVectors =
            if (dotProduct >= 0) Math.abs(Math.PI - angleBetweenVectors) else Math.abs(angleBetweenVectors)

        val centralAngle = Math.PI - angleBetweenVectors //quadrilateral

        val distancePoints = ((startPoint.x - endPoint.x).pow(2) + (startPoint.y - endPoint.y).pow(2)).pow(.5)

        val radius = (distancePoints / 2) / Math.sin(centralAngle / 2) //triangle has angle centralAngle / 2

        val centerX: Double =
            ((Math.tan(vector1Angle) * endPoint.x)
                    + (Math.tan(vector1Angle) * Math.tan(vector2Angle) * endPoint.y)
                    - (Math.tan(vector1Angle) * Math.tan(vector2Angle) * startPoint.y)
                    - (Math.tan(vector2Angle) * startPoint.x)) /
                    (Math.tan(vector1Angle) - Math.tan(vector2Angle))


        //y = mx + b
        val centerY: Double =
            -((1.0 / Math.tan(vector1Angle)) * (centerX - startPoint.x)) + startPoint.y

        var dTheta = centralAngle / numberPoints.toDouble()
        var curvatureAtPt = 1000000.0

        //a left turn is a counterclockwise rotation, and use a negative angle in
        //rotation matrix
        dTheta = if (turnMode == "left") dTheta else if (turnMode == "right") -dTheta else 0.0

        val arrayPoints =
            Array(size = numberPoints + 1) { i -> Triple(0.0, 0.0, radius) }

        for (i in 0..numberPoints) {

            newX =
                ((startPoint.x - centerX) * Math.cos(dTheta * i)) - ((startPoint.y - centerY) * Math.sin(dTheta * i)) + centerX

            newY =
                ((startPoint.x - centerX) * Math.sin(dTheta * i)) + ((startPoint.y - centerY) * Math.cos(dTheta * i)) + centerY

            arrayPoints[i] = Triple(newX, newY, curvatureAtPt)

            /*
               gx = (sx-cx)*cos((i/4)*ψ)-(sy-cy)*sin((i/4)*ψ) + cx
               gy = (sx-cx)*sin((i/4)*ψ)+(sy-cy)*cos((i/4)*ψ) + cy

               shift the points to the origin before rotation (i.e. sx - cx)
               then shift them back (+ kx)
            */

        }

        return arrayPoints
    }

    fun generateVXGraph(
        distance: Double,
        max_vel: Array<Double>,
        max_linear_accel: Double,
        start_vel: Double,
        end_vel: Double
    ): Pair<Array<Double>, Array<Double>> {

        val n = max_vel.size
        val x = TurtleMaths.linspace(0.0, distance, n)
        val v = TurtleMaths.linspace(0.0, 0.0, n)

        v[0] = start_vel
        v[n - 1] = end_vel

        for (i in (1 until n)) {
            v[i] = min(max_vel[i], sqrt(v[i - 1].pow(2.0) + 2 * max_linear_accel * (x[i] - x[i - 1])))
        }

        v[0] = start_vel
        v[n - 1] = end_vel

        for (i in (1 until n-1).reversed()) {
            v[i] = min(v[i], sqrt(v[i + 1].pow(2.0) + 2 * max_linear_accel * (x[i + 1] - x[i])))
        }

        return Pair(x, v)
    }

    fun xvTotv(x: Array<Double>, v: Array<Double>): Pair<Array<Double>, Array<Double>> {

        val n = x.size
        val t = TurtleMaths.linspace(0.0, 0.0, n)
        var dt: Double
        var dx: Double
        var a: Double

        for (i in 1 until n) {
            dx = (x[i] - x[i - 1])
            a = (v[i].pow(2.0) - v[i - 1].pow(2.0)) / (2.0 * dx)

            if (abs(a) < 0.0001) {
                dt = dx / v[i - 1]
            } else {
                var root = v[i - 1].pow(2.0) - 4.0 * 0.5 * (a) * -(dx)

                if (abs(root) < 0.0001) {
                    root = 0.0
                }

                dt = (-v[i - 1] + sqrt(root)) / (a)
            }

            t[i] = t[i - 1] + dt


        }


        return Pair(t, v)

    }

    fun consistentTime(t: Array<Double>, v: Array<Double>, dt: Double): Array<Double> {
        val func = LinearInterpolator().interpolate(t.toDoubleArray(), v.toDoubleArray())
        return TurtleMaths.linspace(0.0, t.last(), (t.last() / dt).roundToInt()).map { func.value(it) }.toTypedArray()
    }

}