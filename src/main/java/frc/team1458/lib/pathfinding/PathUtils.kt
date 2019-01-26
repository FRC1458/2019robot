package frc.team1458.lib.pathfinding

import frc.team1458.lib.odom.Pose2D
import frc.team1458.lib.util.maths.TurtleMaths
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

object PathUtils {
    fun generateLinearPath(points: Array<Pair<Double, Double>>, numPoints: Int): Array<Pair<Double, Double>> {
        val out: ArrayList<Pair<Double, Double>> = ArrayList(numPoints)
        var length = points.toList().zipWithNext { a, b -> TurtleMaths.distance(a, b) }.sum()
        var gap = length / (numPoints.toDouble() - 1.0)

        for ((a, b) in points.toList().zipWithNext()) {
            val numPts = (TurtleMaths.distance(a, b) / gap).toInt()

            for (i in 0..numPts) {
                val x = i.toDouble() / numPts.toDouble()
                val xo = 1.0 - x
                out.add(Pair(a.first * xo + b.first * x, a.second * xo + b.second * x))
            }
        }

        return out.toTypedArray()
    }

    //angle is in degrees, currentAngle in degrees. only call this function once.
    fun interpolateTurnArcWithAngle(
        turnAngle: Double, currentAngle: Double,
        currentPosition: Pair<Double, Double>,
        turnMode: String, numberPoints: Int,
        radius: Double
    ): Array<Pair<Double, Double>> {

        var turnAngleRad = turnAngle * (Math.PI / 180.0)
        var currentAngleRad = currentAngle * (Math.PI / 180.0)

        //dTheta is in
        var array: Array<Pair<Double, Double>> = Array<Pair<Double, Double>>(numberPoints + 1) { i -> currentPosition }
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

            var pointAdjusted: Pair<Double, Double> = Pair(
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
    ): Array<Pair<Double, Double>> {

        //convert angles of points to degrees
        var vector1Angle = (startPoint.theta * (Math.PI / 180.0)) + 0.000001
        var vector2Angle = (endPoint.theta * (Math.PI / 180.0)) + 0.000001
        println("start angle: " + vector1Angle)
        println("end angle: " + vector2Angle)

        //create direction vectors for dot product
        var vector1 = Pair(Math.cos(vector1Angle), Math.sin(vector1Angle))
        println("Vector 1: " + vector1)

        var vector2 = Pair(Math.cos(vector2Angle), Math.sin(vector2Angle))
        println("Vector 2: " + vector2)

        var dotProduct: Double = (vector1.first * vector2.first) + (vector1.second * vector2.second)
        println("Dot Product: " + dotProduct)

        // cos(theta) = vector1 . vector2 / (||vector1|| ||vector2||)
        var angleBetweenVectors = Math.acos(dotProduct) //both vectors have magnitude 1
        println("Angle Between Vectors: : " + angleBetweenVectors)

        //finding external angle that's formed when vectors extend outward
        angleBetweenVectors =
            if (dotProduct >= 0) Math.abs(Math.PI - angleBetweenVectors) else Math.abs(angleBetweenVectors)

        var centralAngle = Math.PI - angleBetweenVectors //quadrilateral
        println("Central Angle: " + centralAngle)

        var distancePoints = ((startPoint.x - endPoint.x).pow(2) + (startPoint.y - endPoint.y).pow(2)).pow(.5)
        println("Distance Points: " + distancePoints)
        var radius = (distancePoints / 2) / Math.sin(centralAngle / 2) //triangle has angle centralAngle / 2
        println("radius: " + radius)

        var centerx: Double =
            ((Math.tan(vector1Angle) * endPoint.x)
                    + (Math.tan(vector1Angle) * Math.tan(vector2Angle) * endPoint.y)
                    - (Math.tan(vector1Angle) * Math.tan(vector2Angle) * startPoint.y)
                    - (Math.tan(vector2Angle) * startPoint.x)) /
                    (Math.tan(vector1Angle) - Math.tan(vector2Angle))

        println("centerx: " + centerx)

        //y = mx + b
        var centery: Double =
            -((1.0 / Math.tan(vector1Angle)) * (centerx - startPoint.x)) + startPoint.y
        println("centery: " + centery)

        var dTheta = centralAngle / numberPoints.toDouble()
        println("dTheta: " + dTheta)

        //a left turn is a counterclockwise rotation, and use a negative angle in
        //rotation matrix
        dTheta = if (turnMode == "left") dTheta else if (turnMode == "right") dTheta else 0.0

        var arrayPoints = Array<Pair<Double, Double>>(size = numberPoints + 1) { i -> Pair(0.0, 0.0) }

        for (i in 0..numberPoints) {

            var newX = ((startPoint.x - centerx) * Math.cos(dTheta * i))
            - ((startPoint.y - centery) * Math.sin(dTheta * i)) + centerx
            println("[ " + ((startPoint.x - centerx) * Math.cos(dTheta * i)) + ", " + (-((startPoint.y - centery) * Math.sin(dTheta * i))) + " ]")

            var newY = ((startPoint.x - centerx) * Math.sin(dTheta * i))
            + ((startPoint.y - centery) * Math.cos(dTheta * i)) + centery
            println("[ " + ((startPoint.x - centerx) * Math.sin(dTheta * i)) + ", " + ((startPoint.y - centery) * Math.cos(dTheta * i)))

            arrayPoints[i] = Pair(newX, newY)
            /*
               gx = (sx-cx)*cos((i/4)*ψ)-(sy-cy)*sin((i/4)*ψ) + cx
               gy = (sx-cx)*sin((i/4)*ψ)+(sy-cy)*cos((i/4)*ψ) + cy

               shift the points to the origin before rotation (i.e. sx - cx)
               then shift them back (+ kx)
            */

        }

        return arrayPoints
    }
}