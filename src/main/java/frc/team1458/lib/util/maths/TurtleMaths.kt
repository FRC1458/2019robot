package frc.team1458.lib.util.maths

/**
 * Math utility classes
 */
object TurtleMaths {

    val TWOPI = 2.0 * 3.14159265

    fun constrainAngle(angle: Double) : Double {
        var a = angle
        while(a >= TWOPI) {
            a -= TWOPI
        }
        while(a <= 0) {
            a += TWOPI
        }
        return a
    }

    fun constrain(value: Double, min: Double, max: Double) : Double {
        if(value > max) {
            return max
        } else if (value < min) {
            return min
        } else {
            return value
        }
    }

    fun deadband(value: Double, deadband: Double = 0.15): Double {
        if(Math.abs(value) < deadband) {
            return 0.0
        } else {
            return value
        }
    }

    fun shift(value: Double, minA: Double, maxA: Double, minB: Double, maxB: Double) : Double {
        return minB + ((maxB - minB) / (maxA - minA)) * (value - minA)
    }

    fun calculateSD(numArray: List<Double>): Double {
        var sum = 0.0
        var standardDeviation = 0.0

        for (num in numArray) {
            sum += num
        }

        val mean = sum / numArray.size

        for (num in numArray) {
            standardDeviation += Math.pow(num - mean, 2.0)
        }

        return Math.sqrt(standardDeviation / numArray.size)
    }

    fun distance(pt1: Pair<Double, Double>, pt2: Pair<Double, Double>): Double {
        return Math.sqrt((pt1.first - pt2.first) * (pt1.first - pt2.first) + (pt1.second - pt2.second) * (pt1.second - pt2.second))
    }
}

fun Double.format(digits: Int) = java.lang.String.format("%.${digits}f", this)
fun Float.format(digits: Int) = java.lang.String.format("%.${digits}f", this)
