package frc.team1458.lib.util.maths

import koma.*
import koma.extensions.*
import koma.matrix.Matrix

class QuinticPolynomialSolver(
    xs: Double,
    vxs: Double,
    axs: Double,
    xe: Double,
    vxe: Double,
    axe: Double,
    T: Double
) {
    private val a0 = xs
    private val a1 = vxs
    private val a2 = axs / 2.0

    private val aMatrix: Matrix<Double> = create(
        arrayOf(
            doubleArrayOf(T.pow(3.0), T.pow(4.0), T.pow(5.0)),
            doubleArrayOf(3.0 * T.pow(2.0), 4.0 * T.pow(3.0), 5.0 * T.pow(4.0)),
            doubleArrayOf(6.0 * T, 12.0 * T.pow(2.0), 20.0 * T.pow(3.0))
        )
    )

    private val bMatrix: Matrix<Double> = create(
        arrayOf(
            doubleArrayOf(
                xe - a0 - a1 * T - a2 * T.pow(2.0),
                vxe - a1 - 2.0 * a2 * T,
                axe - 2.0 * a2
            )
        )
    )

    private val x: Matrix<Double> = aMatrix * bMatrix

    private val a3 = x[0]
    private val a4 = x[1]
    private val a5 = x[2]

    fun calculatePoint(t: Double): Double {
        return a0 + a1 * t + a2 * t.pow(2.0) + a3 * t.pow(3.0) + a4 * t.pow(4.0) + a5 * t.pow(5.0)
    }

    fun calculateFirstDerivative(t: Double): Double {
        return a1 + 2.0 * a2 * t + 3.0 * a3 * t.pow(2.0) + 4.0 * a4 * t.pow(3.0) + 5.0 * a5 * t.pow(4.0)
    }

    fun calculateSecondDerivative(t: Double): Double {
        return 2.0 * a2 + 6.0 * a3 * t + 12.0 * a4 * t.pow(2.0) + 20.0 * a5 * t.pow(3.0)
    }

    fun calculateThirdDerivative(t: Double): Double {
        return 6.0 * a3 + 24.0 * a4 * t + 60.0 * a5 * t.pow(2.0)
    }
}