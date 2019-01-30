package frc.team1458.lib.pathfinding

import frc.team1458.lib.util.maths.QuinticPolynomialSolver
import koma.arange
import koma.create
import kotlin.math.*

object QuinticPolynomialPlanner {

    fun planPointToPointPath(
        startPose: QuinticPlannerPose,
        endPose: QuinticPlannerPose,
        maximumAcceleration: Double = 2.0, // ft/s^2 - Maximum acceleration value
        maximumJerk: Double = 1.5, // ft/s^2 - Maximum jerk value
        timeStepSeconds: Double = 0.02, // s - Time between each update
        minimumTime: Double = 2.0, // s - Minimum time for path to complete
        maximumTime: Double = 100.0 // s - Maximum time for path to complete
    ): Array<ArrayList<Double>> {

        val vxs = startPose.velocity * cos(startPose.theta)
        val vys = startPose.velocity * sin(startPose.theta)
        val vxg = endPose.velocity * cos(endPose.theta)
        val vyg = endPose.velocity * sin(endPose.theta)

        val axs = startPose.acceleration * cos(startPose.theta)
        val ays = startPose.acceleration * sin(startPose.theta)
        val axg = endPose.acceleration * cos(endPose.theta)
        val ayg = endPose.acceleration * sin(endPose.theta)

        var time: ArrayList<Double> = arrayListOf()
        var rx: ArrayList<Double> = arrayListOf()
        var ry: ArrayList<Double> = arrayListOf()
        var ryaw: ArrayList<Double> = arrayListOf()
        var rv: ArrayList<Double> = arrayListOf()
        var ra: ArrayList<Double> = arrayListOf()
        var rj: ArrayList<Double> = arrayListOf()

        var xqp: QuinticPolynomialSolver
        var yqp: QuinticPolynomialSolver

        var vx: Double
        var vy: Double
        var v: Double
        var yaw: Double

        var ax: Double
        var ay: Double
        var a: Double

        var jx: Double
        var jy: Double
        var j: Double

        for (t: Double in arange(minimumTime, maximumTime, minimumTime).toList()) {
            xqp = QuinticPolynomialSolver(startPose.x, vxs, axs, endPose.x, vxg, axg, t)
            yqp = QuinticPolynomialSolver(startPose.y, vys, ays, endPose.y, vyg, ayg, t)

            time = arrayListOf()
            rx = arrayListOf()
            ry = arrayListOf()
            ryaw = arrayListOf()
            rv = arrayListOf()
            ra = arrayListOf()
            rj = arrayListOf()

            for (tn: Double in arange(0.0, t + timeStepSeconds, timeStepSeconds).toList()) {
                time.add(t)

                rx.add(xqp.calculatePoint(t))
                ry.add(yqp.calculatePoint(t))

                vx = xqp.calculateFirstDerivative(t)
                vy = yqp.calculateFirstDerivative(t)
                v = hypot(vx, vy)
                yaw = atan2(vy, vx)

                rv.add(v)
                ryaw.add(yaw)

                ax = xqp.calculateSecondDerivative(t)
                ay = yqp.calculateSecondDerivative(t)
                a = hypot(ax, ay)

                if ((rv.size >= 2) && (rv[rv.size-1] - rv[rv.size-2] < 0.0)) {
                    a *= -1.0
                }

                ra.add(a)

                jx = xqp.calculateThirdDerivative(t)
                jy = yqp.calculateThirdDerivative(t)
                j = hypot(jx, jy)

                if ((ra.size >= 2) && (ra[ra.size - 1] - ra[ra.size-2] < 0.0)) {
                    j *= -1.0
                }

                rj.add(j)

            }

            if ((koma.max(create(ra.toDoubleArray())) <= maximumAcceleration) && (koma.max(create(rj.toDoubleArray())) <= maximumJerk)) {
                println("Quintic Polynomial Path Found!")
                break
            }
        }

        return arrayOf(time, rx, ry, ryaw, rv, ra, rj)
    }
}