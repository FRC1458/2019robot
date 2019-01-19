package frc.team1458.lib.core

import edu.wpi.first.wpilibj.SampleRobot
import frc.team1458.lib.util.DataLogger
import frc.team1458.lib.util.TelemetryLogger
import frc.team1458.lib.util.flow.WaitGroup
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.util.flow.go
import frc.team1458.lib.util.flow.systemTimeMillis
import java.util.*

/*
 * All robot classes should extend from this class.
 *
 * @author asinghani
 */

// TODO Change SampleRobot to non-deprecated replacement
abstract class BaseRobot : SampleRobot {

    constructor() : super()

    /**
     * Initialize the robot's hardware and basic configuration.
     */
    abstract fun robotSetup()

    /**
     * Runs auton
     */
    abstract fun runAuto()

    /**
     * Setup for teleop (if necessary)
     */
    abstract fun teleopInit()

    /**
     * Called constantly during teleop period
     */
    abstract fun teleopPeriodic()

    /**
     * Run the robot's test mode.
     */
    abstract fun runTest()

    /**
     * Called when the robot is disabled.
     */
    abstract fun robotDisabled()
    abstract fun disabledPeriodic()


    override fun robotInit() {
        robotSetup()
    }

    override fun disabled() {
        robotDisabled()
        while(isDisabled) {
            disabledPeriodic()
            delay(3)
        }
    }

    override fun autonomous() {
        runAuto()
    }

    override fun operatorControl() {
        teleopInit()
        while (super.isOperatorControl() && super.isEnabled()) {
            var lastStartMillis = systemTimeMillis
            TelemetryLogger.startIteration()
            teleopPeriodic()
            TelemetryLogger.endIteration()
            var lastEndMillis = systemTimeMillis

            // TODO see if this is still an issue - came up before : if(lastEndMillis - lastStartMillis > 20)

            var nextStartMillis : Double = lastStartMillis

            while(nextStartMillis < lastEndMillis || (nextStartMillis.toLong() % 20) != 0L) {
                nextStartMillis += 1
            }

            while(systemTimeMillis < nextStartMillis) {
                delay(1)
            }
        }
    }

    override fun test() {
        runTest()
    }
}
