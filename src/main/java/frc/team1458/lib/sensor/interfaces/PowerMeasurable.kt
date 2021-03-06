package frc.team1458.lib.sensor.interfaces

interface PowerMeasurable {
    /**
     * Current draw of this device, in Amps
     */
    val currentDraw : Double

    companion object {
        fun create(current: () -> Double) : PowerMeasurable {
            return object : PowerMeasurable {
                override val currentDraw: Double
                    get() = current()
            }
        }
    }
}