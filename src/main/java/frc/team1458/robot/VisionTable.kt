package frc.team1458.robot

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance

object VisionTable {
    var camera : NetworkTableEntry? = null
    var visionEnable : NetworkTableEntry? = null
    var visionReady : NetworkTableEntry? = null

    var horizOffset : NetworkTableEntry? = null
    var angleOffset : NetworkTableEntry? = null

    var pressure : NetworkTableEntry? = null
    var defense_timer : NetworkTableEntry? = null

    fun setup() {
        val table = NetworkTableInstance.getDefault().getTable("VisionTable")
        camera = table.getEntry("current_camera")
        visionEnable = table.getEntry("vision_enabled")
        visionReady = table.getEntry("vision_ready")

        horizOffset = table.getEntry("horiz_offset")
        angleOffset = table.getEntry("angle_offset")

        pressure = table.getEntry("pressure_psi")
        defense_timer = table.getEntry("defense_timer")
    }

}