package frc.team1458.robot

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance

object VisionTable {
    var camera : NetworkTableEntry? = null
    var visionEnable : NetworkTableEntry? = null
    var visionReady : NetworkTableEntry? = null

    fun setup() {
        val table = NetworkTableInstance.getDefault().getTable("VisionTable")
        camera = table.getEntry("current_camera")
        visionEnable = table.getEntry("vision_enabled")
        visionReady = table.getEntry("vision_ready")


    }

}