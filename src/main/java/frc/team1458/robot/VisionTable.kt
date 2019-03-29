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

    var ll_tv : NetworkTableEntry? = null
    var ll_tx : NetworkTableEntry? = null

    var ll_mode : NetworkTableEntry? = null
    var ll_stream : NetworkTableEntry? = null
    var ll_pipeline : NetworkTableEntry? = null

    fun setup() {
        val table = NetworkTableInstance.getDefault().getTable("VisionTable")
        camera = table.getEntry("current_camera")
        visionEnable = table.getEntry("vision_enabled")
        visionReady = table.getEntry("vision_ready")

        horizOffset = table.getEntry("horiz_offset")
        angleOffset = table.getEntry("angle_offset")

        pressure = table.getEntry("pressure_psi")
        defense_timer = table.getEntry("defense_timer")

        val ll_table = NetworkTableInstance.getDefault().getTable("limelight")
        ll_tv = ll_table.getEntry("tv")
        ll_tx = ll_table.getEntry("tx")
        ll_mode = ll_table.getEntry("camMode") // 0 = vision, 1 = driver
        ll_stream = ll_table.getEntry("stream") // 1 = main, 2 = rear
        ll_pipeline = ll_table.getEntry("pipeline") // 0 = center, 1 = left, 2 = right
    }

}
