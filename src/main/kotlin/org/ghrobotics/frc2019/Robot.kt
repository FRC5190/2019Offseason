package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.livewindow.LiveWindow
import org.ghrobotics.frc2019.subsystems.Drivetrain
import org.ghrobotics.lib.wrappers.FalconRobot

object Robot : FalconRobot() {

    var debugActive = true

    val shouldDebug get() = debugActive || !lastEnabledState

    init {
        LiveWindow.disableAllTelemetry()

        Drivetrain
    }

    override fun periodic() {
        Drivetrain.updateState()
    }

}