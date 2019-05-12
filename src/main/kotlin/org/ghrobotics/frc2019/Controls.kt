package org.ghrobotics.frc2019

import org.ghrobotics.frc2019.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kA
import org.ghrobotics.lib.wrappers.hid.xboxController

object Controls {
    val driverFalconXbox = xboxController(0) {
        button(kA).changeOn { Drivetrain.lowGear = true }
        button(kA).changeOff { Drivetrain.lowGear = false }
    }
}