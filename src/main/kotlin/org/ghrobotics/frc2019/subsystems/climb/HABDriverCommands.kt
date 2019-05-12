package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.DoubleSource

class OpenLoopHABDriveCommand(val percentSource: DoubleSource) : FalconCommand(HABDriver) {
    override suspend fun execute() {
        HABDriver.setOpenLoop(percentSource())
    }

    override suspend fun dispose() {
        HABDriver.zeroOutputs()
    }
}