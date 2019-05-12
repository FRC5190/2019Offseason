package org.ghrobotics.frc2019.subsystems.climb

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getY

class TeleopHABDriverCommand : FalconCommand(HABDriver) {
    override suspend fun execute() {
        if (Controls.isClimbing) {
            HABDriver.setOpenLoop(wheelSource())
        } else {
            HABDriver.zeroOutputs()
        }
    }

    private companion object {
        val wheelSource = Controls.driverController.getY(GenericHID.Hand.kRight).withDeadband(0.1)
    }
}