package org.ghrobotics.frc2019.subsystems.climb

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.absoluteValue

class TeleopStiltsCommand : FalconCommand(Stilts) {
    override suspend fun execute() {

        val frontSource = frontStiltsSource()
        val backSource = backStiltsSource()

        if (Controls.isClimbing) {
            if (frontSource.absoluteValue > 0.1 || backSource.absoluteValue > 0.1) {
                Stilts.setOpenLoop(-frontSource, -backSource)
            } else {
                Stilts.setMotionMagic(Stilts.frontHeight, Stilts.backHeight)
            }
        } else {
            Stilts.zeroOutputs()
        }
    }

    private companion object {
        val frontStiltsSource = Controls.operatorController.getY(GenericHID.Hand.kLeft)
        val backStiltsSource = Controls.operatorController.getY(GenericHID.Hand.kRight)
    }
}