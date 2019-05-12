package org.ghrobotics.frc2019.subsystems.drivetrain

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.Controls
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getRawButton
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import org.ghrobotics.lib.wrappers.hid.kX

open class TeleopDriveCommand : FalconCommand(Drivetrain) {
    override suspend fun execute() {
        val curvature = rotationSource()
        val linear = -speedSource()

        Drivetrain.curvatureDrive(linear, curvature, quickTurnSource())
    }

    companion object {
        private const val kDeadband = 0.05
        val speedSource by lazy { Controls.driverController.getY(GenericHID.Hand.kLeft).withDeadband(kDeadband) }
        val rotationSource by lazy { Controls.driverController.getX(GenericHID.Hand.kLeft).withDeadband(kDeadband) }
        val quickTurnSource by lazy { Controls.driverController.getRawButton(kX) }
    }
}