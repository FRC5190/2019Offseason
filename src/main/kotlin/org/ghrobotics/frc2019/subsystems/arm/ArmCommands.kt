package org.ghrobotics.frc2019.subsystems.arm

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.utils.DoubleSource

class OpenLoopArmCommand(private val percentSource: DoubleSource) : FalconCommand(Arm) {
    override suspend fun execute() {
        Arm.setOpenLoop(percentSource())
    }

    override suspend fun dispose() {
        Arm.setNeutral()
    }
}

class ClosedLoopArmCommand(private val angle: Rotation2d) : FalconCommand(Arm) {
    init {
        finishCondition += {
            (Arm.angle - angle).absoluteValue < Constants.kArmClosedLoopTolerance &&
                Arm.velocity.absoluteValue < Constants.kArmClosedLoopVelocityTolerance
        }
    }

    override suspend fun initialize() {
        Arm.setAngle(angle)
    }
}

class DefaultArmCommand : FalconCommand(Arm) {
    override suspend fun initialize() {
        Arm.setAngle(Arm.angle)
    }
}