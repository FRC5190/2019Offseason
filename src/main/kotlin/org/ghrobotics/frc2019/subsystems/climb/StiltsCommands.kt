package org.ghrobotics.frc2019.subsystems.climb

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length

class ClosedLoopStiltsCommand(private val frontTarget: Length, private val backTarget: Length) : FalconCommand(Stilts) {
    init {
        finishCondition += {
            (Stilts.frontHeight - frontTarget).absoluteValue < Constants.kClimbWinchClosedLoopTolerance &&
                (Stilts.backHeight - backTarget).absoluteValue < Constants.kClimbWinchClosedLoopTolerance
        }
    }

    override suspend fun initialize() {
        Stilts.setAutoClimbHeight(frontTarget, backTarget)
    }
}

class ResetStiltsCommand(private val resetFront: Boolean) : FalconCommand(Stilts) {
    init {
        finishCondition += if (resetFront) Stilts::frontLimitSwitchEngaged else Stilts::backLimitSwitchEngaged
    }

    override suspend fun initialize() {
        val frontOutput = if (resetFront) -1.0 else 0.0
        val backOutput = if (resetFront) 0.0 else -1.0

        Stilts.setOpenLoop(frontOutput, backOutput)
    }
}