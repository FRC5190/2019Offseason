package org.ghrobotics.frc2019.subsystems.elevator

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.utils.DoubleSource

class OpenLoopElevatorCommand(private val percentSource: DoubleSource) : FalconCommand(Elevator) {
    override suspend fun execute() {
        Elevator.setOpenLoop(percentSource())
    }

    override suspend fun dispose() {
        Elevator.zeroOutputs()
    }
}

class ClosedLoopElevatorCommand(private val height: Length) : FalconCommand(Elevator) {
    init {
        finishCondition += {
            (Elevator.height - height).absoluteValue < Constants.kElevatorClosedLoopTolerance &&
                Elevator.velocity.absoluteValue < Constants.kElevatorClosedLoopVelocityTolerance
        }
    }

    override suspend fun initialize() {
        Elevator.setHeight(height)
    }
}

class DefaultElevatorCommand : FalconCommand(Elevator) {
    override suspend fun initialize() {
        Elevator.setHeight(Elevator.height)
    }
}