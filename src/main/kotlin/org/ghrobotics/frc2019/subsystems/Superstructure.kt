package org.ghrobotics.frc2019.subsystems

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.arm.Arm
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.elevator.ClosedLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.elevator.Elevator
import org.ghrobotics.frc2019.subsystems.intake.IntakeCloseCommand
import org.ghrobotics.lib.commands.*
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch

object Superstructure {

    private val outOfToleranceRange =
        (90.degree - Constants.kArmFlipTolerance)..(90.degree + Constants.kArmFlipTolerance)

    val kFrontHighRocketHatch
        get() = goToHeightWithAngle(81.inch, 15.degree)
    val kFrontMiddleRocketHatch
        get() = goToHeightWithAngle(50.inch, 5.degree)

    val kFrontHighRocketCargo
        get() = goToHeightWithAngle(85.inch, 25.degree)
    val kFrontMiddleRocketCargo
        get() = goToHeightWithAngle(58.5.inch, 15.degree)
    val kFrontLowRocketCargo
        get() = goToHeightWithAngle(26.5.inch, 15.degree)
    val kFrontHatchFromLoadingStation
        get() = elevatorAndArmHeight(1.inch, 10.degree)
    val kBackHatchFromLoadingStation
        get() = elevatorAndArmHeight(1.inch, 177.degree)

    val kFrontCargoIntake
        get() = elevatorAndArmHeight(0.inch, (-25).degree)
    val kBackCargoIntake
        get() = elevatorAndArmHeight(0.inch, (-155).degree)

    val kFrontCargoIntoCargoShip
        get() = elevatorAndArmHeight(25.inch, 5.degree)
    val kBackCargoFromLoadingStation
        get() = elevatorAndArmHeight(0.inch, 135.degree)

    val kStowedPosition
        get() = elevatorAndArmHeight(0.inch, 90.degree)


    private fun goToHeightWithAngle(
        heightAboveGround: Length,
        armAngle: Rotation2d
    ): FalconCommand {

        // Calculates the wanted elevator height.
        val elevatorHeightWanted =
            (heightAboveGround - Constants.kElevatorHeightFromGround - Constants.kElevatorSecondStageToArmShaft -
                (Constants.kArmLength * armAngle.sin)).coerceIn(0.inch, Constants.kMaxElevatorHeightFromZero)

        return elevatorAndArmHeight(elevatorHeightWanted, armAngle)
    }

    private fun elevatorAndArmHeight(
        elevatorHeightWanted: Length,
        armAngle: Rotation2d
    ): FalconCommand {

        // Values that store the side of the robot the arm is currently in and the side of the robot that the arm
        // wants to be in.
        val isFrontWanted = armAngle.cos > 0

        // Check if the configuration is valid.
        return sequential {

            // Flip arm vs. don't flip arm.
            +ConditionalCommand(
                { isFrontWanted != Arm.angle.cos > 0 || Arm.angle in outOfToleranceRange },

                // We now need to flip the arm
                sequential {
                    val elevatorLimit = (-2).inch

                    +parallel {
                        +IntakeCloseCommand()

                        +sequential {
                            val elevatorWaitCondition = {
                                if (isFrontWanted) {
                                    Arm.angle <=
                                        90.degree - Constants.kArmFlipTolerance + Constants.kArmClosedLoopTolerance &&
                                        Arm.angle.cos > 0
                                } else {
                                    Arm.angle >=
                                        90.degree + Constants.kArmFlipTolerance - Constants.kArmClosedLoopTolerance &&
                                        Arm.angle.cos < 0
                                }
                            }
                            +sequential {
                                +ClosedLoopElevatorCommand((-5).inch).overrideExit { Elevator.isZeroed }
                                +ClosedLoopElevatorCommand(elevatorLimit)
                            }.overrideExit(elevatorWaitCondition)

                            +ConditionCommand(elevatorWaitCondition)
                            +ClosedLoopElevatorCommand(elevatorHeightWanted)
                        }

                        +sequential {
                            val waitCondition = {
                                Elevator.height < Constants.kElevatorSafeFlipHeight
                                    || Elevator.isZeroed
                            }

                            +ClosedLoopArmCommand(
                                if (isFrontWanted) {
                                    90.degree + Constants.kArmFlipTolerance
                                } else {
                                    90.degree - Constants.kArmFlipTolerance
                                }
                            ).overrideExit(waitCondition)

                            +ConditionCommand(waitCondition)
                            +ClosedLoopArmCommand(armAngle)
                        }
                    }
                },

                parallel {
                    +ClosedLoopElevatorCommand(elevatorHeightWanted)
                    +ClosedLoopArmCommand(armAngle)
                }
            )
        }
    }
}