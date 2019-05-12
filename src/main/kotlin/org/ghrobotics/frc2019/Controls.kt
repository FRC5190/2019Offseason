/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.GenericHID
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.OpenLoopArmCommand
import org.ghrobotics.frc2019.subsystems.climb.Understructure
import org.ghrobotics.frc2019.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2019.subsystems.elevator.OpenLoopElevatorCommand
import org.ghrobotics.frc2019.subsystems.intake.Intake
import org.ghrobotics.frc2019.subsystems.intake.IntakeCargoCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.wrappers.hid.*
import kotlin.math.pow
import kotlin.math.withSign

object Controls {

    var isClimbing = false
        private set

    val driverController = xboxController(0) {
        registerEmergencyMode()

        pov(270).changeOn { Intake.badIntakeOffset += .25.inch }
        pov(90).changeOn { Intake.badIntakeOffset -= .25.inch }

        state({ !isClimbing }) {

            //            button(kY).change(VisionDriveCommand(VisionDriveCommand.TargetSide.FRONT))
//            button(kB).change(VisionDriveCommand(VisionDriveCommand.TargetSide.BACK))

            button(kA).changeOn { Drivetrain.lowGear = true }
            button(kA).changeOff { Drivetrain.lowGear = false }

            triggerAxisButton(GenericHID.Hand.kLeft).change(IntakeHatchCommand(true))
            button(kBumperLeft).change(IntakeHatchCommand(false))

            triggerAxisButton(GenericHID.Hand.kRight).change(IntakeCargoCommand(true))
            button(kBumperRight).change(IntakeCargoCommand(false))
        }
    }

    var backModifier = false
        private set

    val operatorController = xboxController(1) {
        registerEmergencyMode()

        // Climb
        button(kB).changeOn {
            isClimbing = !isClimbing
            Superstructure.kStowedPosition.start()
        }

        state({ !isClimbing }) {

            /** MANUAL CONTROL **/
            axisButton(1, 0.1) { change(OpenLoopElevatorCommand(source.map { it.pow(2).withSign(-it) * .5 })) }
            axisButton(5, 0.1) { change(OpenLoopArmCommand(source.map { it.pow(2).withSign(-it) * .5 })) }

            /** PRESETS **/
            triggerAxisButton(GenericHID.Hand.kLeft, 0.20) {
                changeOn { backModifier = true }
                changeOff { backModifier = false }
            }

            pov(0).changeOn {
                if (Intake.isHoldingCargo) Superstructure.kFrontHighRocketCargo.start()
                else Superstructure.kFrontHighRocketHatch.start()
            }
            pov(90).changeOn {
                if (Intake.isHoldingCargo) Superstructure.kFrontMiddleRocketCargo.start()
                else Superstructure.kFrontMiddleRocketHatch.start()
            }
            pov(180).changeOn {
                when {
                    backModifier -> Superstructure.kBackHatchFromLoadingStation.start()
                    Intake.isHoldingCargo -> Superstructure.kFrontLowRocketCargo.start()
                    else -> Superstructure.kFrontHatchFromLoadingStation.start()
                }
            }
            pov(270).changeOn {
                if (backModifier) Superstructure.kBackCargoIntake.start()
                else Superstructure.kFrontCargoIntake.start()
            }
            triggerAxisButton(GenericHID.Hand.kRight).changeOn {
                if (backModifier) Superstructure.kBackCargoFromLoadingStation.start()
                else Superstructure.kFrontCargoIntoCargoShip.start()
            }
            button(kBumperRight).changeOn(Superstructure.kStowedPosition)
        }

        state({ isClimbing }) {
            button(kA).change(Understructure.autoClimb(isLevel2 = false))
            button(kY).change(Understructure.autoClimb(isLevel2 = true))
        }
    }

    private fun FalconXboxBuilder.registerEmergencyMode() {
        button(kBack).changeOn {
            Robot.emergencyReadySystems.forEach { system -> system.activateEmergency() }
            Robot.emergencyActive = true
        }
        button(kStart).changeOn {
            Robot.emergencyReadySystems.forEach { system -> system.recoverFromEmergency() }
            Robot.emergencyActive = false
        }
    }

    fun update() {
        driverController.update()
        operatorController.update()
    }
}