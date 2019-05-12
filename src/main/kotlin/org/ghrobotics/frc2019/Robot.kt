package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.livewindow.LiveWindow
import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.subsystems.arm.Arm
import org.ghrobotics.frc2019.subsystems.climb.Understructure
import org.ghrobotics.frc2019.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2019.subsystems.elevator.Elevator
import org.ghrobotics.frc2019.subsystems.intake.Intake
import org.ghrobotics.frc2019.vision.JeVoisManager
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.subsystems.EmergencyHandleable
import org.ghrobotics.lib.wrappers.FalconRobot

object Robot : FalconRobot() {

    val emergencyReadySystems = ArrayList<EmergencyHandleable>()
    var emergencyActive = false

    init {
        LiveWindow.disableAllTelemetry()

        +Drivetrain
        +Elevator
        +Arm
        +Intake

        JeVoisManager
    }

    override fun periodic() {
        Autonomous.update()
        TargetTracker.update()
        Controls.update()
        LEDs.update()
        Network.update()
        Understructure.update()
    }

    override operator fun FalconSubsystem.unaryPlus() {
        addToSubsystemHandler(this)
        if (this is EmergencyHandleable) {
            emergencyReadySystems.add(this)
        }
    }
}