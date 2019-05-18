package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import io.github.oblarg.oblog.Loggable
import io.github.oblarg.oblog.annotations.Log
import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.subsystems.arm.Arm
import org.ghrobotics.frc2019.subsystems.climb.Stilts
import org.ghrobotics.frc2019.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2019.subsystems.elevator.Elevator
import org.ghrobotics.frc2019.vision.JeVoisManager
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser

@Suppress("unused")
object Network : Loggable {

    override fun configureLogName() = "Main"

    val startingPositionChooser = enumSendableChooser<Autonomous.StartingPositions>()
    val autoModeChooser = enumSendableChooser<Autonomous.Mode>()

    private val mainShuffleboardDisplay: ShuffleboardTab = Shuffleboard.getTab("Main")

    private val autoLayout = mainShuffleboardDisplay.getLayout("Autonomous", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(0, 0)

    val drivetrainLeft @Log(name = "DT Left (in)", rowIndex = 0, columnIndex = 4) get() = Drivetrain.lPosition.inch
    val drivetrainRight @Log(name = "DT Right(in)", rowIndex = 1, columnIndex = 4) get() = Drivetrain.rPosition.inch
    val drivetrainAngle @Log(name = "DT Angle (deg)", rowIndex = 2, columnIndex = 4) get() = Drivetrain.angle.degree

    val elevator @Log(name = "EL (in)", rowIndex = 0, columnIndex = 5) get() = Elevator.height.inch

    val arm @Log(name = "ARM (deg)", rowIndex = 1, columnIndex = 5) get() = Arm.angle.degree

    val forwardStilt @Log(name = "FST (in)", rowIndex = 0, columnIndex = 6) get() = Stilts.frontHeight.inch
    val backwardStilt @Log(name = "BST (in)", rowIndex = 1, columnIndex = 6) get() = Stilts.backHeight.inch

    val frontJeVois
        @Log(name = "Front JeVois", rowIndex = 3, columnIndex = 0) get() = JeVoisManager.isFrontJeVoisConnected

    val backJeVois
        @Log(name = "Drivetrain JeVois", rowIndex = 3, columnIndex = 1) get() = JeVoisManager.isBackJeVoisConnected

    val drivetrainJeVois
        @Log(name = "Front JeVois", rowIndex = 3, columnIndex = 2) get() = JeVoisManager.isDrivetrainJeVoisConnected


    init {
        autoLayout.add("Auto Mode", autoModeChooser)
        autoLayout.add("Starting Position", startingPositionChooser)
    }
}