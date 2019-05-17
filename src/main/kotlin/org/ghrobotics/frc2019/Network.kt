package org.ghrobotics.frc2019

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.subsystems.arm.Arm
import org.ghrobotics.frc2019.subsystems.climb.Stilts
import org.ghrobotics.frc2019.subsystems.drivetrain.Drivetrain
import org.ghrobotics.frc2019.subsystems.elevator.Elevator
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser

object Network {
    val startingPositionChooser = enumSendableChooser<Autonomous.StartingPositions>()
    val autoModeChooser = enumSendableChooser<Autonomous.Mode>()

    private val mainShuffleboardDisplay: ShuffleboardTab = Shuffleboard.getTab("5190")

    private val autoLayout = mainShuffleboardDisplay.getLayout("Autonomous", BuiltInLayouts.kList)
        .withSize(2, 2)
        .withPosition(0, 0)

    private val essentialsLayout = mainShuffleboardDisplay.getLayout("Essential Data", BuiltInLayouts.kGrid)
        .withSize(3, 4)
        .withPosition(0, 2)

    val dtLeftDistInches = essentialsLayout.add("DT Left (inches)", 0.0).withPosition(0, 0).entry
    val dtRightDistInches = essentialsLayout.add("DT Right (inches)", 0.0).withPosition(1, 0).entry
    val dtAngle = essentialsLayout.add("DT Angle (degree)", 0.0).withPosition(2, 0).entry
    val elHeightInches = essentialsLayout.add("Elevator (inches)", 0.0).withPosition(0, 1).entry
    val armAngleDegrees = essentialsLayout.add("Arm (degrees)", 0.0).withPosition(0, 2).entry
    val fStiltInches = essentialsLayout.add("F Stilt (inches)", 0.0).withPosition(0, 3).entry
    val rStiltInches = essentialsLayout.add("R Stilt (inches)", 0.0).withPosition(1, 3).entry

    init {
        autoLayout.add("Auto Mode", autoModeChooser)
        autoLayout.add("Starting Position", startingPositionChooser)
    }

    fun update() {
        dtLeftDistInches.setDouble(Drivetrain.lPosition.inch)
        dtRightDistInches.setDouble(Drivetrain.rPosition.inch)
        dtAngle.setDouble(Drivetrain.angle.degree)
        elHeightInches.setDouble(Elevator.height.inch)
        armAngleDegrees.setDouble(Arm.angle.degree)
        fStiltInches.setDouble(Stilts.frontHeight.inch)
        rStiltInches.setDouble(Stilts.backHeight.inch)
    }
}