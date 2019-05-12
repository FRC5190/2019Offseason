package org.ghrobotics.frc2019.auto

import org.ghrobotics.frc2019.Network
import org.ghrobotics.frc2019.Robot
import org.ghrobotics.frc2019.auto.paths.TrajectoryWaypoints
import org.ghrobotics.frc2019.auto.routines.BottomRocketRoutine
import org.ghrobotics.frc2019.auto.routines.CargoShipRoutine
import org.ghrobotics.frc2019.auto.routines.HybridRoutine
import org.ghrobotics.frc2019.auto.routines.TestTrajectoriesRoutine
import org.ghrobotics.frc2019.subsystems.drivetrain.Drivetrain
import org.ghrobotics.lib.commands.S3ND
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.commands.stateCommandGroup
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.and
import org.ghrobotics.lib.utils.monitor
import org.ghrobotics.lib.utils.onChangeToTrue
import org.ghrobotics.lib.wrappers.FalconRobot

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous {

    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    // Starting position of the robot
    val startingPosition = { Network.startingPositionChooser.selected }

    // Stores whether the current config is valid.
    private var configValid = Source(true)

    // Stores if we are ready to send it.
    private val isReady =
        { Robot.lastRobotMode == FalconRobot.Mode.AUTONOMOUS && Robot.lastEnabledState } and configValid

    // Update the autonomous listener.
    fun update() {
        // Update localization.
        startingPositionMonitor.onChange { if (!Robot.lastEnabledState) Drivetrain.localization.reset(it.pose) }

        modeMonitor.onChange { newValue ->
            if (newValue != FalconRobot.Mode.AUTONOMOUS) JUST.stop()
        }

        isReadyMonitor.onChangeToTrue {
            JUST S3ND IT
        }
    }

    // Autonomous Master Group
    private val JUST = stateCommandGroup(startingPosition) {
        state(StartingPositions.Left, StartingPositions.Right) {
            stateCommandGroup(autoMode) {
                state(Mode.TestTrajectories, TestTrajectoriesRoutine())
                state(Mode.ForwardCargoShip, sequential {})
                state(Mode.DoNothing, sequential {})
                state(Mode.BottomRocket, BottomRocketRoutine()())
                state(Mode.SideCargoShip, CargoShipRoutine(CargoShipRoutine.Mode.SIDE)())
                state(Mode.HybridLeft, sequential {})
                state(Mode.HybridRight, sequential {})
            }
        }
        state(StartingPositions.Center) {
            stateCommandGroup(autoMode) {
                state(Mode.ForwardCargoShip, CargoShipRoutine(CargoShipRoutine.Mode.FRONT)())
                state(Mode.TestTrajectories, TestTrajectoriesRoutine())
                state(Mode.BottomRocket, sequential {})
                state(Mode.SideCargoShip, sequential {})
                state(Mode.HybridLeft, HybridRoutine(HybridRoutine.Mode.LEFT))
                state(Mode.HybridRight, HybridRoutine(HybridRoutine.Mode.RIGHT))
            }
        }
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = startingPosition.monitor
    private val isReadyMonitor = isReady.monitor
    private val modeMonitor = { Robot.lastRobotMode }.monitor


    enum class StartingPositions(val pose: Pose2d) {
        Left(TrajectoryWaypoints.kSideStart.mirror),
        Center(TrajectoryWaypoints.kCenterStart),
        Right(TrajectoryWaypoints.kSideStart)
    }

    enum class Mode { TestTrajectories, BottomRocket, ForwardCargoShip, SideCargoShip, HybridLeft, HybridRight, DoNothing }
}