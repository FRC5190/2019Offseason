/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.auto

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.subsystems.Drivetrain
import org.ghrobotics.lib.mathematics.twodim.geometry.Transform2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.mirror
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.Trajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.*
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.operations.div

object Paths {

  private val kMaxVelocity = 12.feet / 1.seconds
  private val kMaxAcceleration = 6.feet / 1.seconds / 1.seconds
  private val kMaxHabitatVelocity = 3.feet / 1.seconds
  private val kFirstPathMaxAcceleration = 6.feet / 1.seconds / 1.seconds

  private val kVelocityRadiusConstraintRadius = 3.feet
  private val kVelocityRadiusConstraintVelocity = 3.feet / 1.seconds

  private val kMaxCentripetalAccelerationElevatorUp =
    6.feet / 1.seconds / 1.seconds
  private val kMaxCentripetalAccelerationElevatorDown =
    9.feet / 1.seconds / 1.seconds

  private val kMaxVoltage = 10.volts

  /** Adjusted Poses **/

  private val cargoShipFLAdjusted = Waypoints.Waypoint(
    trueLocation = Waypoints.kCargoShipFL,
    transform = Constants.kForwardIntakeToCenter
  )
  private val cargoShipFRAdjusted = Waypoints.Waypoint(
    trueLocation = Waypoints.kCargoShipFR,
    transform = Constants.kForwardIntakeToCenter,
    translationalOffset = Translation2d(0.inches, 5.inches)
  )
  private val cargoShipS1Adjusted = Waypoints.Waypoint(
    trueLocation = Waypoints.kCargoShipS1,
    transform = Constants.kForwardIntakeToCenter,
    translationalOffset = Translation2d(1.9.inches, 0.inches)
  )
  private val cargoShipS2Adjusted = Waypoints.Waypoint(
    trueLocation = Waypoints.kCargoShipS2,
    transform = Constants.kForwardIntakeToCenter,
    translationalOffset = Translation2d(1.9.inches, 1.5.inches)
  )
  private val cargoShipS3Adjusted = Waypoints.Waypoint(
    trueLocation = Waypoints.kCargoShipS3,
    transform = Constants.kForwardIntakeToCenter
  )
  private val depotAdjusted = Waypoints.Waypoint(
    trueLocation = Waypoints.kDepotBRCorner,
    transform = Constants.kBackwardIntakeToCenter
  )
  private val loadingStationAdjusted = Waypoints.Waypoint(
    trueLocation = Waypoints.kLoadingStation,
    transform = Constants.kBackwardIntakeToCenter,
    translationalOffset = Translation2d((-9).inches, 0.inches)
  )
  private val rocketFAdjusted = Waypoints.Waypoint(
    trueLocation = Waypoints.kRocketF,
    transform = Constants.kForwardIntakeToCenter,
    translationalOffset = Translation2d(0.inches, -4.inches)
  )
  private val rocketNAdjusted = Waypoints.Waypoint(
    trueLocation = Waypoints.kRocketN,
    transform = Constants.kForwardIntakeToCenter
  )

  /** Trajectories **/

  val cargoShipFLToRightLoadingStation = generateTrajectory(
    true,
    listOf(
      cargoShipFLAdjusted,
      cargoShipFLAdjusted.position.transformBy(
        Transform2d(
          (-0.7).feet, 0.feet, Rotation2d()
        )
      ).asWaypoint(),
      Pose2d(10.6.feet, 6.614.feet, 69.degrees).asWaypoint(),
      loadingStationAdjusted
    ),
    getConstraints(false, loadingStationAdjusted),
    8.feet.velocity,
    6.feet.acceleration,
    kMaxVoltage
  )

  val cargoShipFLToLeftLoadingStation = generateTrajectory(
    true,
    listOf(
      cargoShipFLAdjusted,
      cargoShipFLAdjusted.position.transformBy(
        Transform2d((-0.7).feet, 0.feet, Rotation2d())
      ).asWaypoint(),
      Pose2d(10.6.feet, 6.614.feet, 69.degrees).mirror().asWaypoint(),
      loadingStationAdjusted.position.mirror().asWaypoint()
    ),
    getConstraints(false, loadingStationAdjusted),
    8.feet.velocity,
    6.feet.acceleration,
    kMaxVoltage
  )

  val cargoShipFRToRightLoadingStation =
    cargoShipFLToLeftLoadingStation.mirror()

  val cargoShipS1ToDepot = generateTrajectory(
    true,
    listOf(
      cargoShipS1Adjusted,
      Pose2d(15.feet, 4.951.feet, 17.degrees).asWaypoint(),
      depotAdjusted
    ),
    getConstraints(false, depotAdjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val cargoShipS1ToLoadingStation = generateTrajectory(
    true,
    listOf(
      cargoShipS1Adjusted,
      Pose2d(15.feet, 4.951.feet, 17.degrees).asWaypoint(),
      loadingStationAdjusted
    ),
    getConstraints(false, loadingStationAdjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val centerStartToCargoShipFL = generateTrajectory(
    false,
    listOf(
      Waypoints.kCenterStart.asWaypoint(),
      cargoShipFLAdjusted
    ),
    getConstraints(false, cargoShipFLAdjusted),
    kMaxVelocity,
    4.feet.acceleration,
    kMaxVoltage
  )

  val centerStartToCargoShipFR = centerStartToCargoShipFL.mirror()

  val depotToCargoShipS2 = generateTrajectory(
    false,
    listOf(
      depotAdjusted,
      Pose2d(15.feet, 4.951.feet, 17.degrees).asWaypoint(),
      cargoShipS2Adjusted
    ),
    getConstraints(false, cargoShipS2Adjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val loadingStationToCargoShipFR = generateTrajectory(
    false,
    listOf(
      loadingStationAdjusted,
      Pose2d(10.6.feet, 6.614.feet, 69.degrees).asWaypoint(),
      cargoShipFRAdjusted.position.transformBy(
        Transform2d(
          (-30).inches, 0.inches, Rotation2d()
        )
      ).asWaypoint(),
      cargoShipFRAdjusted
    ),
    getConstraints(false, cargoShipFRAdjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val loadingStationToCargoShipS2 = generateTrajectory(
    false,
    listOf(
      loadingStationAdjusted,
      Pose2d(15.feet, 4.951.feet, 17.degrees).asWaypoint(),
      cargoShipS2Adjusted
    ),
    getConstraints(false, cargoShipS2Adjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val loadingStationToRocketF = generateTrajectory(
    false,
    listOf(
      loadingStationAdjusted,
      Pose2d(17.039.feet, 6.378.feet, 9.degrees).asWaypoint(),
      rocketFAdjusted
    ),
    getConstraints(true, rocketFAdjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val loadingStationToRocketN = generateTrajectory(
    false,
    listOf(
      loadingStationAdjusted,
      rocketNAdjusted
    ),
    getConstraints(true, rocketNAdjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val rocketNToDepot = generateTrajectory(
    true,
    listOf(
      rocketNAdjusted,
      depotAdjusted
    ),
    getConstraints(false, depotAdjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val rocketFPrepareToRocketF = generateTrajectory(
    false,
    listOf(
      Pose2d(24.074.feet, 3.753.feet, -143.degrees).asWaypoint(),
      rocketFAdjusted.position.transformBy(
        Transform2d(
          -7.inches, 0.inches, Rotation2d()
        )
      ).asWaypoint()
    ),
    getConstraints(false, Pose2d()),
    3.feet.velocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val rocketFToDepot = generateTrajectory(
    true,
    listOf(
      rocketFAdjusted,
      Pose2d(19.216.feet, 5.345.feet, 5.degrees).asWaypoint(),
      depotAdjusted
    ),
    getConstraints(false, depotAdjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val rocketFToLoadingStation = generateTrajectory(
    true,
    listOf(
      rocketFAdjusted,
      Pose2d(19.216.feet, 5.345.feet, 5.degrees).asWaypoint(),
      loadingStationAdjusted
    ),
    getConstraints(false, loadingStationAdjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val rocketNToLoadingStation = generateTrajectory(
    true,
    listOf(
      rocketNAdjusted,
      loadingStationAdjusted
    ),
    getConstraints(false, loadingStationAdjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val sideStartToCargoShipS1 = generateTrajectory(
    false,
    listOf(
      Waypoints.kSideStart.asWaypoint(),
      cargoShipS1Adjusted
    ),
    getConstraints(true, cargoShipS1Adjusted),
    kMaxVelocity,
    kFirstPathMaxAcceleration,
    kMaxVoltage
  )

  val sideStartToRocketF = generateTrajectory(
    false,
    listOf(
      Pose2d(Waypoints.kSideStart.translation, Rotation2d()).asWaypoint(),
      rocketFAdjusted
    ),
    getConstraints(false, rocketFAdjusted),
    kMaxVelocity,
    kMaxAcceleration,
    kMaxVoltage
  )

  val sideStartReversedToRocketFPrepare = generateTrajectory(
    true,
    listOf(
      Waypoints.kSideStartReversed.asWaypoint(),
      Pose2d(15.214.feet, 8.7.feet, 165.degrees).asWaypoint(),
      Pose2d(22.488.feet, 5.639.feet, 143.degrees).asWaypoint(),
      Pose2d(24.074.feet, 3.753.feet, -143.degrees).asWaypoint()
    ),
    getConstraints(false, Pose2d()),
    kMaxVelocity,
    7.feet.acceleration,
    kMaxVoltage
  )

  private fun getConstraints(elevatorUp: Boolean, trajectoryEndpoint: Pose2d) =
    listOf(
      CentripetalAccelerationConstraint(
        if (elevatorUp) {
          kMaxCentripetalAccelerationElevatorUp
        } else {
          kMaxCentripetalAccelerationElevatorDown
        }
      ),
      VelocityLimitRadiusConstraint(
        trajectoryEndpoint.translation,
        kVelocityRadiusConstraintRadius,
        kVelocityRadiusConstraintVelocity
      ),
      VelocityLimitRegionConstraint(
        Waypoints.kHabitatL1Platform,
        kMaxHabitatVelocity
      )
    )

  private fun getConstraints(
    elevatorUp: Boolean,
    trajectoryEndpoint: Waypoints.Waypoint
  ) =
    getConstraints(elevatorUp, trajectoryEndpoint.position)

  @Suppress("LongParameterList")
  private fun generateTrajectory(
    reversed: Boolean,
    points: List<Waypoints.Waypoint>,
    constraints: List<TrajectoryConstraint>,
    maxVelocity: SIUnit<LinearVelocity>,
    maxAcceleration: SIUnit<LinearAcceleration>,
    maxVoltage: SIUnit<Volt>
  ): Trajectory {
    val driveDynamicsConstraint = DifferentialDriveDynamicsConstraint(
      Drivetrain.differentialDrive, maxVoltage
    )
    val allConstraints = ArrayList<TrajectoryConstraint>()
    allConstraints.add(driveDynamicsConstraint)

    if (constraints.isNotEmpty()) allConstraints.addAll(constraints)

    return DefaultTrajectoryGenerator.generateTrajectory(
      points.map { it.position },
      allConstraints,
      0.inches / 1.seconds,
      0.inches / 1.seconds,
      maxVelocity,
      maxAcceleration,
      reversed
    )
  }

  private fun Pose2d.asWaypoint() = Waypoints.Waypoint(this)

  @Suppress("FunctionNaming")
  fun Pose2d(x: SIUnit<Meter>, y: SIUnit<Meter>, theta: SIUnit<Radian>) =
    Pose2d(x.value, y.value, Rotation2d(theta.value))
}
