/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright 2019, Green Hope Falcons
 */

package org.ghrobotics.frc2019.vision

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import org.ghrobotics.frc2019.subsystems.Drivetrain
import org.ghrobotics.lib.debug.LiveDashboard
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.seconds

object TargetTracker {

  private val kTargetTrackingMaxLifetime = 0.5.seconds
  private val kTargetTrackingMinLifetime = 0.1.seconds
  private val kTargetTrackingDistanceErrorTolerance = 16.inches

  private val targets = mutableListOf<TrackedTarget>()

  fun update() {
    synchronized(targets) {
      val currentTime = Timer.getFPGATimestamp()
      val currentRobotPose = Drivetrain.localization()

      // Update and remove old targets
      targets.removeIf {
        it.update(currentTime, currentRobotPose)
        !it.isAlive
      }

      // Publish to dashboard
      LiveDashboard.visionTargets = targets.asSequence()
        .filter { it.isReal }
        .map { it.averagePose }
        .toList()
    }
  }

  fun addSamples(creationTime: Double, samples: Iterable<Pose2d>) {
    // Cannot predict the future
    if (creationTime >= Timer.getFPGATimestamp()) return

    synchronized(targets) {
      for (samplePose in samples) {
        val closestTarget = targets.minBy {
          it.averagePose.translation.getDistance(samplePose.translation)
        }
        val sample = TrackedTargetSample(creationTime, samplePose)
        if (closestTarget == null || closestTarget.averagePose.translation
            .getDistance(samplePose.translation) >
          kTargetTrackingDistanceErrorTolerance.value
        ) {
          // Create new target if no targets are within tolerance
          targets += TrackedTarget(sample)
        } else {
          // Add sample to target within tolerance
          closestTarget.addSample(sample)
        }
      }
    }
  }

  fun getBestTarget(isFront: Boolean) = synchronized(targets) {
    targets.asSequence()
      .filter {
        if (!it.isReal) return@filter false
        val x = it.averagePoseRelativeToRobot.translation.x
        if (isFront) x > 0.0 else x < 0.0
      }.minBy { it.averagePoseRelativeToRobot.translation.norm }
  }

  fun getTargetUsingReference(referencePose: Pose2d, isFront: Boolean) =
    synchronized(targets) {
      targets.asSequence()
        .associateWith { it.averagePose.relativeTo(referencePose) }
        .filter {
          val x = it.value.translation.x
          it.key.isReal && if (isFront) x > 0.0 else x < 0.0
        }.minBy { it.value.translation.norm }?.key
    }

  fun getAbsoluteTarget(translation: Translation2d) = synchronized(targets) {
    targets.asSequence()
      .filter {
        it.isReal &&
          translation.getDistance(it.averagePose.translation) <=
          kTargetTrackingDistanceErrorTolerance.value
      }.minBy { it.averagePose.translation.getDistance(translation) }
  }

  class TrackedTarget(initialTargetSample: TrackedTargetSample) {
    private val samples = mutableListOf<TrackedTargetSample>()

    var averagePose: Pose2d = initialTargetSample.pose
      private set

    var averagePoseRelativeToRobot: Pose2d = Pose2d()
      private set

    var isAlive: Boolean = true
      private set

    var isReal: Boolean = false
      private set

    init {
      addSample(initialTargetSample)
    }

    fun addSample(newSample: TrackedTargetSample) {
      samples.add(newSample)
    }

    fun update(currentTime: Double, currentRobotPose: Pose2d) {
      // Remove expired samples
      samples.removeIf {
        currentTime - it.creationTime >=
          kTargetTrackingMaxLifetime.value
      }

      // Update state
      isAlive = samples.isNotEmpty()
      if (samples.size >= 2) isReal = true

      // Update average pose
      var accumulatedX = 0.0
      var accumulatedY = 0.0
      var accumulatedAngle = 0.0

      for (sample in samples) {
        accumulatedX += sample.pose.translation.x
        accumulatedY += sample.pose.translation.y
        accumulatedAngle += sample.pose.rotation.radians
      }
      averagePose = Pose2d(
        accumulatedX / samples.size,
        accumulatedY / samples.size,
        Rotation2d(accumulatedAngle / samples.size)
      )
      averagePoseRelativeToRobot = averagePose.relativeTo(currentRobotPose)
    }
  }

  data class TrackedTargetSample(val creationTime: Double, val pose: Pose2d)
}
