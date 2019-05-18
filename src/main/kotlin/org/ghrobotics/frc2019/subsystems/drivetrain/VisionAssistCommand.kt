package org.ghrobotics.frc2019.subsystems.drivetrain

import org.ghrobotics.frc2019.subsystems.drivetrain.Drivetrain.tankDrive
import org.ghrobotics.frc2019.subsystems.intake.Intake
import org.ghrobotics.frc2019.vision.TargetTracker
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.Rotation2d
import org.ghrobotics.lib.mathematics.units.radian
import kotlin.math.absoluteValue

class VisionAssistCommand(private val targetSide: TargetSide) : TeleopDriveCommand() {

    private var referencePose = Pose2d()
    private var lastKnownTargetPose: Pose2d? = null

    private var prevError = 0.0

    override suspend fun initialize() {
        isActive = true
        referencePose = Drivetrain.robotPosition
    }

    override suspend fun execute() {
        val newTarget = TargetTracker.getBestTargetUsingReference(referencePose, targetSide == TargetSide.Front)

        val newPose = newTarget?.averagedPose2d
        if (newTarget?.isAlive == true && newPose != null) lastKnownTargetPose = newPose

        val lastKnownTargetPose = this.lastKnownTargetPose

        val source = -speedSource()

        if (lastKnownTargetPose == null) {
            super.execute()
        } else {
            val transform = lastKnownTargetPose inFrameOfReferenceOf Intake.robotPositionWithIntakeOffset
            val angle = Rotation2d(transform.translation.x, transform.translation.y, true)

            val angleError = angle + if (targetSide == TargetSide.Front) Rotation2d.kZero else Math.PI.radian

            if (angleError.degree.absoluteValue > 45) {
                this.lastKnownTargetPose = null
            }

            val error = angleError.radian

            val turn = kCorrectionKp * error + kCorrectionKd * (error - prevError)
            tankDrive(source - turn, source + turn)

            prevError = error
        }
    }


    enum class TargetSide { Front, Back }

    companion object {
        const val kCorrectionKp = 0.8
        const val kCorrectionKd = 8.0
        var isActive = false
            private set
    }
}