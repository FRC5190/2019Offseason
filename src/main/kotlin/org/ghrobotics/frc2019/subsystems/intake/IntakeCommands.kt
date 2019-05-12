package org.ghrobotics.frc2019.subsystems.intake

import org.ghrobotics.lib.commands.FalconCommand

class IntakeHatchCommand(private val releasing: Boolean) : FalconCommand(Intake) {
    override suspend fun initialize() {
        if (releasing) {
            Intake.setExtensionSolenoid(true)
            Intake.setOpenLoop(1.0)
        } else {
            Intake.setExtensionSolenoid(false)
            Intake.setOpenLoop(-1.0)
        }
    }

    override suspend fun dispose() {
        if (releasing) {
            Intake.setExtensionSolenoid(false)
        }
        Intake.zeroOutputs()
    }
}

class IntakeCargoCommand(private val releasing: Boolean) : FalconCommand(Intake) {
    private var sensedBall = 0L

    init {
        if (!releasing) {
            finishCondition += { sensedBall != 0L && System.currentTimeMillis() - sensedBall > 1000 }
        }
    }

    private var startTime = 0L

    override suspend fun initialize() {
        sensedBall = 0L
        startTime = System.currentTimeMillis()

        Intake.setLauncherSolenoid(false)

        if (releasing) {
            Intake.setOpenLoop(-1.0)
            Intake.setExtensionSolenoid(false)
        } else {
            Intake.setExtensionSolenoid(true)
            Intake.setOpenLoop(1.0)
        }
    }

    override suspend fun execute() {
        when (releasing) {
            true -> if (System.currentTimeMillis() - startTime > 125 && !Intake.launcherSolenoidState) {
                Intake.setLauncherSolenoid(true)
            }
            false -> if (Intake.isHoldingCargo && sensedBall == 0L && System.currentTimeMillis() - startTime > 500) {
                Intake.setExtensionSolenoid(false)
                sensedBall = System.currentTimeMillis()
            }
        }
    }

    override suspend fun dispose() {
        Intake.setLauncherSolenoid(false)
        Intake.setExtensionSolenoid(false)
        Intake.zeroOutputs()
    }
}

class IntakeCloseCommand : FalconCommand(Intake) {
    init {
        finishCondition += { true }
    }

    override suspend fun initialize() {
        Intake.setExtensionSolenoid(false)
        Intake.setLauncherSolenoid(false)
    }
}