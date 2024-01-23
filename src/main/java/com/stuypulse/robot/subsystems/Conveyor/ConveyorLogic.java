package com.stuypulse.robot.subsystems.Conveyor;

public abstract class ConveyorLogic {
    public record Input(boolean conveyorIR, boolean ampIR, boolean shooterIR, Conveyor.Direction target) {}
    public record Output(boolean intake, Conveyor.Direction conveyor, boolean feeder, boolean amp) {}

    public static final Output STOP = new Output(false, Conveyor.Direction.NONE, false, false);

    public abstract Output run(Input in);

    /**
     * Intake continuously until a note is at the desired target. If the target
     * is NONE, stop the note at the conveyor IR sensor until a target is set.
     */
    public static class Default extends ConveyorLogic {
        public Output run(Input in) {
            boolean intake = false, feeder = false, amp = false;
            Conveyor.Direction conveyor = Conveyor.Direction.NONE;

            if (in.target == Conveyor.Direction.SHOOTER && !in.shooterIR) {
                intake = true;
                conveyor = in.target;
                feeder = true;
            } else if (in.target == Conveyor.Direction.AMP && !in.ampIR) {
                intake = true;
                conveyor = in.target;
                amp = true;
            } else if (in.target == Conveyor.Direction.NONE && !in.conveyorIR) {
                intake = true;
            }

            return new Output(intake, conveyor, feeder, amp);
        }
    }

    /**
     * Intake continuously until a note is at the desired target. If a note is
     * already at the target, intake a second note until it has reached the
     * conveyor IR sensor.
     */
    public static class Intake extends ConveyorLogic {
        public Output run(Input in) {
            boolean intake = false, feeder = false, amp = false;
            Conveyor.Direction conveyor = Conveyor.Direction.NONE;

            if (in.target == Conveyor.Direction.SHOOTER && !in.shooterIR) {
                intake = true;
                conveyor = in.target;
                feeder = true;
            } else if (in.target == Conveyor.Direction.AMP && !in.ampIR) {
                intake = true;
                conveyor = in.target;
                amp = true;
            } else if (!in.conveyorIR) {
                intake = true;
            }

            return new Output(intake, conveyor, feeder, amp);
        }
    }

    /**
     * Send the note into the shooter.
     */
    public static class ShooterShoot extends ConveyorLogic {
        public Output run(Input in) {
            boolean intake = false, feeder = false, amp = false;
            Conveyor.Direction conveyor = Conveyor.Direction.NONE;

            conveyor = Conveyor.Direction.SHOOTER;
            feeder = true;

            return new Output(intake, conveyor, feeder, amp);
        }
    }

    /**
     * Stop all output motors.
     */
    public static class Stop extends ConveyorLogic {
        public Output run(Input in) {
            return STOP;
        }
    }
}
