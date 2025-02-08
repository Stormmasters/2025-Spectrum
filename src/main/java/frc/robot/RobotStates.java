package frc.robot;

import static frc.robot.auton.Auton.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.robot.swerve.Swerve;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumState;

public class RobotStates {
    private static final Pilot pilot = Robot.getPilot();
    private static final Operator operator = Robot.getOperator();
    private static final Swerve swerve = Robot.getSwerve();

    /**
     * Define Robot States here and how they can be triggered States should be triggers that command
     * multiple mechanism or can be used in teleop or auton Use onTrue/whileTrue to run a command
     * when entering the state Use onFalse/whileFalse to run a command when leaving the state
     * RobotType Triggers
     */
    public static final Trigger am = new Trigger(() -> Rio.id == Rio.AM_2025);

    public static final Trigger fm = new Trigger(() -> Rio.id == Rio.FM_2024);
    public static final Trigger sim = new Trigger(RobotBase::isSimulation);

    // intake Triggers
    public static final Trigger visionIntaking = Trigger.kFalse;
    public static final Trigger intaking = pilot.intake_A.or(visionIntaking, autonSourceIntake);
    public static final Trigger ejecting = pilot.eject_fA;

    // score Triggers
    public static final Trigger score = pilot.score_RB.or(autonScore);

    // climb Triggers
    public static final Trigger climbPrep = pilot.climbPrep_RDP;
    public static final Trigger climbFinish = pilot.climbRoutine_start;

    // mechanism preset Triggers (Wrist, Elevator, etc.)
    public static final Trigger processorScore = operator.lollipopProcessor_A;
    public static final Trigger L2Algae = operator.L2Algae_B.or(autonLowAlgae);
    public static final Trigger L3Algae = operator.L3Algae_X.or(autonHighAlgae);
    public static final Trigger barge = operator.barge_Y;

    public static final Trigger L1Coral = operator.L1Coral_A;
    public static final Trigger L2Coral = operator.L2Coral_B;
    public static final Trigger L3Coral = operator.L3Coral_X;
    public static final Trigger L4Coral = operator.L4Coral_Y.or(autonL4);

    public static final Trigger homeAll = operator.homeState;

    //reset triggers
    public static final Trigger homeElevator_A = operator.homeElevator_A;
    public static final Trigger homeInClimb_B = operator.homeInClimb_B;

    // Robot States
    // These are states that aren't directly tied to hardware or buttons, etc.
    // If they should be set by multiple Triggers do that in SetupStates()
    public static final SpectrumState coastMode = new SpectrumState("coast");
    public static final SpectrumState leftScore = new SpectrumState("leftScore");
    public static final SpectrumState rightScore = new SpectrumState("rightScore");
    public static final Trigger coastOn = pilot.coastOn_dB;

    // Setup any binding to set states
    public static void setupStates() {
        pilot.coastOn_dB.onTrue(coastMode.setTrue().ignoringDisable(true));
        pilot.coastOff_dA.onTrue(coastMode.setFalse().ignoringDisable(true));

        operator.leftScore_Dpad
                .or(pilot.L3Algae_A)
                .onTrue(leftScore.setTrue(), rightScore.setFalse());
        operator.rightScore_Dpad
                .or(pilot.L2Coral_B)
                .onTrue(rightScore.setTrue(), leftScore.setFalse());
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
