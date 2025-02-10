package frc.robot;

import static frc.robot.auton.Auton.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.Field;
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

    // zones
    public static final Trigger topLeftZone =
            swerve.inXzoneAlliance(Field.Reef.center.getX(), Field.getHalfLength())
                    .and(swerve.inYzoneAlliance(Field.Reef.center.getY(), Field.getFieldWidth()));
    public static final Trigger topRightZone =
            swerve.inXzoneAlliance(Field.Reef.center.getX(), Field.getHalfLength())
                    .and(swerve.inYzoneAlliance(0, Field.Reef.center.getY()));
    public static final Trigger bottomLeftZone =
            swerve.inXzoneAlliance(0, Field.Reef.center.getX())
                    .and(swerve.inYzoneAlliance(Field.Reef.center.getY(), Field.getFieldWidth()));
    public static final Trigger bottomRightZone =
            swerve.inXzoneAlliance(0, Field.Reef.center.getX())
                    .and(swerve.inYzoneAlliance(0, Field.Reef.center.getY()));

    public static final Trigger bargeZone =
            swerve.inXzoneAlliance(
                            3 * Field.getHalfLength() / 4,
                            Field.getHalfLength()
                                    - Units.inchesToMeters(24)
                                    - swerve.getConfig().getRobotLength() / 2)
                    .and(topLeftZone);

    // intake Triggers
    public static final Trigger visionIntaking = Trigger.kFalse;
    public static final Trigger stationIntaking =
            pilot.stationIntake_Y
                    .or(visionIntaking, autonSourceIntake)
                    .and(bottomLeftZone.or(bottomRightZone));
    public static final Trigger ejecting = pilot.eject_fA;

    // score Triggers
    public static final Trigger score = pilot.score_RB.or(autonScore);

    // climb Triggers
    public static final Trigger climbPrep = pilot.climbPrep_RDP.or(operator.climbPrep_B);
    public static final Trigger climbFinish = pilot.climbRoutine_start.or(operator.climbFinish_B);

    // mechanism preset Triggers (Wrist, Elevator, etc.)
    public static final Trigger algaeFloorIntake = operator.algaeFloor_X.or(autonGroundIntake);
    public static final Trigger coralFloorIntake = operator.coralFloor_fX;
    public static final Trigger handOffAlgae = operator.handOffAlgae_Y;
    // potentially everything in position, then elevator goes down while spinning coralIntake
    public static final Trigger handOffCoral = operator.handOffCoral_fY;
    public static final Trigger barge = operator.barge_A.and(bargeZone);
    public static final Trigger processorScore = operator.floorScore_A;

    public static final Trigger L2Algae = pilot.L2Algae_fB.or(autonLowAlgae);
    public static final Trigger L3Algae = pilot.L3Algae_A.or(autonHighAlgae);
    public static final Trigger L1Coral = pilot.L1Coral_B;
    public static final Trigger L2Coral = pilot.L2Coral_B;
    public static final Trigger L3Coral = pilot.L3Coral_X;
    public static final Trigger L4Coral = pilot.L4Coral_fY.or(autonL4);
    public static final Trigger homeAll = pilot.home;

    // Robot States
    // These are states that aren't directly tied to hardware or buttons, etc.
    // If they should be set by multiple Triggers do that in SetupStates()
    public static final SpectrumState coastMode = new SpectrumState("coast");
    public static final Trigger coastOn = pilot.coastOn_dB;

    public static final Trigger reefPosition =
            L2Algae.or(L3Algae, L1Coral, L2Coral, L3Coral, L4Coral);

    public static final SpectrumState backwardMode = new SpectrumState("backward");

    // Setup any binding to set states
    public static void setupStates() {
        pilot.coastOn_dB.onTrue(coastMode.setTrue().ignoringDisable(true));
        pilot.coastOff_dA.onTrue(coastMode.setFalse().ignoringDisable(true));

        bottomLeftZone
                .and(
                        stationIntaking,
                        () ->
                                !swerve.frontClosestToAngle(
                                        Field.CoralStation.leftCenterFace
                                                .getRotation()
                                                .getDegrees()))
                .onTrue(backwardMode.setTrue());
        bottomLeftZone
                .and(
                        stationIntaking,
                        () ->
                                swerve.frontClosestToAngle(
                                        Field.CoralStation.leftCenterFace
                                                .getRotation()
                                                .getDegrees()))
                .onTrue(backwardMode.setFalse());

        bottomRightZone
                .and(
                        stationIntaking,
                        () ->
                                !swerve.frontClosestToAngle(
                                        Field.CoralStation.rightCenterFace
                                                .getRotation()
                                                .getDegrees()))
                .onTrue(backwardMode.setTrue());
        bottomRightZone
                .and(
                        stationIntaking,
                        () ->
                                swerve.frontClosestToAngle(
                                        Field.CoralStation.rightCenterFace
                                                .getRotation()
                                                .getDegrees()))
                .onTrue(backwardMode.setFalse());

        bargeZone.and(barge, () -> !swerve.frontClosestToAngle(180)).onTrue(backwardMode.setTrue());
        bargeZone.and(barge, () -> swerve.frontClosestToAngle(180)).onTrue(backwardMode.setFalse());

        topLeftZone
                .and(reefPosition, () -> !swerve.frontClosestToAngle(60))
                .onTrue(backwardMode.setTrue());
        topLeftZone
                .and(reefPosition, () -> swerve.frontClosestToAngle(60))
                .onTrue(backwardMode.setFalse());

        topRightZone
                .and(reefPosition, () -> !swerve.frontClosestToAngle(-60))
                .onTrue(backwardMode.setTrue());
        topRightZone
                .and(reefPosition, () -> swerve.frontClosestToAngle(-60))
                .onTrue(backwardMode.setFalse());

        bottomLeftZone
                .and(reefPosition, () -> !swerve.frontClosestToAngle(120))
                .onTrue(backwardMode.setTrue());
        bottomLeftZone
                .and(reefPosition, () -> swerve.frontClosestToAngle(120))
                .onTrue(backwardMode.setFalse());

        bottomRightZone
                .and(reefPosition, () -> !swerve.frontClosestToAngle(-120))
                .onTrue(backwardMode.setTrue());
        bottomRightZone
                .and(reefPosition, () -> swerve.frontClosestToAngle(-120))
                .onTrue(backwardMode.setFalse());
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }
}
