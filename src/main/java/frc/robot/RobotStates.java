package frc.robot;

import static frc.robot.auton.Auton.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.reefscape.Field;
import frc.robot.elbow.ElbowStates;
import frc.robot.elevator.ElevatorStates;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.robot.shoulder.ShoulderStates;
import frc.robot.swerve.Swerve;
import frc.spectrumLib.Rio;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.util.Util;

public class RobotStates {
    private static final Pilot pilot = Robot.getPilot();
    private static final Operator operator = Robot.getOperator();
    private static final Swerve swerve = Robot.getSwerve();

    // Robot States
    // These are states that aren't directly tied to hardware or buttons, etc.
    // If they should be set by multiple Triggers do that in SetupStates()
    public static final SpectrumState coastMode = new SpectrumState("coast");
    public static final SpectrumState coral = new SpectrumState("coral");
    public static final SpectrumState algae = new SpectrumState("algae");
    public static final SpectrumState l1 = new SpectrumState("l1");
    public static final SpectrumState l2 = new SpectrumState("l2");
    public static final SpectrumState l3 = new SpectrumState("l3");
    public static final SpectrumState l4 = new SpectrumState("l4");
    public static final SpectrumState leftScore = new SpectrumState("leftScore");
    public static final SpectrumState rightScore = new SpectrumState("rightScore");
    public static final SpectrumState reverse = new SpectrumState("reverse");
    public static final SpectrumState actionPrepState = new SpectrumState("actionPrepState");
    public static final SpectrumState actionState = new SpectrumState("actionState");
    public static final SpectrumState homeAll = new SpectrumState("homeAll");
    public static final SpectrumState intaking = new SpectrumState("intaking");
    public static final SpectrumState climbIntake = new SpectrumState("climbIntake");
    public static final SpectrumState floorIntake = new SpectrumState("floorIntake");

    /**
     * Define Robot States here and how they can be triggered States should be triggers that command
     * multiple mechanism or can be used in teleop or auton Use onTrue/whileTrue to run a command
     * when entering the state Use onFalse/whileFalse to run a command when leaving the state
     * RobotType Triggers
     */
    public static final Trigger pm = new Trigger(() -> Rio.id == Rio.PM_2025);

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

    // Intake Triggers
    public static final Trigger stationIntaking = pilot.stationIntake_LT.or(autonSourceIntake);
    public static final Trigger stationExtendedIntaking = pilot.stationIntakeExtended_LT_RB;
    public static final Trigger groundAlgae = pilot.groundAlgae_RT;
    public static final Trigger groundCoral = Trigger.kFalse;

    // climb Triggers
    public static final Trigger climbPrep = operator.climbPrep_start;
    public static final Trigger climbFinish = pilot.climbRoutine_start;

    // mechanism preset Triggers (Wrist, Elevator, etc.)
    public static final Trigger processorAlgae = (l1.and(algae)).or(autonProcessor);
    public static final Trigger L2Algae = (l2.and(algae)).or(autonLowAlgae);
    public static final Trigger L3Algae = (l3.and(algae)).or(autonHighAlgae);
    public static final Trigger netAlgae = (l4.and(algae)).or(autonNet);
    public static final Trigger stagedAlgae = processorAlgae.or(L2Algae, L3Algae, netAlgae);

    public static final Trigger L1Coral = l1.and(coral).or(autonL1);
    public static final Trigger L2Coral = l2.and(coral);
    public static final Trigger L3Coral = l3.and(coral);
    public static final Trigger L4Coral = (l4.and(coral)).or(autonLeftL4, autonRightL4);
    public static final Trigger branch = L2Coral.or(L3Coral, L4Coral);
    public static final Trigger stagedCoral = L1Coral.or(L2Coral, L3Coral, L4Coral);

    public static final Trigger staged = stagedAlgae.or(stagedCoral);

    // TODO: Handoffs are disabled
    // public static final Trigger algaeHandoff = operator.algaeHandoff_X;
    // public static final Trigger coralHandoff = operator.coralHandoff_Y;

    public static final Trigger isAtHome =
            ElevatorStates.isHome.and(ElbowStates.isHome, ShoulderStates.isHome);

    // reset triggers
    public static final Trigger homeElevator = operator.homeElevator_A;
    public static final Trigger homeInClimb = operator.homeInClimb_B;

    // public static final SpectrumState passiveCoral = new SpectrumState("passiveCoralIntake");
    // public static final SpectrumState passiveAlgae = new SpectrumState("passiveAlgaeIntake");

    public static final Trigger hasCoral = new Trigger(Robot.getCoralIntake()::hasIntakeCoral);
    public static final Trigger hasAlgae = new Trigger(Robot.getCoralIntake()::hasIntakeAlgae);

    public static final Trigger homeAllStopIntake = operator.nothingStaged.and(actionState);

    public static final SpectrumState backwardMode = new SpectrumState("backward");

    // Setup any binding to set states
    public static void setupStates() {
        Util.disabled.whileTrue(clearStates().repeatedly());

        pilot.home_select.or(operator.home_select).whileTrue(homeAll.setTrue());
        pilot.home_select.or(operator.home_select).onFalse(homeAll.setFalse());

        pilot.coastOn_dB.or(operator.coastOn_dB).onTrue(coastMode.setTrue().ignoringDisable(true));
        pilot.coastOff_dA
                .or(operator.coastOff_dA)
                .onTrue(coastMode.setFalse().ignoringDisable(true));

        // Intaking States
        stationIntaking.or(stationExtendedIntaking).whileTrue(coral.setTrue(), algae.setFalse());
        stationIntaking.onChangeToFalse(homeAll.setTrue());

        groundAlgae.whileTrue(floorIntake.setTrue(), algae.setTrue(), coral.setFalse());
        stationIntaking.or(floorIntake).onTrue(intaking.setTrue());
        stationIntaking.not().and(floorIntake.not()).onTrue(intaking.setFalse());

        // Staging and Scoring
        coral.not()
                .and(algae.not(), actionState.not(), actionPrepState.not())
                .onTrue(clearStaged()); // Clear if we aren't scoring, holding, or staged

        actionState
                .not()
                .and(operator.coralStage.not())
                .onTrue(coral.setFalse()); // When we change to not scoring and not coral stage
        // we turn off coral
        actionState
                .not()
                .and(operator.algaeStage.not())
                .onTrue(algae.setFalse()); // When we change to not scoring and not algae stage
        // we turn off algae

        operator.coralStage
                .or(autonLeftL4, autonRightL4)
                .onTrue(coral.setTrue(), algae.setFalse()); // Set coral if we are staging coral

        operator.algaeStage
                .or(autonHighAlgae, autonLowAlgae, autonProcessor, autonNet)
                .onTrue(algae.setTrue(), coral.setFalse()); // Set algae if we are staging algae

        (pilot.actionReady.and(coral.or(algae)))
                .or(autonPreScore)
                .onTrue(actionPrepState.setTrue(), actionState.setFalse());

        // TODO: This currently set actionState true when intaking which is bad
        // (pilot.actionReady.not().and(coral.or(algae)))
        //         .or(autonScore)
        //         .onTrue(actionState.setTrue(), actionPrepState.setFalse());

        // Set Levels
        operator.L1
                .and(operator.staged)
                .onTrue(l1.setTrue(), l2.setFalse(), l3.setFalse(), l4.setFalse());
        operator.L2
                .and(operator.staged)
                .onTrue(l2.setTrue(), l1.setFalse(), l3.setFalse(), l4.setFalse());
        operator.L3
                .and(operator.staged)
                .onTrue(l3.setTrue(), l1.setFalse(), l2.setFalse(), l4.setFalse());
        operator.L4
                .and(operator.staged)
                .onTrue(l4.setTrue(), l1.setFalse(), l2.setFalse(), l3.setFalse());

        operator.leftScore.and(operator.staged).onTrue(leftScore.setTrue(), rightScore.setFalse());
        operator.rightScore.and(operator.staged).onTrue(rightScore.setTrue(), leftScore.setFalse());

        autonLeftL4.onTrue(leftScore.setTrue(), rightScore.setFalse());
        autonRightL4.onTrue(rightScore.setTrue(), leftScore.setFalse());

        pilot.actionReady.onTrue(actionPrepState.setTrue());
        pilot.actionReady.onFalse(actionPrepState.setFalse());

        actionPrepState.onTrue(actionState.setFalse());
        actionPrepState
                .or(autonPreScore)
                .onChangeToFalse(
                        actionState
                                .setTrue()
                                .alongWith(new WaitCommand(2))
                                .andThen(actionState.setFalse()));
        operator.algaeStage.or(operator.coralStage).onTrue(actionState.setFalse());

        // Home if we aren't doing coral, algae, or intaking
        (coral.not().and(algae.not(), intaking.not()))
                .onChangeToTrue(
                        (homeAll.setTrue().alongWith(new WaitCommand(1.5)))
                                .andThen(homeAll.setFalse()));
        homeAllStopIntake.onTrue(
                homeAll.setTrue().alongWith(new WaitCommand(1.5)).andThen(homeAll.setFalse()));
        isAtHome.onTrue(homeAll.setFalse());

        // hasCoral.and(operator.operatorCoralStage.or(stationIntaking))
        //         .onTrue(passiveCoral.setTrue());
        // hasCoral.not()
        //         .and(operator.operatorCoralStage.not())
        //         .or(scoreState)
        //         .debounce(0.5)
        //         .onTrue(passiveCoral.setFalse());

        // hasAlgae.and(operator.operatorAlgaeStage).onTrue(passiveAlgae.setTrue());
        // hasAlgae.not()
        //         .or(operator.operatorAlgaeStage.not())
        //         .debounce(0.5)
        //         .onTrue(passiveAlgae.setFalse());

        // TODO: uncomment the backwards zones
        // bottomLeftZone
        //         .and(stationIntaking)
        //         .onTrue(
        //                 backwardMode
        //                         .setTrue()
        //                         .onlyIf(
        //                                 () ->
        //                                         !swerve.frontClosestToAngle(
        //                                                 Field.CoralStation.leftCenterFace
        //                                                         .getRotation()
        //                                                         .getDegrees())));
        // bottomLeftZone
        //         .and(stationIntaking)
        //         .onTrue(
        //                 backwardMode
        //                         .setFalse()
        //                         .onlyIf(
        //                                 () ->
        //                                         swerve.frontClosestToAngle(
        //                                                 Field.CoralStation.leftCenterFace
        //                                                         .getRotation()
        //                                                         .getDegrees())));

        // bottomRightZone
        //         .and(stationIntaking)
        //         .onTrue(
        //                 backwardMode
        //                         .setTrue()
        //                         .onlyIf(
        //                                 () ->
        //                                         !swerve.frontClosestToAngle(
        //                                                 Field.CoralStation.rightCenterFace
        //                                                         .getRotation()
        //                                                         .getDegrees())));
        // bottomRightZone
        //         .and(stationIntaking)
        //         .onTrue(
        //                 backwardMode
        //                         .setFalse()
        //                         .onlyIf(
        //                                 () ->
        //                                         swerve.frontClosestToAngle(
        //                                                 Field.CoralStation.rightCenterFace
        //                                                         .getRotation()
        //                                                         .getDegrees())));

        // bargeZone
        //         .and(barge)
        //         .onTrue(backwardMode.setTrue().onlyIf(() -> !swerve.frontClosestToAngle(180)));
        // bargeZone
        //         .and(barge)
        //         .onTrue(backwardMode.setFalse().onlyIf(() -> swerve.frontClosestToAngle(180)));

        // topLeftZone
        //         .and(reefPosition)
        //         .onTrue(backwardMode.setTrue().onlyIf(() -> !swerve.frontClosestToAngle(60)));
        // topLeftZone
        //         .and(reefPosition)
        //         .onTrue(backwardMode.setFalse().onlyIf(() -> swerve.frontClosestToAngle(60)));

        // topRightZone
        //         .and(reefPosition)
        //         .onTrue(backwardMode.setTrue().onlyIf(() -> !swerve.frontClosestToAngle(-60)));
        // topRightZone
        //         .and(reefPosition)
        //         .onTrue(backwardMode.setFalse().onlyIf(() -> swerve.frontClosestToAngle(-60)));

        // bottomLeftZone
        //         .and(reefPosition)
        //         .onTrue(backwardMode.setTrue().onlyIf(() -> !swerve.frontClosestToAngle(120)));
        // bottomLeftZone
        //         .and(reefPosition)
        //         .onTrue(backwardMode.setFalse().onlyIf(() -> swerve.frontClosestToAngle(120)));

        // bottomRightZone
        //         .and(reefPosition)
        //         .onTrue(backwardMode.setTrue().onlyIf(() -> !swerve.frontClosestToAngle(-120)));
        // bottomRightZone
        //         .and(reefPosition)
        //         .onTrue(backwardMode.setFalse().onlyIf(() -> swerve.frontClosestToAngle(-120)));
    }

    private RobotStates() {
        throw new IllegalStateException("Utility class");
    }

    private static Command clearStaged() {
        return l1.setFalse()
                .alongWith(
                        l2.setFalse(),
                        l3.setFalse(),
                        l4.setFalse(),
                        leftScore.setFalse(),
                        rightScore.setFalse(),
                        coral.setFalse(),
                        algae.setFalse());
    }

    private static Command clearStates() {
        return clearStaged()
                .alongWith(
                        reverse.setFalse(),
                        actionPrepState.setFalse(),
                        actionState.setFalse(),
                        homeAll.setFalse(),
                        intaking.setFalse(),
                        climbIntake.setFalse(),
                        floorIntake.setFalse());
    }
}
