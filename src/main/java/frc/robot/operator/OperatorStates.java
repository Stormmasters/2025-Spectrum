package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/** This class should have any command calls that directly call the Operator */
public class OperatorStates {
    private static Operator operator = Robot.getOperator();

    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
        operator.setDefaultCommand(rumble(0, 1)); //.repeatedly().withName("Operator.default"));
    }

    /** Set the states for the operator controller */
    public static void setStates() {}

    /** Command that can be used to rumble the operator controller */
    public static Command rumble(double intensity, double durationSeconds) {
        return operator.rumbleCommand(intensity, durationSeconds);
    }
}
