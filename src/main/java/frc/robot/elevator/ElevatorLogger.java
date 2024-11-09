package frc.robot.elevator;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(Elevator.class)
public class ElevatorLogger extends ClassSpecificLogger<Elevator> {
    public ElevatorLogger() {
        super(Elevator.class);
    }

    @Override
    public void update(DataLogger dataLogger, Elevator elevator) {
        // This is where you would log the mechanism's state to the data logger

        dataLogger.log("isAttached", elevator.isAttached());
        dataLogger.log("Position", elevator.getMotorPosition());
    }
}
