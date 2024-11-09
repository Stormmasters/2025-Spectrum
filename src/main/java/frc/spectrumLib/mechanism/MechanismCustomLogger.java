package frc.spectrumLib.mechanism;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(Mechanism.class)
public class MechanismCustomLogger extends ClassSpecificLogger<Mechanism> {
    public MechanismCustomLogger() {
        super(Mechanism.class);
    }

    @Override
    public void update(DataLogger dataLogger, Mechanism mechanism) {
        // This is where you would log the mechanism's state to the data logger

        dataLogger.log("isAttached", mechanism.isAttached());
    }
}
