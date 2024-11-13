package frc.spectrumLib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class SpectrumState implements BooleanSupplier {

    private boolean state = false;
    private final Trigger trigger;

    public SpectrumState() {
        this.trigger = new Trigger(this::getState);
    }

    private boolean getState() {
        return state;
    }

    public Trigger get() {
        return trigger;
    }

    private void setState(boolean state) {
        this.state = state;
    }

    private void toggleState() {
        setState(!state);
    }

    public Command set(boolean state) {
        return Commands.run(() -> setState(state));
    }

    public Command toggle() {
        return Commands.run(() -> toggleState());
    }

    @Override
    public boolean getAsBoolean() {
        return state;
    }
}
