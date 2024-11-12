package frc.robot.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.leds.SpectrumLEDs;
import lombok.Getter;

public class Led extends SpectrumLEDs {

    @Getter
    private Command defaultCommand =
            setPattern(blink(Color.kOrange, 1), -1).withName("LEDs.defaultCommand");

    final Trigger defaultTrigger = new Trigger(() -> defaultCommand.isScheduled());

    public static class LedConfig extends Config {
        public LedConfig() {
            super();
            setPort(0);
            setLength(29);
            setLedSpacing(Meters.of(1 / 120.0));
        }
    }

    protected LedConfig config;

    public Led(LedConfig config) {
        super(config);
        this.config = config;
        setCurrentCommand(defaultCommand);
    }

    /**
     * Binds the triggers for the LED commands. This method overrides the bindTriggers method to
     * ensure that the LED commands are properly bound to their respective triggers.
     */
    @Override
    public void bindTriggers() {
        LedCommands.bindTriggers();
    }

    /**
     * Sets up the default command for the LED subsystem. This method is called to assign the
     * default command that will run when no other commands are scheduled for the subsystem.
     */
    @Override
    public void setupDefaultCommand() {
        setDefaultCommand(defaultCommand);
    }
}
