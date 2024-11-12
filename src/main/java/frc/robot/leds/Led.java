package frc.robot.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.leds.SpectrumLEDs;
import lombok.Getter;

public class Led extends SpectrumLEDs {

    @Getter
    private Command defaultCommand =
            setPattern(LEDPattern.solid(Color.kOrange).blink(Seconds.of(1)))
                    .withName("LEDs.defaultCommand");

    final Trigger defaultTrigger;

    public static class LedsConfig extends Config {
        public LedsConfig() {
            super();
            setPort(0);
            setLength(29);
            setLedSpacing(Meters.of(1 / 120.0));
        }
    }

    protected LedsConfig config;

    public Led(LedsConfig config) {
        super(config);
        this.config = config;

        defaultTrigger = new Trigger(() -> defaultCommand.isScheduled());
    }

    @Override
    public void bindTriggers() {
        LedCommands.bindTriggers();
        // TODO Auto-generated method stub
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
