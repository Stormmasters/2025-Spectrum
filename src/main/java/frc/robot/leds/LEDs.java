package frc.robot.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.util.Color;
import frc.spectrumLib.leds.SpectrumLEDs;

public class LEDs extends SpectrumLEDs {

    public static class LedsConfig extends Config {
        public LedsConfig() {
            super();
            setPort(0);
            setLength(29);
            setLedSpacing(Meters.of(1 / 120.0));
        }
    }

    protected LedsConfig config;

    public LEDs(LedsConfig config) {
        super(config);
        this.config = config;
    }

    @Override
    public void bindTriggers() {
        // TODO Auto-generated method stub
    }

    @Override
    public void setupDefaultCommand() {
        // setDefaultCommand(
        // setPattern(
        //         bounce(config.getSPECTRUM_COLOR(), 3)
        //                 .breathe(Seconds.of(0.5))));
        // setDefaultCommand(setPattern(ombre(config.getSPECTRUM_COLOR(), Color.kWhite)));
        // setDefaultCommand(setPattern(countdown(() -> Timer.getFPGATimestamp(), 15)));
        setDefaultCommand(setPattern(wave(config.getSPECTRUM_COLOR(), Color.kWhite, 8, 1)));
    }
}
