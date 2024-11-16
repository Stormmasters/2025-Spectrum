package frc.spectrumLib.sim;

public interface LinkSim {
    
    double getAttachedX();

    double getAttachedY();

    void setAttached(double initialX, double initialY);
}
