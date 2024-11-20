package frc.spectrumLib.sim;

import frc.spectrumLib.sim.Mount.MountType;

public interface Mountable {

    default double getUpdatedX(
            MountType mountType,
            double initialX,
            double initialY,
            double mountX,
            double mountY,
            double displacementX,
            double displacementY,
            double mountAngle) {
        switch (mountType) {
            case LINEAR:
                return initialX + displacementX;
            case ARM:
                return getXWithAngle(
                        getMountRootDistance(initialX, initialY, mountX, mountY),
                        mountAngle,
                        displacementX + mountX);
            default:
                return 0;
        }
    }

    default double getUpdatedY(
            MountType mountType,
            double initialX,
            double initialY,
            double mountX,
            double mountY,
            double displacementX,
            double displacementY,
            double mountAngle) {
        switch (mountType) {
            case LINEAR:
                return initialY + displacementY;
            case ARM:
                return getYWithAngle(
                        getMountRootDistance(initialX, initialY, mountX, mountY),
                        mountAngle,
                        displacementY + mountY);
            default:
                return 0;
        }
    }

    default double getUpdatedX(RollerConfig config) {
        Mount mount = config.getMount();
        return getUpdatedX(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getMountX(),
                config.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    default double getUpdatedX(ArmConfig config) {
        Mount mount = config.getMount();
        return getUpdatedX(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getMountX(),
                config.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    default double getUpdatedX(LinearConfig config) {
        Mount mount = config.getMount();
        return getUpdatedX(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getMountX(),
                config.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    default double getUpdatedY(RollerConfig config) {
        Mount mount = config.getMount();
        return getUpdatedY(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getMountX(),
                config.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    default double getUpdatedY(ArmConfig config) {
        Mount mount = config.getMount();
        return getUpdatedY(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getMountX(),
                config.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    default double getUpdatedY(LinearConfig config) {
        Mount mount = config.getMount();
        return getUpdatedY(
                mount.getMountType(),
                config.getInitialX(),
                config.getInitialY(),
                config.getMountX(),
                config.getMountY(),
                mount.getDisplacementX(),
                mount.getDisplacementY(),
                mount.getAngle());
    }

    static double getMountRootDistance(
            double initialX, double initialY, double mountX, double mountY) {
        return Math.sqrt(Math.pow(initialX - mountX, 2) + Math.pow(initialY - mountY, 2));
    }

    static double getXWithAngle(double radius, double angle, double displacementX) {
        return radius * Math.cos(angle) + displacementX;
    }

    static double getYWithAngle(double radius, double angle, double displacementY) {
        return radius * Math.sin(angle) + displacementY;
    }
}
