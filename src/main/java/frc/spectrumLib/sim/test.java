// package frc.spectrumLib.sim;

// import frc.spectrumLib.sim.Mount.MountType;

// public interface Mountable {

//     default double getUpdatedX(
//             MountType mountType,
//             double initialX,
//             double initialY,
//             double initMountX,
//             double initMountY,
//             double initMountAngle,
//             double mountX,
//             double mountY,
//             double displacementX,
//             double displacementY,
//             double mountAngle) {
//         switch (mountType) {
//             case LINEAR:
//                 return getXWithAngle(
//                         getDistance(
//                                 initialX + displacementX, initialY + displacementY, mountX,
// mountY),
//                         mountAngle
//                                 + getAngleOffset(
//                                         initialX, initialY, initMountX, initMountY,
// initMountAngle),
//                         mountX);
//                 // + displacementX;
//             case ARM:
//                 return getXWithAngle(
//                         getDistance(initialX, initialY, mountX, mountY),
//                         mountAngle
//                                 + getAngleOffset(
//                                         initialX, initialY, initMountX, initMountY,
// initMountAngle),
//                         mountX);
//                 // displacementX + mountX);
//             default:
//                 return 0;
//         }
//     }

//     default double getUpdatedY(
//             MountType mountType,
//             double initialX,
//             double initialY,
//             double initMountX,
//             double initMountY,
//             double initMountAngle,
//             double mountX,
//             double mountY,
//             double displacementX,
//             double displacementY,
//             double mountAngle) {
//         switch (mountType) {
//             case LINEAR:
//                 return getYWithAngle(
//                         getDistance(
//                                 initialX + displacementX, initialY + displacementY, mountX,
// mountY),
//                         mountAngle
//                                 + getAngleOffset(
//                                         initialX, initialY, initMountX, initMountY,
// initMountAngle),
//                         mountY);
//                 // + displacementY;
//             case ARM:
//                 return getYWithAngle(
//                         getDistance(initialX, initialY, initMountX, initMountY),
//                         mountAngle
//                                 + getAngleOffset(
//                                         initialX, initialY, initMountX, initMountY,
// initMountAngle),
//                         mountY);
//                 // displacementY + mountY);
//             default:
//                 return 0;
//         }
//     }

//     default double getUpdatedX(
//             Mount mount,
//             double initialX,
//             double initialY,
//             double initMountX,
//             double initMountY,
//             double initMountAngle) {
//         return getUpdatedX(
//                 mount.getMountType(),
//                 initialX,
//                 initialY,
//                 initMountX,
//                 initMountY,
//                 initMountAngle,
//                 mount.getMountX(),
//                 mount.getMountY(),
//                 mount.getDisplacementX(),
//                 mount.getDisplacementY(),
//                 mount.getAngle());
//     }

//     default double getUpdatedY(
//             Mount mount,
//             double initialX,
//             double initialY,
//             double initMountX,
//             double initMountY,
//             double initMountAngle) {
//         return getUpdatedX(
//                 mount.getMountType(),
//                 initialX,
//                 initialY,
//                 initMountX,
//                 initMountY,
//                 initMountAngle,
//                 mount.getMountX(),
//                 mount.getMountY(),
//                 mount.getDisplacementX(),
//                 mount.getDisplacementY(),
//                 mount.getAngle());
//     }

//     default double getUpdatedX(RollerConfig config) {
//         return getUpdatedX(
//                 config.getMount(),
//                 config.getInitialX(),
//                 config.getInitialY(),
//                 config.getInitMountX(),
//                 config.getInitMountY(),
//                 config.getInitMountAngle());
//     }

//     default double getUpdatedX(ArmConfig config) {
//         return getUpdatedX(
//                 config.getMount(),
//                 config.getInitialX(),
//                 config.getInitialY(),
//                 config.getInitMountX(),
//                 config.getInitMountY(),
//                 config.getInitMountAngle());
//     }

//     default double getUpdatedX(LinearConfig config) {
//         return getUpdatedX(
//                 config.getMount(),
//                 config.getInitialX(),
//                 config.getInitialY(),
//                 config.getInitMountX(),
//                 config.getInitMountY(),
//                 config.getInitMountAngle());
//     }

//     default double getUpdatedY(RollerConfig config) {
//         return getUpdatedY(
//                 config.getMount(),
//                 config.getInitialX(),
//                 config.getInitialY(),
//                 config.getInitMountX(),
//                 config.getInitMountY(),
//                 config.getInitMountAngle());
//     }

//     default double getUpdatedY(ArmConfig config) {
//         return getUpdatedY(
//                 config.getMount(),
//                 config.getInitialX(),
//                 config.getInitialY(),
//                 config.getInitMountX(),
//                 config.getInitMountY(),
//                 config.getInitMountAngle());
//     }

//     default double getUpdatedY(LinearConfig config) {
//         return getUpdatedY(
//                 config.getMount(),
//                 config.getInitialX(),
//                 config.getInitialY(),
//                 config.getInitMountX(),
//                 config.getInitMountY(),
//                 config.getInitMountAngle());
//     }

//     static double getAngleOffset(
//             double initialX, double initialY, double mountX, double mountY, double startingAngle)
// {
//         double hypotenuse = getDistance(initialX, initialY, mountX, mountY);
//         if (initialX >= mountX) {
//             return Math.asin((initialY - mountY) / hypotenuse) - startingAngle;
//         } else {
//             return Math.toRadians(180)
//                     - Math.asin((initialY - mountY) / hypotenuse)
//                     - startingAngle;
//         }
//     }

//     static double getDistance(double x1, double y1, double x2, double y2) {
//         return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
//     }

//     static double getXWithAngle(double radius, double angle, double displacementX) {
//         return radius * Math.cos(angle) + displacementX;
//     }

//     static double getYWithAngle(double radius, double angle, double displacementY) {
//         return radius * Math.sin(angle) + displacementY;
//     }
// }
