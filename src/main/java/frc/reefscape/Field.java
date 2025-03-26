// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.reefscape;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import lombok.Getter;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class Field {
    @Getter public static final double fieldLength = Units.inchesToMeters(690.876);
    @Getter private static final double halfLength = fieldLength / 2.0;
    @Getter public static final double fieldWidth = Units.inchesToMeters(317);
    @Getter private static final double halfWidth = fieldWidth / 2.0;

    @Getter
    private static final Pose2d centerField = new Pose2d(halfLength, halfWidth, new Rotation2d());

    @Getter
    public static final double startingLineX =
            Units.inchesToMeters(299.438); // Measured from the inside of starting line

    public static class Processor {
        public static final Pose2d centerFace =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
    }

    public static class Barge {
        public static final Translation2d farCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d middleCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d closeCage =
                new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double deepHeight = Units.inchesToMeters(3.125);
        public static final double shallowHeight = Units.inchesToMeters(30.125);

        /**
         * Returns the y value of a cage when closest to a given cage. 
         * @return double
         */
        public double getCageToClimb() {
            Pose2d robotPose = Robot.getSwerve().getRobotPose();
            int index = 0;
            //Default is blue cages
            double[] cageDiffs = {
                Math.abs(robotPose.getY() - farCage.getY()),
                Math.abs(robotPose.getY() - middleCage.getY()),
                Math.abs(robotPose.getY() - closeCage.getY())};
    
            if(isRed()) {
                cageDiffs[0] =
                        Math.abs(
                                Field.flipYifRed(robotPose.getY())
                                        - Field.flipYifRed(Units.inchesToMeters(286.779)));
                cageDiffs[1] =
                        Math.abs(
                                Field.flipYifRed(robotPose.getY())
                                        - Field.flipYifRed(Units.inchesToMeters(242.855)));
                cageDiffs[2] =
                        Math.abs(
                                Field.flipYifRed(robotPose.getY())
                                        - Field.flipYifRed(Units.inchesToMeters(199.947)));
    
                
            }
            index = indexOfSmallest(cageDiffs);


            //R1 or B1 
            if (index == 0) {
                //R1 or B1 
                return cageDiffs[index];
            } else if (index == 1) { 
                //R2 or B2
                return cageDiffs[index];
            } else if (index == 2) {
                //R3 or B3
                return cageDiffs[index];
            } else {
                //Robot Y if no target zone found
                return Robot.getSwerve().getRobotPose().getY();
            }

        }
    
        public static int indexOfSmallest(double[] array) {
            int indexOfSmallest = 0;
            double smallestIndex = array[indexOfSmallest];
            for (int i = 0; i < array.length; i++) {
                if (array[i] <= smallestIndex) {
                    smallestIndex = array[i];
                    indexOfSmallest = i;
                }
            }
            return indexOfSmallest;
        }
    
    }

    public static class CoralStation {
        public static final Pose2d leftCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d rightCenterFace =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(25.824),
                        Rotation2d.fromDegrees(144.011 - 90));
    }

    public static class Reef {
        public static final Translation2d center =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
        public static final double faceToZoneLine =
                Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        @SuppressWarnings("all")
        @Getter
        public static final Pose2d[] centerFaces =
                new Pose2d[6]; // Starting facing the driver station in clockwise order

        @SuppressWarnings("all")
        public static final List<Map<ReefHeight, Pose3d>> branchPositions =
                new ArrayList<>(); // Starting at the right branch facing the driver station in
        // clockwise

        static {
            // Initialize faces
            centerFaces[0] = // reef id 17
                    new Pose2d(
                            Units.inchesToMeters(160.375),
                            Units.inchesToMeters(130.144),
                            Rotation2d.fromDegrees(-120));
            centerFaces[1] = // reef id 18
                    new Pose2d(
                            Units.inchesToMeters(144.003),
                            Units.inchesToMeters(158.500),
                            Rotation2d.fromDegrees(180));
            centerFaces[2] = // reef id 19
                    new Pose2d(
                            Units.inchesToMeters(160.373),
                            Units.inchesToMeters(186.857),
                            Rotation2d.fromDegrees(120));
            centerFaces[3] = // reef id 20
                    new Pose2d(
                            Units.inchesToMeters(193.116),
                            Units.inchesToMeters(186.858),
                            Rotation2d.fromDegrees(60));
            centerFaces[4] = // reef id 21
                    new Pose2d(
                            Units.inchesToMeters(209.489),
                            Units.inchesToMeters(158.502),
                            Rotation2d.fromDegrees(0));
            centerFaces[5] = // reef id 22
                    new Pose2d(
                            Units.inchesToMeters(193.118),
                            Units.inchesToMeters(130.145),
                            Rotation2d.fromDegrees(-60));
        }

        /**
         * Returns the reef index zone based on the robot's pose
         *
         * @param pose
         * @return
         */
        public static int getReefZone(Pose2d pose) {
            Translation2d point = pose.getTranslation();
            Translation2d relativePoint = point.minus(center);
            double angle = Math.atan2(relativePoint.getY(), relativePoint.getX());
            double distance = relativePoint.getNorm();

            // Normalize angle to be between 0 and 2*PI
            if (angle < 0) {
                angle += 2 * Math.PI;
            }

            // Check if the point is within the 4.5 meters radius
            if (distance > 4.5) {
                return -1; // Outside the zones
            }

            // Determine the zone based on the angle
            double zoneAngle = Math.PI / 3; // 60 degrees per zone
            for (int i = 0; i < 6; i++) {
                if (angle >= i * zoneAngle && angle < (i + 1) * zoneAngle) {
                    return i;
                }
            }

            return -1; // Should not reach here
        }

        /**
         * Returns the reef tag ID based on the robot's pose
         *
         * @param pose
         * @return
         */
        public static int getReefZoneTagID(Pose2d pose) {
            pose = flipIfRed(pose);
            int tag = indexToReefTagID(getReefZone(pose));

            if (isRed()) {
                tag = blueToRedTagID(tag);
            }

            return tag;
        }

        public static Pose2d getOffsetPosition(int blueTagID, double offsetMeters) {
            if(blueTagID < 0) {
                return Robot.getSwerve().getRobotPose();
            }

            int faceIndex = reefTagIDToIndex(blueTagID);
            if (isRed()) {
                //converts blue tag to red and indexes the tag
                faceIndex = reefTagIDToIndex(blueToRedTagID(blueTagID));
            } 

            if (faceIndex < 0 || faceIndex >= centerFaces.length) {
                System.out.println("Invalid face index: returning mid field");
                return new Pose2d(halfLength, halfWidth, new Rotation2d());
            }


            Pose2d face = centerFaces[faceIndex];
            Rotation2d rotation = face.getRotation();
            // Calculate the perpendicular offset
            //TODO: check if rotation matches the target
            Translation2d offset = new Translation2d(offsetMeters, rotation);
            // Apply the offset to the face's position
            Translation2d newTranslation = face.getTranslation().plus(offset);
            return new Pose2d(newTranslation, rotation);
        }

        /**
         * Converts an index to a reef tag ID
         *
         * @param index
         * @return
         */
        public static int indexToReefTagID(int index) {
            return index + 17;
        }


        /**
         * Converts a given Reef Tag Id into 
         * index form for centerfaces to pull from
         * 
         * @param tagID
         * @return
         */
        public static int reefTagIDToIndex(int tagID) {
            //red reef indexer
            if(isRed()) {
                if (tagID < 6 || tagID > 11) {
                    return -1;
                }
    
                return -1 * (tagID - 11);
            }

            //blue reef indexer
            if (tagID < 17 || tagID > 22) {
                return -1;
            }
            return tagID - 17;
        }


        public static int blueToRedTagID(int blueTagID) {
            switch (blueTagID) {
                case 17:
                    return 8;
                case 18:
                    return 7;
                case 19:
                    return 6;
                case 20:
                    return 11;
                case 21:
                    return 10;
                case 22:
                    return 9;
                default:
                    return blueTagID;
            }
        }

        /**Returns the reef face pose 
         * based on the tag ID sent from either red or blue
         * 
         * @param tagID 
         * @return Pose2d of reef side
         * */
        public static Pose2d getReefSideFromTagID(int tagID) {
            Pose2d reefFacePose = centerFaces[reefTagIDToIndex(tagID)];

            if (isRed()) {
                reefFacePose = flipIfRed(reefFacePose);
                return flipIfRed(reefFacePose);
            }

            return reefFacePose;
        }

        /**
         * Returns the score pose based on the reef tag ID received by the limelight
         *
         * @param blueReefTagID may also be a red reef tag ID that will later be converted from a
         *     blue reef tagID
         * @return
         */
        //TODO: Build a Swerve Command in Swerve States to use this method for a target value
        public static Pose2d getScorePoseFromTagID(int blueReefTagID) {
            if (blueReefTagID < 0) {
                return Robot.getSwerve().getRobotPose();
            }
            if (blueReefTagID > 22) {
                return Robot.getSwerve().getRobotPose();
            }
            double offSetMeters = Robot.getSwerve().homeOffsets(blueReefTagID);

            return getOffsetPosition(blueReefTagID, offSetMeters);
        }
    }

    public static class StagingPositions {
        // Measured from the center of the ice cream
        public static final Pose2d leftIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
        public static final Pose2d middleIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
        public static final Pose2d rightIceCream =
                new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
    }

    public enum ReefHeight {
        L4(Units.inchesToMeters(72), -90),
        L3(Units.inchesToMeters(47.625), -35),
        L2(Units.inchesToMeters(31.875), -35),
        L1(Units.inchesToMeters(18), 0);

        ReefHeight(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // in degrees
        }

        public final double height;
        public final double pitch;
    }

    @Getter private static final double aprilTagWidth = Units.inchesToMeters(6.50);

    /** Returns {@code true} if the robot is on the blue alliance. */
    public static boolean isBlue() {
        return DriverStation.getAlliance()
                .orElse(DriverStation.Alliance.Blue)
                .equals(DriverStation.Alliance.Blue);
    }

    /** Returns {@code true} if the robot is on the red alliance. */
    public static boolean isRed() {
        return !isBlue();
    }

    public static final Trigger red = new Trigger(Field::isRed);
    public static final Trigger blue = new Trigger(Field::isBlue);

    public static double flipAngleIfRed(double blue) {
        if (Field.isRed()) {
            return (blue + 180) % 360;
        }
        return blue;
    }

    public static Rotation2d flipAngleIfRed(Rotation2d blue) {
        if (Field.isRed()) {
            return blue.rotateBy(Rotation2d.fromDegrees(180));
        }
        return blue;
    }

    public static Translation2d flipIfRed(Translation2d blue) {
        return new Translation2d(flipXifRed(blue.getX()), flipYifRed(blue.getY()));
    }

    public static Translation3d flipIfRed(Translation3d blue) {
        return new Translation3d(flipXifRed(blue.getX()), flipYifRed(blue.getY()), blue.getZ());
    }

    public static Pose2d flipIfRed(Pose2d blue) {
        return new Pose2d(flipIfRed(blue.getTranslation()), flipAngleIfRed(blue.getRotation()));
    }

    // If we are red flip the x pose to the other side of the field
    public static double flipXifRed(double xCoordinate) {
        if (Field.isRed()) {
            return Field.fieldLength - xCoordinate;
        }
        return xCoordinate;
    }

    // If we are red flip the y pose to the other side of the field
    public static double flipYifRed(double yCoordinate) {
        if (Field.isRed()) {
            return Field.fieldWidth - yCoordinate;
        }
        return yCoordinate;
    }

    public static boolean poseOutOfField(Pose2d pose2D) {
        double x = pose2D.getX();
        double y = pose2D.getY();
        return (x <= 0 || x >= fieldLength) || (y <= 0 || y >= fieldWidth);
    }

    public static boolean poseOutOfField(Pose3d pose3D) {
        return poseOutOfField(pose3D.toPose2d());
    }
}
