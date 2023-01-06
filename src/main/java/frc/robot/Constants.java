// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean testing = true;

    public static class CameraConstants {
        // "x+" = Pigeon2 orientation dependent ;p - check which direction points forward.. x+ can be technically defined as the x+ from a pose
        // defined from the origin of the robot chassis facing the "front face" of the robot. Or atleast it should be.
        public static enum CameraDefaults {
            MountOne(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            MountTwo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

            Transform3d overallTransform;
            private CameraDefaults(double translationInX, double translationInY, double translationInZ, double changeInRoll, double changeInPitch, double changeInYaw) {
                Translation3d translationRelativeToGyroPosition = new Translation3d(translationInX, changeInYaw, translationInZ);
                Rotation3d rotationRelativeToGyroOrientation = new Rotation3d(changeInRoll, changeInPitch, changeInYaw);

                this.overallTransform = new Transform3d(translationRelativeToGyroPosition, rotationRelativeToGyroOrientation);
            }

            public Transform3d getTransformation() {
                return overallTransform;
            }
        }
    }

    
    
    // 
    public static final double FIELD_ZERO_MAGNETIC_HEADING = 0;


    public static enum AlignedPose {
        ;
        
        Translation2d aligned_pose;
        private AlignedPose(double x, double y) {
            aligned_pose = new Translation2d(x, y);
        }
    }
}