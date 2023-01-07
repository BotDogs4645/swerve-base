package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.bdlib.misc.ValuePair;

import frc.robot.Constants;
import frc.robot.Constants.CameraConstants.CameraDefaults;

public class Vision extends SubsystemBase {
    private enum AprilTagOption {
        AprilOne(0, "Name"),
        AprilTwo(1, "Name2"),
        AprilThree(2, "Name3")
        
        ;
        
        private int id;
        private String name;

        private AprilTagOption(int id, String name) {
            this.id = id;
            this.name = name;
        }
    }

    private PhotonCamera driver_cam;
    private PhotonCamera apriltag_cam;
    private Transform3d centerToAprilTagCamera;
    private PhotonPipelineResult current_captures = new PhotonPipelineResult();

    private AprilTagFieldLayout tag_locations;

    private ArrayList<ValuePair<String, AprilTag>> selectable_tags = new ArrayList<>();

    private ValuePair<String, AprilTag> selected_apriltag;

    private boolean target_in_view = false;

    private ShuffleboardTab sub_tab = Shuffleboard.getTab("Vision");

    public Vision() {
        this.driver_cam = new PhotonCamera("drivervision");
        driver_cam.setDriverMode(true);

        this.apriltag_cam = new PhotonCamera("apriltagvision");
        apriltag_cam.setDriverMode(false);
        this.centerToAprilTagCamera = CameraDefaults.MountOne.getTransformation();

        // assume that we are testing within our own facilities while testing, else use the current field resource file.
        if (Constants.testing) {
            tag_locations = new AprilTagFieldLayout(new ArrayList<AprilTag>(), 0, 0);
        } else {
            try {
                tag_locations = new AprilTagFieldLayout(AprilTagFields.k2022RapidReact.m_resourceFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        if (DriverStation.getAlliance() == Alliance.Blue) {
            tag_locations.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } else {
            tag_locations.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        }



        // add our list of supported apriltags
        for (AprilTagOption avail: AprilTagOption.values()) {
            selectable_tags.add(
                ValuePair.of(
                    avail.name,
                    new AprilTag(avail.id, new Pose3d(0, 0, 0, new Rotation3d()))// tag_locations.getTags().get(avail.id)
                )
            );
        }
        
        this.selected_apriltag = selectable_tags.get(0);

        ShuffleboardLayout layout = sub_tab.getLayout("Selected Value", BuiltInLayouts.kList);
        layout.addString("Name", () -> selected_apriltag.getLeftValue());
        layout.addString("ID", () -> selected_apriltag.getRightValue().ID + "");
        layout.addString("Pose", () -> selected_apriltag.getRightValue().pose.toString());
    }

    @Override
    public void periodic() {
        if (apriltag_cam.getLatestResult().hasTargets()) {
            current_captures = apriltag_cam.getLatestResult();
            for (PhotonTrackedTarget target: current_captures.targets) {
                if (target.getFiducialId() == selected_apriltag.getRightValue().ID) {
                    target_in_view = true;
                    break;
                } else {
                    target_in_view = false;
                }
            }
        } else {
            current_captures = new PhotonPipelineResult();
            target_in_view = false;
        }
    }
    
    // uses optionals for optimal handling outside of the method.
    public Optional<Pose2d> getRobotPoseContributor() {
        if (current_captures.hasTargets()) {
            PhotonTrackedTarget best_target = current_captures.getBestTarget();
            if (best_target.getPoseAmbiguity() < .1) {
                // good capture
                Pose3d fieldRelativeAprilTagPose = tag_locations.getTagPose(best_target.getFiducialId()).get();
                Pose2d calculatedRobotPose = 
                    fieldRelativeAprilTagPose
                        .transformBy(best_target.getBestCameraToTarget().inverse())
                        .transformBy(centerToAprilTagCamera.inverse())
                        .toPose2d();
                return Optional.of(calculatedRobotPose);
            }
        }
        return Optional.empty();
    }

    public boolean isTargetInView() {
        return target_in_view;
    }

    public void cursorRight() {
        int indexOfNext = selectable_tags.indexOf(selected_apriltag) + 1;
        if (indexOfNext == selectable_tags.size()) {
            indexOfNext = 0;
        }

        selected_apriltag = selectable_tags.get(indexOfNext);
    }

    public void cursorLeft() {
        int indexOfNext = selectable_tags.indexOf(selected_apriltag) - 1;
        if (indexOfNext == -1) {
            indexOfNext = selectable_tags.size() - 1;
        }

        selected_apriltag = selectable_tags.get(indexOfNext);
    }
}
