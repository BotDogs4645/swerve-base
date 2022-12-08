package frc.bd_util.driver;

import java.io.File;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;


import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriverProfileManager {
    static public HashMap<String, String> info_list = new HashMap<String, String>();
    static {
        info_list.put("stick_deadband", Constants.stickDeadband + "");
        info_list.put("open_ramp", Constants.Swerve.openLoopRamp + "");
        info_list.put("closed_ramp", Constants.Swerve.closedLoopRamp + "");
        info_list.put("max_speed", Constants.Swerve.maxSpeed + "");
        info_list.put("max_angular_velocity", Constants.Swerve.maxAngularVelocity + "");
        info_list.put("brake_angle", true + "");
        info_list.put("brake_drive", false + "");
    }

    private ShuffleboardTab tab;
    private Swerve swerve;
    private ArrayList<DriverProfile> arr = new ArrayList<DriverProfile>();
    private NetworkTableInstance inst;
    private String entry_path;

    private DriverProfile cur_selected;

    private SendableChooser<DriverProfile> chooser = new SendableChooser<DriverProfile>();

    public DriverProfileManager(Swerve swerve) {
        tab = Shuffleboard.getTab("Driver Profile Manager");
        inst = NetworkTableInstance.getDefault();
        this.swerve = swerve;
        this.entry_path = new File("").getAbsolutePath();
        
        SimpleWidget profile_adder = tab.add("Create new profile?", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(1, 1);

        ShuffleboardLayout profile_chooser_layout = tab.getLayout("Profile", BuiltInLayouts.kList);
        profile_chooser_layout.add("Profile Chooser", chooser);

        inst.addListener(profile_adder.getEntry(), EnumSet.of(Kind.kValueRemote), event -> {
            profile_adder.getEntry().setBoolean(false);
            DriverProfile new_driver = new DriverProfile("Driver " + (arr.size()));
            chooser.addOption(new_driver.getName(), new_driver);
            arr.add(new_driver);
        });

        SimpleWidget select_button = tab.add("Select profile", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(3, 3);

        inst.addListener(select_button.getEntry(), EnumSet.of(Kind.kValueRemote), event -> {
            select_button.getEntry().setBoolean(false);
            cur_selected = chooser.getSelected();
        });

        SimpleWidget save = tab.add("save", false)
        .withWidget(BuiltInWidgets.kToggleButton);

        SimpleWidget namer = tab.add("Name", cur_selected != null ? cur_selected.getName() : "None")
        .withWidget(BuiltInWidgets.kTextView);

        inst.addListener(save.getEntry(), EnumSet.of(Kind.kValueRemote), event -> {            
            select_button.getEntry().setBoolean(false);
            String name = namer.getEntry().getString("None");
            chooser.getSelected().changeName(name);
            
            refreshSendable(chooser.getSelected());

            File search = new File(entry_path + "/src/main/java/fr/bd_util/driver/driver_profiles/" + chooser.getSelected() + ".json");
            if (search.isFile()) {
                System.out.println("happy!");
            } else {
                System.out.println("NO happy!");
            }
        });


    }

    public void refreshSendable(DriverProfile defaulted) {
        SendableChooser<DriverProfile> replacement = new SendableChooser<DriverProfile>();
        for (DriverProfile entry: arr) {
            if (!defaulted.equals(entry)) {
                replacement.addOption(entry.getName(), entry);
            }
        }
        replacement.setDefaultOption(defaulted.getName(), defaulted);

        chooser.close();

        tab.add(replacement);
    }

    public ShuffleboardTab getTab() {
        return tab;
    }


}