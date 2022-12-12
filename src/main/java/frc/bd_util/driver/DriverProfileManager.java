package frc.bd_util.driver;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Swerve;
import frc.swervelib.util.SwerveConstants;

public class DriverProfileManager {
    static public HashMap<String, String> info_list = new HashMap<String, String>();
    static {
        info_list.put("stick_deadband", SwerveConstants.deadzone + "");
        info_list.put("open_ramp", SwerveConstants.Swerve.openLoopRamp + "");
        info_list.put("closed_ramp", SwerveConstants.Swerve.closedLoopRamp + "");
        info_list.put("max_speed", SwerveConstants.Swerve.maxSpeed + "");
        info_list.put("max_angular_velocity", SwerveConstants.Swerve.maxAngularVelocity + "");
        info_list.put("brake_angle", true + "");
        info_list.put("brake_drive", false + "");
    }

    private ShuffleboardTab tab;
    private Swerve swerve;
    private ArrayList<DriverProfile> arr = new ArrayList<DriverProfile>();
    private NetworkTableInstance inst;
    private String entry_path;

    private ArrayList<SimpleWidget> widgets;

    private DriverProfile cur_selected;

    private SendableChooser<DriverProfile> chooser = new SendableChooser<DriverProfile>();

    @SuppressWarnings("unchecked")
    public DriverProfileManager(Swerve swerve) {
        tab = Shuffleboard.getTab("Driver Profile Manager");
        inst = NetworkTableInstance.getDefault();
        this.swerve = swerve;
        this.widgets = new ArrayList<SimpleWidget>();
        this.entry_path = new File("").getAbsolutePath();
        
        ShuffleboardLayout layout_1 = tab.getLayout("Settings", BuiltInLayouts.kList)
        .withPosition(6, 0)
        .withSize(3, 4);

        for (Map.Entry<String, String> str_ptr: info_list.entrySet()) {
            try {
                double dval = Double.valueOf(str_ptr.getValue());
                SimpleWidget saved = layout_1.add(str_ptr.getKey(), dval);
                widgets.add(saved);
            } catch (NumberFormatException ed) {
                // Not a double. Assume it is boolean
                boolean b = Boolean.valueOf(str_ptr.getValue());
                SimpleWidget saved = layout_1.add(str_ptr.getKey(), b)
                .withWidget(BuiltInWidgets.kToggleButton);
                widgets.add(saved);
            }
        }

        SimpleWidget profile_adder = tab.add("Create new profile?", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(1, 1)
        .withSize(2, 1);

        SimpleWidget namer = tab.add("Name", cur_selected != null ? cur_selected.getName() : "None")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(1,0);

        ShuffleboardLayout profile_chooser_layout = tab.getLayout("Profile", BuiltInLayouts.kList)
        .withPosition(2, 0)
        .withSize(2, 1);
        profile_chooser_layout.add("Profile Chooser", chooser);

        inst.addListener(profile_adder.getEntry(), EnumSet.of(Kind.kValueRemote), event -> {
            profile_adder.getEntry().setBoolean(false);
            DriverProfile new_driver = new DriverProfile(namer.getEntry().getString("Driver " + arr.size()));
            chooser.addOption(new_driver.getName(), new_driver);
            arr.add(new_driver);

            // reset the settings
            for (SimpleWidget widget: widgets) {
                try {
                    double dval = Double.valueOf(info_list.get(widget.getTitle()));
                    widget.getEntry().setDouble(dval);
                } catch (NumberFormatException ed) {
                    // Not a double. Assume it is boolean
                    boolean b = Boolean.valueOf(info_list.get(widget.getTitle()));
                    widget.getEntry().setBoolean(b);
                }
            }
        });

        SimpleWidget select_button = tab.add("Select and refresh", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(3, 1)
        .withSize(2, 1);

        inst.addListener(select_button.getEntry(), EnumSet.of(Kind.kValueRemote), event -> {
            select_button.getEntry().setBoolean(false);
            cur_selected = chooser.getSelected();

            for (SimpleWidget widget: widgets) {
                try {
                    double dval = Double.valueOf(cur_selected.getSettings().get(widget.getTitle()));
                    widget.getEntry().setDouble(dval);
                } catch (NumberFormatException ed) {
                    // Not a double. Assume it is boolean
                    boolean b = Boolean.valueOf(cur_selected.getSettings().get(widget.getTitle()));
                    widget.getEntry().setBoolean(b);
                }
            }
        });

        SimpleWidget save_cur_to_obj = tab.add("Save profile", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(4,0);

        inst.addListener(save_cur_to_obj.getEntry(), EnumSet.of(Kind.kValueRemote), event -> { 
            save_cur_to_obj.getEntry().setBoolean(false);           
            HashMap<String, String> replacement = new HashMap<String, String>();

            for (SimpleWidget widget: widgets) {
                replacement.put(widget.getTitle(), widget.getEntry().get().getValue() + "");
            }
            
            cur_selected.setSettings(replacement);
            swerve.updatePreferences(cur_selected.getSettings());
        });

        SimpleWidget save = tab.add("Save all to JSON", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(2, 2)
        .withSize(2, 1);

        inst.addListener(save.getEntry(), EnumSet.of(Kind.kValueRemote), event -> {            
            save.getEntry().setBoolean(false);
            
            for (DriverProfile driver_profile: arr) {
                File search = new File(entry_path + "/src/main/java/frc/bd_util/driver/driver_profiles/" + driver_profile.getName() + ".json");
                if (search.isFile()) {
                    System.out.println("exists");
                    try {
                        try (FileWriter filewriter = new FileWriter(search)) {
                            JSONObject finale = new JSONObject(driver_profile.getSettings());
                            filewriter.write(finale.toJSONString());
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                        DriverStation.reportError("FileWriter failed in class DriverProfileManager", false);
                    }
                } else {
                    DriverStation.reportError("Did not write to file " + driver_profile.getName() + " because file did not previously exist.", false);
                }
            }
        });

        File[] list_of_files = new File(entry_path + "/src/main/java/frc/bd_util/driver/driver_profiles").listFiles();
        for (int i = 0; i < list_of_files.length; i++) {
            File current = list_of_files[i];
            JSONParser parser = new JSONParser();
            try {
                JSONObject obj = (JSONObject) parser.parse(new FileReader(current));
                HashMap<String, String> new_map = new HashMap<String, String>();

                new_map.putAll(obj);
                DriverProfile new_driver = new DriverProfile(current.getName().split("\\.")[0], new_map);

                arr.add(new_driver);

                chooser.addOption(new_driver.getName(), new_driver);

            } catch (IOException | ParseException e) {
                e.printStackTrace();
            }
        }

    }

    public ShuffleboardTab getTab() {
        return tab;
    }
}