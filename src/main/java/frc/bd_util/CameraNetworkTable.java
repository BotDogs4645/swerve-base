package frc.bd_util;

import java.util.HashMap;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.bd_util.BDConstants.UTIL_CONSTANTS.CAMERA_DEFAULTS.*;
import frc.bd_util.widgets.BooleanWidget;
import frc.bd_util.widgets.NumberSliderWidget;

public class CameraNetworkTable {
    public static TreeMap<String, CameraNetworkTable> CameraList = new TreeMap<String, CameraNetworkTable>();
    public static NetworkTable CameraTable = NetworkTableInstance.getDefault().getTable("CameraTable");
    public static CameraNetworkTable CurrentCamera_VIEW;
    public static CameraNetworkTable CurrentCamera_IMAGE_PROCESSING;

    private String id;
    private String cameraLocation;
    private NetworkTable camTable;
    private ShuffleboardTab camTab;
    public HashMap<String, SendableChooser<String>> selectableChoosers = new HashMap<String, SendableChooser<String>>();

    public static CameraNetworkTable findCamera(String id) {
        return CameraList.get(id);
    }

    public static void cycleView() {
        if (CurrentCamera_VIEW == null && CameraList.size() != 0) {
            setViewCamera(CameraList.firstEntry().getValue());
        } else if(CurrentCamera_VIEW != null) {
            SortedMap<String, CameraNetworkTable> mapUp = CameraList.tailMap(CurrentCamera_VIEW.id, true);
            if (mapUp.size() == 0) {
                // we've reached the end of the map
                setViewCamera(CameraList.firstEntry().getValue());
            } else {
                setViewCamera(mapUp.get(mapUp.firstKey()));
            }
        }
    }

    public static void panTiltZoomControl(double pan, double tilt, double zoom) {
        CurrentCamera_VIEW.getNetworkTable().getEntry("xyzoom").setNumberArray(
            new Number[] {(int)Math.floor(pan * 6), (int)Math.floor(tilt * 6), (int)Math.floor(zoom * 6)}
        );
    }

    public static void cycleProcessedCamera() {
        if (CurrentCamera_IMAGE_PROCESSING == null && CameraList.size() != 0) {
            setProcessedCamera(CameraList.firstEntry().getValue());
        } else if(CurrentCamera_IMAGE_PROCESSING != null) {
            SortedMap<String, CameraNetworkTable> mapUp = CameraList.tailMap(CurrentCamera_IMAGE_PROCESSING.id, false);
            if (mapUp.size() == 0) {
                // we've reached the end of the map
                setProcessedCamera(CameraList.firstEntry().getValue());
            } else {
                setProcessedCamera(mapUp.get(mapUp.firstKey()));
            }
        }
    }

    public static void setProcessedCamera(CameraNetworkTable camera) {
        CameraTable.getEntry("CURRENT_PROCESSED").setString(camera.id);
        CurrentCamera_IMAGE_PROCESSING = camera;
    }

    public static CameraNetworkTable getProcessedCamera() {
        return CurrentCamera_IMAGE_PROCESSING;
    }

    public static CameraNetworkTable getViewCamera() {
        return CurrentCamera_VIEW;
    }

    public static void setViewCamera(CameraNetworkTable camera) {
        CameraTable.getEntry("CURRENT_VIEW").setString(camera.id);
        CurrentCamera_VIEW = camera;
    }

    public static CameraNetworkTable findCameraByLocation(String type) {
        for (Map.Entry<String, CameraNetworkTable> cameraClass: CameraList.entrySet()) {
            if(cameraClass.getValue().getNetworkTable().getEntry("camera_location").getString("None").equals(type)) {
                return cameraClass.getValue();
            }
        }
        return null;
    }

    public CameraNetworkTable(String id, String cameraLocation) {
        this.id = id;
        this.cameraLocation = cameraLocation;
        this.camTab = Shuffleboard.getTab(cameraLocation + " Camera");
        
        // Adds our camera to the static camera list. This allows us to avoid the need to pass around the camera object if we need to quickly reference it somewhere.
        CameraList.put(this.id, this);

        // Creates a new sub-table in the main CameraTable so our server can index information solely about this camera.
        this.camTable = CameraTable.getSubTable(id);
        // Set up our "entries" into the NetworkTables, meaning that our jetson can access these values and change the camera's settings based on them.
        setupEntries();
    }

    public void setupEntries() {
        camTable.getEntry("id").setString(id);
        camTable.getEntry("camera_location").setString(cameraLocation);

        for(Map.Entry<String, Object[]> store : NORMAL_DEFAULTS.DEFAULT_INT_MAP.entrySet()) {
            camTable.getEntry(store.getKey()).setNumber((int)store.getValue()[0]);
            new NumberSliderWidget(camTab, camTable, store.getKey(), store.getValue());
        }

        for(Map.Entry<String, Object[]> store : NORMAL_DEFAULTS.DEFAULT_BOOLEAN_MAP.entrySet()) {
            camTable.getEntry(store.getKey()).setBoolean((Boolean)store.getValue()[0]);
            new BooleanWidget(camTab, camTable, store.getKey(), store.getValue());
        }

        for(Map.Entry<String, String[]> store : NORMAL_DEFAULTS.DEFAULT_STRING_MAP.entrySet()) {
            SendableChooser<String> sendie = new SendableChooser<String>();
            sendie.setDefaultOption(store.getValue()[0], store.getValue()[0]);

            for (int i = 1; i < store.getValue().length; i++) {
                sendie.addOption(store.getValue()[i], store.getValue()[i]);
            }

            selectableChoosers.put(store.getKey(), sendie);
            camTab.addString(store.getKey(), () -> sendie.getSelected())
                .withWidget(BuiltInWidgets.kTextView);
        }

        camTable.getEntry("bitrate_minmax").setNumberArray(
            new Number[] {SPECIAL_DEFAULTS.DEFAULT_MIN_BITRATE, SPECIAL_DEFAULTS.DEFAULT_MAX_BITRATE}
        );

        camTab.add("bitrate_max", 488)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 48, "max", 488))
        .getEntry()
        .andThen(event -> {
            camTable.getEntry("bitrate_minmax").setNumberArray(new Number[] {SPECIAL_DEFAULTS.DEFAULT_MIN_BITRATE, 
                event.getDouble()
            });
        });

        

        camTable.getEntry("server_ready_status").setBoolean(true);
    }

    public NetworkTable getNetworkTable() {
        return camTable;
    }

    public void setAsProcessedCamera() {
        setProcessedCamera(this);
    }

    public boolean isReady() {
        return camTable.getEntry("client_ready_status").getBoolean(false);
    }

    public void setAsViewCamera() {
        setViewCamera(this);
    }
}