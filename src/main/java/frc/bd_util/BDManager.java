package frc.bd_util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class BDManager {
    static private BDManager _singleton;
    static private boolean _started = false;

    private ArrayList<BDUpdatable> update_list;
    private ShuffleboardTab tab;
    private ShuffleboardLayout status_layout;

    public static BDManager getInstance() {
        if (_singleton != null) {
            return _singleton;
        } else {
            DriverStation.reportError("BDUpdateManager is not initialized", false);
            return null;
        }
    }

    public static boolean isInstantiated() {
        return _started;
    }

    public static void initialize() {
        if (!_started) {
            _singleton = new BDManager();
            _started = true;
        }
    }

    private BDManager() {
        update_list = new ArrayList<BDUpdatable>();
        tab = Shuffleboard.getTab("BDManager");
        status_layout = tab.getLayout("BD Device Statuses", BuiltInLayouts.kGrid);
    }

    public void register(BDUpdatable item) {
        if (isInstantiated()) {
            update_list.add(item);
            status_layout.addString(item.getID(), () -> item.getStatus());
        }
    }

    public void update() {
        for (BDUpdatable item: update_list) {
            item.update();
        }
    }
}