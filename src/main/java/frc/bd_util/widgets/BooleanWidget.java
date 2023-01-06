package frc.bd_util.widgets;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class BooleanWidget {
    ShuffleboardTab tab;
    SimpleWidget widget;
    
    public BooleanWidget(ShuffleboardTab tab, String name, boolean default_val) {
        this.tab = tab;
        this.widget = tab.add(name, default_val)
            .withWidget(BuiltInWidgets.kToggleButton);
    }

    public BooleanWidget(ShuffleboardTab tab, String name, Object[] settings) {
        this.tab = tab;
        this.widget = tab.add(name, (boolean)settings[0])
            .withWidget(BuiltInWidgets.kToggleButton);
    }

    public BooleanWidget(ShuffleboardTab tab, NetworkTable table, String name, Object[] settings) {
        this.tab = tab;
        if((boolean)settings[1]) {
            this.widget = tab.add(name, (boolean)settings[0])
                .withWidget(BuiltInWidgets.kToggleButton);
            widget.getEntry()
            .andThen(event -> {
                table.getEntry(name).setBoolean(event.getBoolean());
            });
        } else {
            this.widget = tab.add(name, (boolean)settings[0])
            .withWidget(BuiltInWidgets.kBooleanBox);
        }
    }


}
