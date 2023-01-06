package frc.bd_util.widgets;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
/* settings: 
    Strings: name
    ints: min, max
    functions: connect2network tables -> port to networktable value
*/
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class NumberSliderWidget {
    ShuffleboardTab tab;
    SimpleWidget widget;

    public NumberSliderWidget(ShuffleboardTab tab, String name, int default_number, int min, int max) {
        this.tab = tab;
        this.widget = tab.add(name, default_number)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", min, "max", max)
        );
    }

    public NumberSliderWidget(ShuffleboardTab tab, String name, Object[] settings) {
        this.tab = tab;
        this.widget = tab.add(name, settings[0])
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", (int)settings[1], "max", (int)settings[2])
        );
    }

    //shuffleboard only nums
    public NumberSliderWidget(ShuffleboardTab tab, NetworkTable table, String name, Object[] settings) {
        this.tab = tab;
        this.widget = tab.add(name, (int)settings[0])
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", (int)settings[1], "max", (int)settings[2]));

        widget.getEntry()
        .andThen(event -> {
            table.getEntry(name).setValue(event.getValue());
        });
    }

    public SimpleWidget place(int x, int y) {
        return widget.withPosition(x, y);
    }

    public SimpleWidget resize(int x, int y) {
        return widget.withSize(x, y);
    }

    public GenericEntry getEntry() {
        return widget.getEntry();
    }
}