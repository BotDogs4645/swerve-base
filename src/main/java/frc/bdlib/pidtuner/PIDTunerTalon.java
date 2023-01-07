package frc.bdlib.pidtuner;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class PIDTunerTalon {
    TalonFX tuning_motor;
    ShuffleboardTab subsystem_tab;
    int id;
    int modifier = 1;
    double CONVERSION_RATE = 600.0 / 2048.0;

    boolean sus_mode = false;
    boolean bench_on = false;
    public String cur_string = "none";
    HashMap<String, Double> save = new HashMap<String, Double>();
    double threshold = 5.0;
    SuppliedValueWidget<double[]> veloGraph;
    SuppliedValueWidget<Double> errorGraph;
    GenericEntry time_to_threshold_reporter;
    SimpleWidget benchWidget;
    SimpleWidget RPMDirect;
    SimpleWidget kPwidgetDirect;
    SimpleWidget kIwidgetDirect;
    SimpleWidget kDwidgetDirect;

    public PIDTunerTalon(TalonFX tuning_motor, ShuffleboardTab tab) {
        this.tuning_motor = tuning_motor;
        this.subsystem_tab = tab;
        this.id = tuning_motor.getDeviceID();
        initalize();
    }

    public void initalize() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        
        // RPM
        ShuffleboardLayout RPMlayout = subsystem_tab.getLayout("RPM Settings", BuiltInLayouts.kList)
        .withSize(4, 2)
        .withPosition(3, 3);

        this.RPMDirect = RPMlayout.add("RPM Control Direct", 1.0);

        inst.addListener(RPMDirect.getEntry(), EnumSet.of(Kind.kValueAll), event -> {
            if (!sus_mode) {
                tuning_motor.set(ControlMode.Velocity, event.valueData.value.getDouble() / CONVERSION_RATE);
            } else {
                RPMDirect.getEntry().setValue(save.get("RPM"));
            }
        });

        SimpleWidget inverter = RPMlayout.add("Invert Direction?", false)
            .withWidget(BuiltInWidgets.kToggleButton);

        inst.addListener(inverter.getEntry(), EnumSet.of(Kind.kValueAll), event -> {
            tuning_motor.setInverted(event.valueData.value.getBoolean());
        });
        
        // kP
        ShuffleboardLayout kPLayout = subsystem_tab.getLayout("kP Settings", BuiltInLayouts.kList)
        .withSize(2, 1)
        .withPosition(0, 0);

        this.kPwidgetDirect = kPLayout.add("kP Direct", 1.0);

        inst.addListener(kPwidgetDirect.getEntry(), EnumSet.of(Kind.kValueAll), event -> {
            if (!sus_mode) {
                tuning_motor.config_kP(0, event.valueData.value.getDouble(), 30);
            } else {
                kPwidgetDirect.getEntry().setValue(save.get("kP"));
            }
        });

        // kI
        ShuffleboardLayout kILayout = subsystem_tab.getLayout("kI Settings", BuiltInLayouts.kList)
        .withPosition(0, 1)
        .withSize(2,1);

        this.kIwidgetDirect = kILayout.add("kI Direct", 1.0);
        inst.addListener(kIwidgetDirect.getEntry(), EnumSet.of(Kind.kValueAll), event -> {
            if (!sus_mode) {
                tuning_motor.config_kI(0, event.valueData.value.getDouble(), 30);
            } else {
                kIwidgetDirect.getEntry().setDouble(save.get("kI"));
            }
        });

        // kD
        ShuffleboardLayout kDLayout = subsystem_tab.getLayout("kD Settings", BuiltInLayouts.kList)
        .withPosition(0, 2)
        .withSize(2, 1);

        this.kDwidgetDirect = kDLayout.add("kD Direct", 1.0);
        inst.addListener(kDwidgetDirect.getEntry(), EnumSet.of(Kind.kValueAll), event -> {
            if (!sus_mode) {
                tuning_motor.config_kD(0, event.valueData.value.getDouble(), 30);
            } else {
                kDwidgetDirect.getEntry().setDouble(save.get("kD"));
            }
        });
    
        // FF
        ShuffleboardLayout FFLayout = subsystem_tab.getLayout("FF Settings", BuiltInLayouts.kList)
        .withSize(2, 1)
        .withPosition(0, 3);
        
        SimpleWidget FFTune = FFLayout.add("FF Tune Direct", 1.0);

        inst.addListener(FFTune.getEntry(), EnumSet.of(Kind.kValueAll), event -> {
            if (!sus_mode) {
                tuning_motor.config_kF(0, event.valueData.value.getDouble(), 30);
            } else {
                FFTune.getEntry().setValue(save.get("FFTune"));
            }
        });
        
        graphSetups();

        ShuffleboardLayout benchmode_layout = subsystem_tab.getLayout("Bench Settings", BuiltInLayouts.kList)
        .withSize(2, 3)
        .withPosition(8, 0);

        SimpleWidget bench_mode = benchmode_layout.add("Benchmark Mode", false)
        .withWidget(BuiltInWidgets.kToggleButton);

        inst.addListener(bench_mode.getEntry(), EnumSet.of(Kind.kValueAll), event -> {
            if (DriverStation.isEnabled()) {
                sus_mode = event.valueData.value.getBoolean();
                if (bench_on || benchWidget.getEntry().getBoolean(false)) {
                    sus_mode = true;
                    bench_mode.getEntry().setBoolean(true);
                }
                if (sus_mode) {
                    save.put("RPM", RPMDirect.getEntry().getDouble(0.0));
                    save.put("kP", kPwidgetDirect.getEntry().getDouble(0.0));
                    save.put("kI", kIwidgetDirect.getEntry().getDouble(0.0));
                    save.put("kD", kDwidgetDirect.getEntry().getDouble(0.0));
                    save.put("FFTune", FFTune.getEntry().getDouble(0.0));
                    // ready to start motors on command
                    tuning_motor.set(ControlMode.PercentOutput, 0); // effectively turns off motor.
                }
            } else {
                bench_mode.getEntry().setBoolean(false);
                sus_mode = false;
            }
        });

        this.time_to_threshold_reporter = benchmode_layout.add("Time", 0.0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

        SimpleWidget thresholder = benchmode_layout.add("Threshold", 5.0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", .01, "max", 20) // threshold should not exceed 20%.. seriously.
        );

        inst.addListener(thresholder.getEntry(), EnumSet.of(Kind.kValueAll), event -> {
            if (sus_mode && !bench_on) {
                threshold = event.valueData.value.getDouble();
            } else if (!sus_mode) {
                thresholder.getEntry().setDouble(threshold);
            }
        });

        this.benchWidget = benchmode_layout.add("Begin Benchmark", false)
        .withWidget(BuiltInWidgets.kToggleButton);

        inst.addListener(benchWidget.getEntry(), EnumSet.of(Kind.kValueAll), event -> {
            if (sus_mode && !bench_on && event.valueData.value.getBoolean()) {
                bench_on = true;
                new ParallelCommandGroup(
                    new TalonPIDBenchmarker(this, tuning_motor, benchWidget, time_to_threshold_reporter))
                .execute();
            } else if (sus_mode && bench_on && !event.valueData.value.getBoolean()) {
                benchWidget.getEntry().setBoolean(true);
            } else {
                benchWidget.getEntry().setBoolean(false);
            }
        });

    }

    public void graphSetups() {
        this.veloGraph = subsystem_tab.addDoubleArray("Current Velocity", () -> getGraphSetpoints())
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("Visible time", 20, "Unit", "RPM")
        )
        .withPosition(2, 0);

        this.errorGraph = subsystem_tab.addNumber("Current Error", () -> tuning_motor.getErrorDerivative() * CONVERSION_RATE)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of("Visible time", 20, "Unit", "RPM")
        )
        .withPosition(5, 0);
    }

    public void setBenchValue(boolean value) {
        bench_on = value;
    }

    public double getRPMSetpoint() {
        return save.get("RPM");
    }

    public double[] getGraphSetpoints() {
        return new double[] {tuning_motor.getSelectedSensorVelocity() * CONVERSION_RATE, RPMDirect.getEntry().getDouble(0.0)};
    }


    public String getString() {
        return cur_string;
    }

    public double getThreshold() {
        return threshold;
    }
}