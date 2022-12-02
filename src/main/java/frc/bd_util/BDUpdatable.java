package frc.bd_util;

public interface BDUpdatable {
    public abstract void update();
    
    public abstract String getID();

    public abstract void updateStatus();

    public abstract String getStatus();
}
