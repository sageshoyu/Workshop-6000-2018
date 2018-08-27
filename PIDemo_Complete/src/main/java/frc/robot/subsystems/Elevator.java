package frc.robot.subsystems;


public class Elevator{
    //Possible states in state machine
    public enum State {DISABLED, LOWERING, RUNNING}
    public State state;

    //Absolute goal
    private double goal;
    //Elevator position on robot startup is not necesserilly at zero. When elevator is zeroed, the new encoder "position" reading is noted
    //here to compensate
    private double offset;
    //Goal to feed controller after going through state machine
    private double filteredGoal;

    //Software elevator boundaries
    public static double kMaxHeight = 2.1;
    public static double kMinHeight = -0.2;

    /*Control loop constants HERE =========================================================================================================*/
    private static double Kp = 100;
    private static double Kd = 25;
    private static double dt = 0.01;
    private static double kZeroingVelocity = 0.05;
    /* ====================================================================================================================================*/

    private double lastError;

    public Elevator() {
        state = State.DISABLED;
    }

    //Used to get next output voltage for elevator
    public double update(double encoder, boolean limitTriggered, boolean enabled) {
        double kVoltageLimit = 0.0;
        switch(state) {
            case DISABLED:
                /*DISABLED code HERE*/
                kVoltageLimit = 0.0;
                if(enabled) {
                    state = State.LOWERING;
                    filteredGoal = encoder;
                }
                break;
                /*==================*/

            case LOWERING:
                /*LOWERING code HERE*/
                kVoltageLimit = 4.0;
                if(!enabled) {
                    state = State.DISABLED;
                } else if (limitTriggered) {
                    offset = -encoder;
                    state = State.RUNNING;
                } else {
                    filteredGoal -= kZeroingVelocity * dt;
                }
                /*==================*/

                break;
            case RUNNING:
                /*RUNNING code HERE*/
                kVoltageLimit = 12.0;
                filteredGoal = goal;
                if (!enabled) {
                    state = State.DISABLED;
                }
                if (filteredGoal > kMaxHeight) {
                    filteredGoal = kMaxHeight;
                } else if (filteredGoal < kMinHeight) {
                    filteredGoal = kMinHeight;
                }
                break;
                /*=================*/
        }
/* Control loop calculations HERE ============================================================================================*/
        double error = filteredGoal - (encoder + offset);
        double voltage = Kp*error + Kd*(error-lastError) / dt;
        lastError = error;
/*============================================================================================================================*/

        //Making sure we are staying within the bounds of our control loop
        if (Math.abs(voltage)>kVoltageLimit) {
            voltage = Math.abs(voltage)/voltage * kVoltageLimit;
        }
        return voltage;
    }

    //Getters and setters for setpoint setting
    public void setGoal(double goal) {
        this.goal = goal;
    }

    public double getGoal() {
        return goal;
    }
}
