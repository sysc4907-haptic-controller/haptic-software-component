import std.datetime;
import std;

class Pid
{
    private double currentTarget;
    private MonoTime previousTime = MonoTime.zero();
    private double previousError = 0;
    private double previousIntegralTerm = 0;

    // Adjustment constants
    private double kp = 0.02;
    private double ki = 0.02;

    this(double target)
    {
        currentTarget = target;
    }

    // Update target current
    public void updateTarget(double target)
    {
        this.currentTarget = target;
    }

    // Calculates the control signal from the error, proportional and integral
    public double calculateControlSignal(double currentValue)
    {
        writeln("Current Target: " ~to!string(currentTarget));
        MonoTime currentTime = MonoTime.currTime;
        double currentError = currentTarget - currentValue;

        double integralTerm = previousIntegralTerm + (((currentTime - previousTime)
                .total!"seconds") / 2) * (currentError + previousError);
        double controlSignal = (currentError * kp) + (integralTerm * ki);

        if (controlSignal > 255)
        {
            controlSignal = 255;
        }
        else if (controlSignal < -255)
        {
            controlSignal = -255;
        }

        previousTime = currentTime;
        previousError = currentError;

        return controlSignal;
    }
}
