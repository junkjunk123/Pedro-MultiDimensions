package PathGeneration;

public abstract class PIDController<T> implements Controller<T> {
    private double P, I, D;
    private long previousTime;
    private double previousError;
    private double sum = 0;

    public PIDController(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
    }

    public void updateConstants(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
    }

    @Override
    public double driveToState(T currentState, T targetState) {
        double error = convertToScalarError(currentState, targetState);
        long timestamp = System.nanoTime();
        double elapsedTime = (timestamp - previousTime) / 1e9;
        double proportional = P * error;
        double derivative = D * (error - previousError) / elapsedTime;
        sum += elapsedTime * error;
        double integral = I * sum;
        previousTime = timestamp;
        previousError = error;

        return proportional + integral + derivative;
    }

    public abstract double convertToScalarError(T current, T target);

    @Override
    public void reset() {
        previousTime = System.nanoTime();
        previousError = 0;
        sum = 0;
    }
}
