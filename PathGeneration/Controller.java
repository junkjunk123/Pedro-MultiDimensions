package PathGeneration;

public interface Controller<T> {
    double driveToState(T currentState, T targetState);
    void reset();
}
