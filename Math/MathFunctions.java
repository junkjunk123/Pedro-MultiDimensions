package Math;

public class MathFunctions {
    public static double fallingFactorial(int a, int b) {
        double result = 1;
        for (int i = 0; i < b; i++) {
            result *= (a - i);
        }
        return result;
    }
}
