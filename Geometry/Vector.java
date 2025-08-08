package Geometry;

import java.util.Arrays;

public class Vector {
    private double[] components;

    public Vector(int dimension) {
        if (dimension <= 0) throw new IllegalArgumentException("Dimension must be positive.");
        components = new double[dimension];
    }

    public Vector(double... components) {
        if (components == null || components.length == 0)
            throw new IllegalArgumentException("Components cannot be null or empty.");
        this.components = components.clone();
    }

    public int getDimension() {
        return components.length;
    }

    public double get(int index) {
        return components[index];
    }

    public void set(int index, double value) {
        components[index] = value;
    }

    /** Non-mutating vector addition: returns new Vector = this + other */
    public Vector add(Vector other) {
        checkDimensionMatch(other);
        double[] result = new double[getDimension()];
        for (int i = 0; i < getDimension(); i++)
            result[i] = this.components[i] + other.components[i];
        return new Vector(result);
    }

    /** Mutating vector addition: this += other */
    public Vector addInPlace(Vector other) {
        checkDimensionMatch(other);
        for (int i = 0; i < getDimension(); i++)
            this.components[i] += other.components[i];
        return this;
    }

    /** Non-mutating vector subtraction: returns new Vector = this - other */
    public Vector subtract(Vector other) {
        checkDimensionMatch(other);
        double[] result = new double[getDimension()];
        for (int i = 0; i < getDimension(); i++)
            result[i] = this.components[i] - other.components[i];
        return new Vector(result);
    }

    /** Mutating vector subtraction: this -= other */
    public Vector subtractInPlace(Vector other) {
        checkDimensionMatch(other);
        for (int i = 0; i < getDimension(); i++)
            this.components[i] -= other.components[i];
        return this;
    }

    /** Scalar multiply (non-mutating): returns new Vector = this * scalar */
    public Vector scale(double scalar) {
        double[] result = new double[getDimension()];
        for (int i = 0; i < getDimension(); i++)
            result[i] = this.components[i] * scalar;
        return new Vector(result);
    }

    /** Scalar multiply (mutating): this *= scalar */
    public Vector scaleInPlace(double scalar) {
        for (int i = 0; i < getDimension(); i++)
            components[i] *= scalar;
        return this;
    }

    /** Scalar divide (non-mutating): returns new Vector = this / scalar */
    public Vector divide(double scalar) {
        if (scalar == 0) throw new ArithmeticException("Division by zero");
        return scale(1.0 / scalar);
    }

    /** Scalar divide (mutating): this /= scalar */
    public Vector divideInPlace(double scalar) {
        if (scalar == 0) throw new ArithmeticException("Division by zero");
        return scaleInPlace(1.0 / scalar);
    }

    /** Dot product */
    public double dot(Vector other) {
        checkDimensionMatch(other);
        double sum = 0;
        for (int i = 0; i < getDimension(); i++)
            sum += components[i] * other.components[i];
        return sum;
    }

    /** Magnitude squared */
    public double magnitudeSquared() {
        double sum = 0;
        for (double c : components)
            sum += c * c;
        return sum;
    }

    /** Magnitude */
    public double magnitude() {
        return Math.sqrt(magnitudeSquared());
    }

    /** Returns a normalized (unit) vector, non-mutating */
    public Vector normalize() {
        double mag = magnitude();
        if (mag == 0) throw new ArithmeticException("Cannot normalize zero vector");
        return divide(mag);
    }

    /** Normalizes this vector in place */
    public Vector normalizeInPlace() {
        double mag = magnitude();
        if (mag == 0) throw new ArithmeticException("Cannot normalize zero vector");
        return divideInPlace(mag);
    }

    /** Cross product for 3D vectors */
    public Vector cross(Vector other) {
        if (getDimension() != 3 || other.getDimension() != 3)
            throw new UnsupportedOperationException("Cross product is defined only for 3D vectors");
        double x = components[1] * other.components[2] - components[2] * other.components[1];
        double y = components[2] * other.components[0] - components[0] * other.components[2];
        double z = components[0] * other.components[1] - components[1] * other.components[0];
        return new Vector(x, y, z);
    }

    /** Creates and returns a copy of this vector */
    public Vector copy() {
        return new Vector(components);
    }

    private void checkDimensionMatch(Vector other) {
        if (other == null)
            throw new IllegalArgumentException("Other vector is null");
        if (this.getDimension() != other.getDimension())
            throw new IllegalArgumentException("Dimension mismatch: this=" + getDimension() + ", other=" + other.getDimension());
    }

    public Vector multiply(Matrix matrix) {
        if (this.getDimension() != matrix.getRows()) {
            throw new IllegalArgumentException("Math.Vector length and matrix row count must match.");
        }

        int resultLength = matrix.getCols();
        double[] result = new double[resultLength];

        for (int col = 0; col < resultLength; col++) {
            for (int i = 0; i < this.getDimension(); i++) {
                result[col] += this.get(i) * matrix.get(i, col);
            }
        }

        return new Vector(result);
    }

    public double distance(Vector other) {
        return subtract(other).magnitude();
    }

    public Vector projectOnto(Vector other) {
        double scalar = dot(other) / other.magnitudeSquared();
        return other.scale(scalar);
    }

    @Override
    public String toString() {
        return "Math.Vector" + Arrays.toString(components);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Vector)) return false;
        Vector other = (Vector) o;
        return Arrays.equals(this.components, other.components);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(components);
    }
}
