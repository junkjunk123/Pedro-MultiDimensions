package Math;

import java.util.Arrays;

public class Matrix {
    private final double[][] data;
    private final int rows;
    private final int cols;

    /**
     * Constructs a new Matrix from a 2D array of doubles.
     * A deep copy of the data is made to ensure immutability.
     *
     * @param data The 2D array representing the matrix.
     * @throws IllegalArgumentException if data is null, empty, or represents a jagged array.
     */
    public Matrix(double[][] data) {
        if (data == null || data.length == 0 || data[0].length == 0) {
            throw new IllegalArgumentException("Math.Matrix data cannot be null or empty.");
        }
        this.rows = data.length;
        this.cols = data[0].length;
        this.data = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            if (data[i].length != cols) {
                throw new IllegalArgumentException("All rows in the matrix must have the same length.");
            }
            System.arraycopy(data[i], 0, this.data[i], 0, cols);
        }
    }

    /**
     * Constructs a new Matrix with the given number of rows and columns.
     * @param rows The number of rows.
     * @param cols The number of columns.
     */
    public Matrix(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;
        this.data = new double[rows][cols];
    }

    /**
     * Gets the number of rows in the matrix.
     *
     * @return The number of rows.
     */
    public int getRows() {
        return rows;
    }

    /**
     * Gets the number of columns in the matrix.
     *
     * @return The number of columns.
     */
    public int getCols() {
        return cols;
    }

    /**
     * Gets the element at the specified row and column.
     *
     * @param row The row index.
     * @param col The column index.
     * @return The element at the specified position.
     * @throws IndexOutOfBoundsException if the row or col is out of range.
     */
    public double get(int row, int col) {
        return data[row][col];
    }

    /**
     * Adds another matrix to this matrix.
     *
     * @param other The matrix to add.
     * @return A new Matrix representing the sum.
     * @throws IllegalArgumentException if the matrices have different dimensions.
     */
    public Matrix add(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Matrices must have the same dimensions for addition.");
        }
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = this.data[i][j] + other.data[i][j];
            }
        }
        return new Matrix(result);
    }

    /**
     * Multiplies this matrix by another matrix.
     *
     * @param other The matrix to multiply by.
     * @return A new Matrix representing the product.
     * @throws IllegalArgumentException if the number of columns in this matrix does not equal the number of rows in the other matrix.
     */
    public Matrix multiply(Matrix other) {
        if (this.cols != other.rows) {
            throw new IllegalArgumentException("The number of columns of the first matrix must equal the number of rows of the second matrix.");
        }
        double[][] result = new double[this.rows][other.cols];
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                for (int k = 0; k < this.cols; k++) {
                    result[i][j] += this.data[i][k] * other.data[k][j];
                }
            }
        }
        return new Matrix(result);
    }

    /**
     * Multiplies this matrix by a vector.
     *
     * @param vector The vector to multiply by.
     * @return A new Vector representing the product.
     * @throws IllegalArgumentException if the number of columns in the matrix does not match the dimension of the vector.
     */
    public Vector multiply(Vector vector) {
        if (this.cols != vector.getDimension()) {
            throw new IllegalArgumentException("Math.Matrix columns must match vector dimension for multiplication.");
        }
        double[] result = new double[this.rows];
        for (int i = 0; i < this.rows; i++) {
            double sum = 0;
            for (int j = 0; j < this.cols; j++) {
                sum += this.data[i][j] * vector.get(j);
            }
            result[i] = sum;
        }
        return new Vector(result);
    }

    /**
     * Computes the transpose of this matrix.
     *
     * @return A new Matrix that is the transpose of this matrix.
     */
    public Matrix transpose() {
        double[][] result = new double[this.cols][this.rows];
        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < this.cols; j++) {
                result[j][i] = this.data[i][j];
            }
        }
        return new Matrix(result);
    }

    /**
     * Sets the value of a specific entry in the matrix
     * @param row the row of the entry to set
     * @param col the column of the entry to set
     * @param value the value to give to the entry
     */
    public void set(int row, int col, double value) {
        data[row][col] = value;
    }

    public double[][] get() {
        return data;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder("Math.Matrix[\n");
        for (int i = 0; i < rows; i++) {
            sb.append("  ").append(Arrays.toString(data[i]));
            if (i < rows - 1) {
                sb.append("\n");
            }
        }
        sb.append("\n]");
        return sb.toString();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Matrix matrix = (Matrix) o;
        return Arrays.deepEquals(data, matrix.data);
    }

    @Override
    public int hashCode() {
        return Arrays.deepHashCode(data);
    }

    public static class RotationMatrix extends Matrix{
        private final int dimension;

        // Private constructor
        private RotationMatrix(double[][] matrix) {
            super(matrix);
            this.dimension = matrix.length;
        }

        // Static constructor for 2D rotation
        public static RotationMatrix from2DAngle(double angleRadians) {
            double[][] m = {
                    {Math.cos(angleRadians), -Math.sin(angleRadians)},
                    {Math.sin(angleRadians),  Math.cos(angleRadians)}
            };
            return new RotationMatrix(m);
        }

        // Static constructor for 3D rotation around x-axis
        public static RotationMatrix from3DRotationX(double angleRadians) {
            double[][] m = {
                    {1, 0, 0},
                    {0, Math.cos(angleRadians), -Math.sin(angleRadians)},
                    {0, Math.sin(angleRadians),  Math.cos(angleRadians)}
            };
            return new RotationMatrix(m);
        }

        // Static constructor for 3D rotation around y-axis
        public static RotationMatrix from3DRotationY(double angleRadians) {
            double[][] m = {
                    { Math.cos(angleRadians), 0, Math.sin(angleRadians)},
                    {0, 1, 0},
                    {-Math.sin(angleRadians), 0, Math.cos(angleRadians)}
            };
            return new RotationMatrix(m);
        }

        // Static constructor for 3D rotation around z-axis
        public static RotationMatrix from3DRotationZ(double angleRadians) {
            double[][] m = {
                    {Math.cos(angleRadians), -Math.sin(angleRadians), 0},
                    {Math.sin(angleRadians),  Math.cos(angleRadians), 0},
                    {0, 0, 1}
            };
            return new RotationMatrix(m);
        }

        // Apply rotation to a vector
        public Vector apply(Vector v) {
            if (v.getDimension() != dimension) {
                throw new IllegalArgumentException("Math.Vector dimension must match rotation matrix.");
            }
            double[] result = new double[dimension];
            for (int i = 0; i < dimension; i++) {
                for (int j = 0; j < dimension; j++) {
                    result[i] += get(i, j) * v.get(j);
                }
            }
            return new Vector(result);
        }

        // Compose this rotation with another (this * other)
        public RotationMatrix compose(RotationMatrix other) {
            if (this.dimension != other.dimension) {
                throw new IllegalArgumentException("Rotation matrices must have the same dimension.");
            }

            double[][] result = new double[dimension][dimension];
            for (int i = 0; i < dimension; i++) {
                for (int j = 0; j < dimension; j++) {
                    for (int k = 0; k < dimension; k++) {
                        result[i][j] += get(i, k) * get()[k][j];
                    }
                }
            }

            return new RotationMatrix(result);
        }

        // Return the transpose (inverse for orthogonal rotation matrix)
        public RotationMatrix transpose() {
            double[][] result = new double[dimension][dimension];
            for (int i = 0; i < dimension; i++) {
                for (int j = 0; j < dimension; j++) {
                    result[i][j] = get(j, i);
                }
            }
            return new RotationMatrix(result);
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder("RotationMatrix[");
            for (int i = 0; i < dimension; i++) {
                sb.append("[");
                for (int j = 0; j < dimension; j++) {
                    sb.append(String.format("%.4f", get(i, j)));
                    if (j < dimension - 1) sb.append(", ");
                }
                sb.append("]");
                if (i < dimension - 1) sb.append(", ");
            }
            sb.append("]");
            return sb.toString();
        }
    }
}
