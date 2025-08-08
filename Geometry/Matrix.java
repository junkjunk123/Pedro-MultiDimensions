package Geometry;

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

    /**
     * Multiplies every entry in the matrix by the given scalar.
     *
     * @param scalar The scalar to multiply by.
     * @return A new Matrix representing the scaled result.
     */
    public Matrix scale(double scalar) {
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = this.data[i][j] * scalar;
            }
        }
        return new Matrix(result);
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

        public int getDimension() {
            return dimension;
        }

        public SkewSymmetricMatrix log() {
            int n = getRows();
            Matrix I = Matrix.identity(n);
            Matrix X = this.subtract(I);  // (R - I)
            Matrix term = X.copy();
            Matrix result = X.copy();

            for (int k = 2; k < 50; k++) {
                term = term.multiply(X);
                Matrix increment = term.scale((k % 2 == 0 ? -1.0 : 1.0) / k);
                result = result.add(increment);

                if (increment.frobeniusNorm() < 1e-10) break;
            }

            return new SkewSymmetricMatrix(result.get());
        }

        public RotationMatrix multiply(RotationMatrix other) {
            return (RotationMatrix) super.multiply(other);
        }

        public RotationMatrix transpose() {
            return (RotationMatrix) super.transpose();
        }

        public RotationMatrix scale(double scalar) {
            return (RotationMatrix) super.scale(scalar);
        }
    }

    public static class SkewSymmetricMatrix extends Matrix {
        private final int dimension;

        /**
         * Constructs a zero skew-symmetric matrix of given dimension.
         */
        public SkewSymmetricMatrix(int dimension) {
            super(dimension, dimension);
            this.dimension = dimension;
        }

        /**
         * Constructs a skew-symmetric matrix from a 2D array.
         * Throws if the matrix is not skew-symmetric.
         */
        public SkewSymmetricMatrix(double[][] data) {
            super(data);
            if (getRows() != getCols()) {
                throw new IllegalArgumentException("Skew-symmetric matrix must be square.");
            }
            this.dimension = getRows();

            // Validate skew-symmetry
            for (int i = 0; i < dimension; i++) {
                if (Math.abs(get(i, i)) > 1e-9) {
                    throw new IllegalArgumentException("Diagonal of skew-symmetric matrix must be zero.");
                }
                for (int j = i + 1; j < dimension; j++) {
                    if (Math.abs(get(i, j) + get(j, i)) > 1e-9) {
                        throw new IllegalArgumentException("Matrix is not skew-symmetric at (" + i + ", " + j + ").");
                    }
                }
            }
        }

        /**
         * Returns the dimension (same as rows and cols)
         */
        public int getDimension() {
            return dimension;
        }

        /**
         * Sets the off-diagonal (i, j) entry and enforces skew-symmetry.
         * You must not set diagonal entries to nonzero values.
         */
        @Override
        public void set(int i, int j, double value) {
            if (i == j && Math.abs(value) > 1e-9) {
                throw new IllegalArgumentException("Diagonal entries of a skew-symmetric matrix must be zero.");
            }
            super.set(i, j, value);
            super.set(j, i, -value);
        }

        /**
         * Factory method to construct from a Geometry.Matrix (if skew-symmetric).
         */
        public static SkewSymmetricMatrix from(Matrix matrix) {
            return new SkewSymmetricMatrix(matrix.get());
        }

        public RotationMatrix exp() {
            int n = getRows();
            Matrix term = Matrix.identity(n);   // A^0 / 0!
            Matrix result = Matrix.identity(n); // Start with I

            Matrix A = this;
            double factorial = 1.0;

            for (int k = 1; k < 20; k++) {
                term = term.multiply(A);
                factorial *= k;
                Matrix increment = term.scale(1.0 / factorial);
                result = result.add(increment);

                if (increment.frobeniusNorm() < 1e-10) break;
            }

            return new RotationMatrix(result.get());
        }

        public SkewSymmetricMatrix transpose() {
            return (SkewSymmetricMatrix) super.transpose();
        }

        public SkewSymmetricMatrix scale(double scalar) {
            return (SkewSymmetricMatrix) super.scale(scalar);
        }
    }

    public double frobeniusNorm() {
        return Math.sqrt(frobeniusInnerProduct(this));
    }

    public double frobeniusInnerProduct(Matrix other) {
        return transpose().multiply(other).trace();
    }

    public double trace() {
        double sum = 0;

        for (int i = 0; i < rows; i++) {
            sum += data[i][i];
        }

        return sum;
    }

    public Matrix copy() {
        return new Matrix(this.get());
    }

    public static Matrix identity(int n) {
        Matrix I = new Matrix(n, n);
        for (int i = 0; i < n; i++) {
            I.set(i, i, 1.0);
        }
        return I;
    }

    public Matrix subtract(Matrix other) {
        return this.add(other.scale(-1));
    }
}
