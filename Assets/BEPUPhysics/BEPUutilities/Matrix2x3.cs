﻿



using FixMath.NET;

namespace BEPUutilities
{
    /// <summary>
    /// 2 row, 3 column matrix.
    /// </summary>
    public struct Matrix2x3
    {
        /// <summary>
        /// Value at row 1, column 1 of the matrix.
        /// </summary>
        public Fix64 M11;

        /// <summary>
        /// Value at row 1, column 2 of the matrix.
        /// </summary>
        public Fix64 M12;

        /// <summary>
        /// Value at row 1, column 2 of the matrix.
        /// </summary>
        public Fix64 M13;

        /// <summary>
        /// Value at row 2, column 1 of the matrix.
        /// </summary>
        public Fix64 M21;

        /// <summary>
        /// Value at row 2, column 2 of the matrix.
        /// </summary>
        public Fix64 M22;

        /// <summary>
        /// Value at row 2, column 3 of the matrix.
        /// </summary>
        public Fix64 M23;


        /// <summary>
        /// Constructs a new 2 row, 2 column matrix.
        /// </summary>
        /// <param name="m11">Value at row 1, column 1 of the matrix.</param>
        /// <param name="m12">Value at row 1, column 2 of the matrix.</param>
        /// <param name="m13">Value at row 1, column 3 of the matrix.</param>
        /// <param name="m21">Value at row 2, column 1 of the matrix.</param>
        /// <param name="m22">Value at row 2, column 2 of the matrix.</param>
        /// <param name="m23">Value at row 2, column 3 of the matrix.</param>
        public Matrix2x3(Fix64 m11, Fix64 m12, Fix64 m13, Fix64 m21, Fix64 m22, Fix64 m23)
        {
            M11 = m11;
            M12 = m12;
            M13 = m13;
            M21 = m21;
            M22 = m22;
            M23 = m23;
        }

        /// <summary>
        /// Adds the two matrices together on a per-element basis.
        /// </summary>
        /// <param name="a">First matrix to add.</param>
        /// <param name="b">Second matrix to add.</param>
        /// <param name="result">Sum of the two matrices.</param>
        public static void Add(ref Matrix2x3 a, ref Matrix2x3 b, out Matrix2x3 result)
        {
            Fix64 m11 = a.M11 + b.M11;
            Fix64 m12 = a.M12 + b.M12;
            Fix64 m13 = a.M13 + b.M13;

            Fix64 m21 = a.M21 + b.M21;
            Fix64 m22 = a.M22 + b.M22;
            Fix64 m23 = a.M23 + b.M23;

            result.M11 = m11;
            result.M12 = m12;
            result.M13 = m13;

            result.M21 = m21;
            result.M22 = m22;
            result.M23 = m23;
        }


        /// <summary>
        /// Multiplies the two matrices.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void Multiply(ref Matrix2x3 a, ref Matrix3x3 b, out Matrix2x3 result)
        {
            Fix64 resultM11 = a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31;
            Fix64 resultM12 = a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32;
            Fix64 resultM13 = a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33;

            Fix64 resultM21 = a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31;
            Fix64 resultM22 = a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32;
            Fix64 resultM23 = a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33;

            result.M11 = resultM11;
            result.M12 = resultM12;
            result.M13 = resultM13;

            result.M21 = resultM21;
            result.M22 = resultM22;
            result.M23 = resultM23;
        }

        /// <summary>
        /// Multiplies the two matrices.
        /// </summary>
        /// <param name="a">First matrix to multiply.</param>
        /// <param name="b">Second matrix to multiply.</param>
        /// <param name="result">Product of the multiplication.</param>
        public static void Multiply(ref Matrix2x3 a, ref Matrix b, out Matrix2x3 result)
        {
            Fix64 resultM11 = a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31;
            Fix64 resultM12 = a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32;
            Fix64 resultM13 = a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33;

            Fix64 resultM21 = a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31;
            Fix64 resultM22 = a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32;
            Fix64 resultM23 = a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33;

            result.M11 = resultM11;
            result.M12 = resultM12;
            result.M13 = resultM13;

            result.M21 = resultM21;
            result.M22 = resultM22;
            result.M23 = resultM23;
        }

        /// <summary>
        /// Negates every element in the matrix.
        /// </summary>
        /// <param name="matrix">Matrix to negate.</param>
        /// <param name="result">Negated matrix.</param>
        public static void Negate(ref Matrix2x3 matrix, out Matrix2x3 result)
        {
            Fix64 m11 = -matrix.M11;
            Fix64 m12 = -matrix.M12;
            Fix64 m13 = -matrix.M13;

            Fix64 m21 = -matrix.M21;
            Fix64 m22 = -matrix.M22;
            Fix64 m23 = -matrix.M23;

            result.M11 = m11;
            result.M12 = m12;
            result.M13 = m13;

            result.M21 = m21;
            result.M22 = m22;
            result.M23 = m23;
        }

        /// <summary>
        /// Subtracts the two matrices from each other on a per-element basis.
        /// </summary>
        /// <param name="a">First matrix to subtract.</param>
        /// <param name="b">Second matrix to subtract.</param>
        /// <param name="result">Difference of the two matrices.</param>
        public static void Subtract(ref Matrix2x3 a, ref Matrix2x3 b, out Matrix2x3 result)
        {
            Fix64 m11 = a.M11 - b.M11;
            Fix64 m12 = a.M12 - b.M12;
            Fix64 m13 = a.M13 - b.M13;

            Fix64 m21 = a.M21 - b.M21;
            Fix64 m22 = a.M22 - b.M22;
            Fix64 m23 = a.M23 - b.M23;

            result.M11 = m11;
            result.M12 = m12;
            result.M13 = m13;

            result.M21 = m21;
            result.M22 = m22;
            result.M23 = m23;
        }


        /// <summary>
        /// Transforms the vector by the matrix.
        /// </summary>
        /// <param name="v">Vector2 to transform.  Considered to be a row vector for purposes of multiplication.</param>
        /// <param name="matrix">Matrix to use as the transformation.</param>
        /// <param name="result">Row vector product of the transformation.</param>
        public static void Transform(ref Vector2 v, ref Matrix2x3 matrix, out Vector3 result)
        {
#if !WINDOWS
            result = new Vector3();
#endif
            result.X = v.X * matrix.M11 + v.Y * matrix.M21;
            result.Y = v.X * matrix.M12 + v.Y * matrix.M22;
            result.Z = v.X * matrix.M13 + v.Y * matrix.M23;
        }

        /// <summary>
        /// Transforms the vector by the matrix.
        /// </summary>
        /// <param name="v">Vector2 to transform.  Considered to be a column vector for purposes of multiplication.</param>
        /// <param name="matrix">Matrix to use as the transformation.</param>
        /// <param name="result">Column vector product of the transformation.</param>
        public static void Transform(ref Vector3 v, ref Matrix2x3 matrix, out Vector2 result)
        {
#if !WINDOWS
            result = new Vector2();
#endif
            result.X = matrix.M11 * v.X + matrix.M12 * v.Y + matrix.M13 * v.Z;
            result.Y = matrix.M21 * v.X + matrix.M22 * v.Y + matrix.M23 * v.Z;
        }


        /// <summary>
        /// Computes the transposed matrix of a matrix.
        /// </summary>
        /// <param name="matrix">Matrix to transpose.</param>
        /// <param name="result">Transposed matrix.</param>
        public static void Transpose(ref Matrix2x3 matrix, out Matrix3x2 result)
        {
            result.M11 = matrix.M11;
            result.M12 = matrix.M21;

            result.M21 = matrix.M12;
            result.M22 = matrix.M22;

            result.M31 = matrix.M13;
            result.M32 = matrix.M23;
        }


        /// <summary>
        /// Creates a string representation of the matrix.
        /// </summary>
        /// <returns>A string representation of the matrix.</returns>
        public override string ToString()
        {
            return "{" + M11 + ", " + M12 + ", " + M13 + "} " +
                   "{" + M21 + ", " + M22 + ", " + M23 + "}";
        }
    }
}