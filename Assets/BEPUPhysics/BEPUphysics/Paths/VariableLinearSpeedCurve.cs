﻿

using BEPUutilities;
using FixMath.NET;

namespace BEPUphysics.Paths
{
    /// <summary>
    /// Wraps a curve that is traveled along with arbitrary defined linear speed.
    /// </summary>
    /// <remarks>
    /// The speed curve should be designed with the wrapped curve's times in mind.
    /// Speeds will be sampled based on the wrapped curve's interval.</remarks>
    public class VariableLinearSpeedCurve : VariableSpeedCurve<Vector3>
    {
        /// <summary>
        /// Constructs a new variable speed curve.
        /// </summary>
        /// <param name="speedCurve">Curve defining speeds to use.</param>
        /// <param name="curve">Curve to wrap.</param>
        public VariableLinearSpeedCurve(Path<Fix64> speedCurve, Curve<Vector3> curve)
            : base(speedCurve, curve)
        {
        }

        /// <summary>
        /// Constructs a new variable speed curve.
        /// </summary>
        /// <param name="speedCurve">Curve defining speeds to use.</param>
        /// <param name="curve">Curve to wrap.</param>
        /// <param name="sampleCount">Number of samples to use when constructing the wrapper curve.
        /// More samples increases the accuracy of the speed requirement at the cost of performance.</param>
        public VariableLinearSpeedCurve(Path<Fix64> speedCurve, Curve<Vector3> curve, int sampleCount)
            : base(speedCurve, curve, sampleCount)
        {
        }

        protected override Fix64 GetDistance(Vector3 start, Vector3 end)
        {
            Fix64 distance;
            Vector3.Distance(ref start, ref end, out distance);
            return distance;
        }
    }
}