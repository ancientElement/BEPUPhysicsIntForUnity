﻿using BEPUutilities;
using FixMath.NET;

namespace BEPUphysics.Paths
{
    /// <summary>
    /// Wrapper around an orientation curve that specifies a specific velocity at which to travel.
    /// </summary>
    public class ConstantAngularSpeedCurve : ConstantSpeedCurve<Quaternion>
    {
        /// <summary>
        /// Constructs a new constant speed curve.
        /// </summary>
        /// <param name="speed">Speed to maintain while traveling around a curve.</param>
        /// <param name="curve">Curve to wrap.</param>
        public ConstantAngularSpeedCurve(Fix64 speed, Curve<Quaternion> curve)
            : base(speed, curve)
        {
        }

        /// <summary>
        /// Constructs a new constant speed curve.
        /// </summary>
        /// <param name="speed">Speed to maintain while traveling around a curve.</param>
        /// <param name="curve">Curve to wrap.</param>
        /// <param name="sampleCount">Number of samples to use when constructing the wrapper curve.
        /// More samples increases the accuracy of the speed requirement at the cost of performance.</param>
        public ConstantAngularSpeedCurve(Fix64 speed, Curve<Quaternion> curve, int sampleCount)
            : base(speed, curve, sampleCount)
        {
        }

        protected override Fix64 GetDistance(Quaternion start, Quaternion end)
        {
            Quaternion.Conjugate(ref end, out end);
            Quaternion.Multiply(ref end, ref start, out end);
            return Quaternion.GetAngleFromQuaternion(ref end);
        }
    }
}