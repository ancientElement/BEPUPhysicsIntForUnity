﻿

using BEPUutilities;
using FixMath.NET;

namespace BEPUphysics.Paths
{
    /// <summary>
    /// Cardinal spline implementation of the 3D hermite curve.  Uses a tension parameter to control
    /// the tightness of the curve.  When tension is zero, a cardinal spline acts like a Catmull-Rom spline.
    /// </summary>
    public class CardinalSpline3D : HermiteCurve3D
    {
        private Fix64 tension;

        /// <summary>
        /// Gets or sets the tension parameter of the cardinal spline.
        /// A value of 0 acts like a Catmull-Rom spline, while a 
        /// value of 1 produces 0-length tangents.
        /// </summary>
        public Fix64 Tension
        {
            get { return tension; }
            set { tension = MathHelper.Clamp(value, F64.C0, F64.C1); }
        }


        /// <summary>
        /// Gets the curve's bounding index information.
        /// </summary>
        /// <param name="minIndex">Index of the minimum control point in the active curve segment.</param>
        /// <param name="maxIndex">Index of the maximum control point in the active curve segment.</param>
        public override void GetCurveIndexBoundsInformation(out int minIndex, out int maxIndex)
        {
            if (ControlPoints.Count > 3)
            {
                minIndex = 1;
                maxIndex = ControlPoints.Count - 2;
            }
            else
            {
                minIndex = -1;
                maxIndex = -1;
            }
        }

        protected override void ComputeTangents()
        {
            tangents.Add(Vector3.Zero);
            for (int i = 1; i < ControlPoints.Count - 1; i++)
            {
                Vector3 tangent;
                Vector3 previous = ControlPoints[i - 1].Value;
                Vector3 next = ControlPoints[i + 1].Value;
                Vector3.Subtract(ref next, ref previous, out tangent);
                Vector3.Multiply(ref tangent, (Fix64)((F64.C1 - tension) / (ControlPoints[i + 1].Time - ControlPoints[i - 1].Time)), out tangent);
                tangents.Add(tangent);
            }
            tangents.Add(Vector3.Zero);
        }
    }
}