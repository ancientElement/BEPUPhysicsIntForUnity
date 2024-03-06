using System.Collections.Generic;
using FixMath.NET;

namespace AE_BEPUPhysics_Addition
{
    public static class AEUtilities
    {
        public static Fix64 ToFix64(this float f)
        {
            return System.Convert.ToDecimal(f);
        }
        public static float ToFloat(this Fix64 f)
        {
            return (float)f;
        }

        public static UnityEngine.Vector3 ToFloat(this BEPUutilities.Vector3 v3)
        {
            return new UnityEngine.Vector3(v3.X.ToFloat(), v3.Y.ToFloat(), v3.Z.ToFloat());
        }
        public static BEPUutilities.Vector3 ToFix64(this UnityEngine.Vector3 v3)
        {
            return new BEPUutilities.Vector3(v3.x.ToFix64(), v3.y.ToFix64(), v3.z.ToFix64());
        }
        
        
        public static BEPUutilities.Matrix ToFix64(this UnityEngine.Matrix4x4 m)
        {
            return new BEPUutilities.Matrix(
                m.m00.ToFix64(), m.m01.ToFix64(), m.m02.ToFix64(), m.m03.ToFix64(),
                m.m10.ToFix64(), m.m11.ToFix64(), m.m12.ToFix64(), m.m13.ToFix64(),
                m.m20.ToFix64(), m.m21.ToFix64(), m.m22.ToFix64(), m.m23.ToFix64(),
                m.m30.ToFix64(), m.m31.ToFix64(), m.m32.ToFix64(), m.m33.ToFix64());
        }
        
        public static BEPUutilities.Quaternion ToFix64(this UnityEngine.Quaternion q)
        {
            return new BEPUutilities.Quaternion(q.x.ToFix64(), q.y.ToFix64(), q.z.ToFix64(), q.w.ToFix64());
        }
        public static UnityEngine.Quaternion ToFloat(this BEPUutilities.Quaternion q)
        {
            return new UnityEngine.Quaternion(q.X.ToFloat(), q.Y.ToFloat(), q.Z.ToFloat(), q.W.ToFloat());
        }

        public static IList<BEPUutilities.Vector3> ToFix64(this IList<UnityEngine.Vector3> l)
        {
            List<BEPUutilities.Vector3> newl = new List<BEPUutilities.Vector3>();
            foreach (var item in l)
            {
                newl.Add(item.ToFix64());
            }

            return newl;
        }
    }
}