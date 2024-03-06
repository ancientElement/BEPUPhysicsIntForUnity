using System.Collections.Generic;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using FixMath.NET;
using UnityEngine;
using UnityEngine.Serialization;

namespace AE_BEPUPhysics_Addition
{
    public class AESphereVolumnBaseCollider : BaseVolumnBaseCollider
    {
        private BEPUphysics.Entities.Prefabs.Sphere m_sphere;

        [FormerlySerializedAs("m_radius")] [SerializeField]
        private float radius;

        public float Radius
        {
            get => radius;
            set
            {
                radius = value;
                if (m_sphere != null)
                {
                    m_sphere.Radius = value.ToFix64();
                }
            }
        }

        protected override Entity OnCreateEnity()
        {
            if (IsStatic)
            {
                m_sphere = new Sphere(transform.position.ToFix64(), radius.ToFix64());
            }
            else
            {
                m_sphere = new Sphere(transform.position.ToFix64(), radius.ToFix64(), Mass.ToFix64());
            }

            return m_sphere;
        }

        protected override Entity GetEntity()
        {
            return m_sphere;
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            //这里的center是相对目标中心而言，因为旋转cube与目标位置相同所以是zero
            float error = 0.005f;
            Gizmos.color = Color.green;
            var pos1 = transform.position;
            AEGizmosDraw.DrawWireCircle(pos1, transform.up, transform.forward, radius + error, 360f);
            AEGizmosDraw.DrawWireCircle(pos1, transform.up, transform.right, radius + error, 360f);
            AEGizmosDraw.DrawWireCircle(pos1, transform.forward, transform.right, radius + error, 360f);
        }
#endif
    }
}