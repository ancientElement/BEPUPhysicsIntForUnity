using System;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using FixMath.NET;
using Unity.Collections;
using UnityEngine;

namespace AE_BEPUPhysics_Addition
{
    public class AECylinderVolumnBaseCollider : BaseVolumnBaseCollider
    {
        private Cylinder m_cylinder;
        [SerializeField] private float m_height;
        [SerializeField] private float m_radius;

        public float Height
        {
            get { return m_height; }
            set
            {
                m_height = value;
                if (m_cylinder != null)
                {
                    m_cylinder.Height = value.ToFix64();
                }
            }
        }

        public float Radius
        {
            get { return m_radius; }
            set
            {
                m_radius = value;
                if (m_cylinder != null)
                {
                    m_cylinder.Radius = value.ToFix64();
                }
            }
        }
        
        protected override Entity OnCreateEnity()
        {
            if (IsStatic)
            {
                m_cylinder = new Cylinder(transform.position.ToFix64(), (m_height * transform.lossyScale.y).ToFix64(),
                    (m_radius * Mathf.Max(transform.lossyScale.x, transform.lossyScale.z)).ToFix64());
            }
            else
            {
                m_cylinder = new Cylinder(transform.position.ToFix64(), (m_height * transform.lossyScale.y).ToFix64(),
                    (m_radius * Mathf.Max(transform.lossyScale.x, transform.lossyScale.z)).ToFix64(), Mass.ToFix64());
            }

            return m_cylinder;
        }

        protected override Entity GetEntity()
        {
            return m_cylinder;
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            float error = 0.005f;
            float radius = m_radius * Mathf.Max(transform.lossyScale.x, transform.lossyScale.z) + error;
            var height = m_height * transform.lossyScale.y;

            Gizmos.color = Color.green;
            Vector3 upCenter = transform.position + transform.up * height / 2f;
            Vector3 downCenter = transform.position - transform.up * height / 2f;
            AEGizmosDraw.DrawWireCircle(upCenter, transform.right, transform.forward, radius, 360f);
            AEGizmosDraw.DrawWireCircle(downCenter, transform.right, transform.forward, radius, 360f);
            AEGizmosDraw.DrawWireCircle(transform.position, transform.right, transform.forward, radius, 360f);
            Gizmos.DrawLine(upCenter + transform.right * radius, downCenter + transform.right * radius);
            Gizmos.DrawLine(upCenter - transform.right * radius, downCenter - transform.right * radius);
            Gizmos.DrawLine(upCenter - transform.forward * radius, downCenter - transform.forward * radius);
            Gizmos.DrawLine(upCenter - transform.forward * radius, downCenter - transform.forward * radius);
        }
#endif
    }
}