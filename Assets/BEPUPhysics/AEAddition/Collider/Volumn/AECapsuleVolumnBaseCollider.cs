using System;
using System.Collections.Generic;
using BEPUphysics.Constraints.TwoEntity.Joints;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using FixMath.NET;
using TMPro;
using UnityEditor;
using UnityEngine;
using UnityEngine.Android;
using UnityEngine.Serialization;

namespace AE_BEPUPhysics_Addition
{
    public class AECapsuleVolumnBaseCollider : BaseVolumnBaseCollider
    {
        private BEPUphysics.Entities.Prefabs.Capsule m_capsule;
        [SerializeField] private float m_halfLength;
        [SerializeField] private float m_radius;

        public float HalfLength
        {
            get { return m_halfLength; }
            set
            {
                m_halfLength = value;
                if (m_capsule != null)
                {
                    m_capsule.Length = value.ToFix64();
                }
            }
        }

        public float Radius
        {
            get { return m_radius; }
            set
            {
                m_radius = value;
                if (m_capsule != null)
                {
                    m_capsule.Radius = value.ToFix64();
                }
            }
        }

        protected override Entity OnCreateEnity()
        {
            if (IsStatic)
            {
                m_capsule = new Capsule(transform.position.ToFix64(),
                    (m_halfLength * transform.lossyScale.y).ToFix64(),
                    (m_radius * Mathf.Max(transform.lossyScale.x, transform.lossyScale.z)).ToFix64());
            }
            else
            {
                m_capsule = new Capsule(transform.position.ToFix64(),
                    (m_halfLength * transform.lossyScale.y).ToFix64(),
                    (m_radius * Mathf.Max(transform.lossyScale.x, transform.lossyScale.z)).ToFix64(),
                    Mass.ToFix64());
            }

            return m_capsule;
        }

        protected override Entity GetEntity()
        {
            return m_capsule;
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            //这里的center是相对目标中心而言，因为旋转cube与目标位置相同所以是zero
            float error = 0.005f;
            Gizmos.color = Color.green;

            var radius = m_radius * Mathf.Max(transform.lossyScale.x, transform.lossyScale.z) + error;

            var length = m_halfLength * transform.lossyScale.y;

            AEGizmosDraw.DrawWirePlaneCapsule(transform.position, transform.forward, transform.up, radius,
                length * 2f);
            AEGizmosDraw.DrawWirePlaneCapsule(transform.position, transform.right, transform.up, radius,
                length * 2f);

            var pos1 = transform.position;
            pos1 += transform.up * (length - m_radius);
            AEGizmosDraw.DrawWireCircle(pos1, transform.right, transform.forward, radius, 360f);
            pos1 -= transform.up * 2f * (length - m_radius);
            AEGizmosDraw.DrawWireCircle(pos1, transform.right, transform.forward, radius, 360f);
        }
#endif
    }
}