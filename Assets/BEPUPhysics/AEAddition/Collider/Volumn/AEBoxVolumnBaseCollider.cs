using FixMath.NET;
using BEPUphysics;
using UnityEngine;
using UnityEditor;
using BEPUphysics.Entities;
using BEPUphysics.Entities.Prefabs;
using UnityEngine.Serialization;

namespace AE_BEPUPhysics_Addition
{
    public class AEBoxVolumnBaseCollider : BaseVolumnBaseCollider
    {
        private BEPUphysics.Entities.Prefabs.Box m_box;

        [FormerlySerializedAs("m_length")] [SerializeField]
        private float length;

        [FormerlySerializedAs("m_width")] [SerializeField]
        private float width;

        [FormerlySerializedAs("m_height")] [SerializeField]
        private float height;

        public float Length
        {
            get => length;
            set
            {
                length = value;
                if (m_box != null)
                {
                    m_box.Length = value.ToFix64();
                }
            }
        }

        public float Width
        {
            get => width;
            set
            {
                width = value;
                if (m_box != null)
                {
                    m_box.Width = value.ToFix64();
                }
            }
        }

        public float Height
        {
            get => height;
            set
            {
                height = value;
                if (m_box != null)
                {
                    m_box.Height = value.ToFix64();
                }
            }
        }

        protected override Entity OnCreateEnity()
        {
            if (IsStatic)
            {
                m_box = new Box(transform.position.ToFix64(),
                    (width * transform.lossyScale.x).ToFix64(),
                    (height * transform.lossyScale.y).ToFix64(),
                    (length * transform.lossyScale.z).ToFix64());
            }
            else
            {
                m_box = new Box(transform.position.ToFix64(),
                    (width * transform.lossyScale.x).ToFix64(),
                    (height * transform.lossyScale.y).ToFix64(),
                    (length * transform.lossyScale.z).ToFix64(),
                    Mass.ToFix64());
            }

            return m_box;
        }

        protected override Entity GetEntity()
        {
            return m_box;
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            Vector3 halfExtents = new Vector3(width, height, length);
            // // 获取原始变换的位置、旋转和缩放信息
            // Vector3 position = transform.position;
            // Quaternion rotation = transform.rotation;
            // Vector3 scale = new Vector3(1, 1, 1); // 将缩放设置为 (1, 1, 1)
            // // 构建一个新的局部到世界的变换矩阵，缩放为 (1, 1, 1)
            // Matrix4x4 newLocalToWorldMatrix = Matrix4x4.TRS(position, rotation, scale);
            Matrix4x4 newLocalToWorldMatrix = transform.localToWorldMatrix;
            Gizmos.matrix = newLocalToWorldMatrix;
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(Vector3.zero, halfExtents);
        }
#endif
    }
}