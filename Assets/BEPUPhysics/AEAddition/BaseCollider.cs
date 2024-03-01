using UnityEngine;

namespace AE_BEPUPhysics_Addition
{
    public abstract class BaseCollider : MonoBehaviour
    {
        public bool IsStatic;

        public float CenterX; //与transform的相对位置
        public float CenterY; //与transform的相对位置
        public float CenterZ; //与transform的相对位置

        protected bool m_inited;
    }
}