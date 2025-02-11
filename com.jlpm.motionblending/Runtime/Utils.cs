using Unity.Mathematics;

namespace MotionBlending
{
    public static class Utils
    {
        public static float HalfLifeToDamping(float halfLife, float eps = 1e-5f)
        {
            const float LN2f = 0.69314718056f;
            return (4.0f * LN2f) / (halfLife + eps);
        }

        public static float FastNEgeExp(float x)
        {
            return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
        }

        /// <summary>
        /// Quaternion absolute forces the quaternion to take the shortest path
        /// </summary>
        public static quaternion Abs(this quaternion q)
        {
            return q.value.w < 0.0f ? new quaternion(-q.value.x, -q.value.y, -q.value.z, -q.value.w) : q;
        }

        public static float3 ToScaledAngleAxis(this quaternion q, float eps = 1e-8f)
        {
            return 2.0f * q.Log(eps);
        }

        public static quaternion ToQuaternion(this float3 scaledAngleAxis, float eps = 1e-8f)
        {
            return Exp(scaledAngleAxis * 0.5f, eps);
        }

        public static float3 Log(this quaternion q, float eps = 1e-8f)
        {
            float length = math.sqrt(q.value.x * q.value.x + q.value.y * q.value.y + q.value.z * q.value.z);
            if (length < eps)
            {
                return new float3(q.value.x, q.value.y, q.value.z);
            }
            else
            {
                float halfangle = math.acos(math.clamp(q.value.w, -1f, 1f));
                return halfangle * (new float3(q.value.x, q.value.y, q.value.z) / length);
            }
        }

        public static quaternion Exp(this float3 angleAxis, float eps = 1e-8f)
        {
            float halfangle = math.sqrt(angleAxis.x * angleAxis.x + angleAxis.y * angleAxis.y + angleAxis.z * angleAxis.z);
            if (halfangle < eps)
            {
                return math.normalize(new quaternion(angleAxis.x, angleAxis.y, angleAxis.z, 1f));
            }
            else
            {
                float c = math.cos(halfangle);
                float s = math.sin(halfangle) / halfangle;
                return new quaternion(s * angleAxis.x, s * angleAxis.y, s * angleAxis.z, c);
            }
        }
    }
}