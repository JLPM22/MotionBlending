using System;
using System.Diagnostics;

namespace MotionBlending
{
    public class InertializationFloat
    {
        public float[] InertializedFloats;
        public float[] InertializedVelocities;

        private readonly float[] Offsets;
        private readonly float[] OffsetVelocities;

        public InertializationFloat(int num)
        {
            InertializedFloats = new float[num];
            InertializedVelocities = new float[num];
            Offsets = new float[num];
            OffsetVelocities = new float[num];
        }

        /// <summary>
        /// It sets up the inertialization, which can then by updated by calling Update(...).
        /// </summary>
        public void Transition(ReadOnlySpan<float> source, ReadOnlySpan<float> sourceVelocities,
                               ReadOnlySpan<float> target, ReadOnlySpan<float> targetVelocities)
        {
            Debug.Assert(source.Length == InertializedFloats.Length);
            Debug.Assert(sourceVelocities.Length == InertializedFloats.Length);
            Debug.Assert(target.Length == InertializedFloats.Length);
            Debug.Assert(targetVelocities.Length == InertializedFloats.Length);
            for (int i = 0; i < source.Length; i++)
            {
                InertializeTransition(source[i], sourceVelocities[i],
                                      target[i], targetVelocities[i],
                                      ref Offsets[i], ref OffsetVelocities[i]);
            }
        }

        /// <summary>
        /// Updates the inertialization decaying the offset from the
        /// source floats (specified in Transition(...)) to the target floats
        /// and applying it to the current target floats. 
        /// </summary>
        public void Update(ReadOnlySpan<float> currentTarget,
                           ReadOnlySpan<float> currentTargetVelocities,
                           float halfLife, float deltaTime)
        {
            Debug.Assert(currentTarget.Length == InertializedFloats.Length);
            Debug.Assert(currentTargetVelocities.Length == InertializedFloats.Length);
            for (int i = 0; i < currentTarget.Length; i++)
            {
                InertializeUpdate(currentTarget[i], currentTargetVelocities[i],
                                  halfLife, deltaTime,
                                  ref Offsets[i], ref OffsetVelocities[i],
                                  out InertializedFloats[i], out InertializedVelocities[i]);
            }
        }

        /// <summary>
        /// Compute the offsets from the source float to the target float.
        /// Offsets are in/out since we may start a inertialization in the middle of another inertialization.
        /// </summary>
        public static void InertializeTransition(float source, float sourceVel,
                                                 float target, float targetVel,
                                                 ref float offset, ref float offsetVel)
        {
            offset = (source + offset) - target;
            offsetVel = (sourceVel + offsetVel) - targetVel;
        }

        /// <summary>
        /// Updates the inertialization decaying the offset and applying it to the target floats.
        /// </summary>
        public static void InertializeUpdate(float target, float targetVel,
                                             float halfLife, float deltaTime,
                                             ref float offset, ref float offsetVel,
                                             out float newValue, out float newVel)
        {
            DecaySpringDamperImplicit(ref offset, ref offsetVel, halfLife, deltaTime);
            newValue = target + offset;
            newVel = targetVel + offsetVel;
        }

        /// <summary>
        /// Special type of SpringDamperImplicit when the value is 0
        /// </summary>
        public static void DecaySpringDamperImplicit(ref float value, ref float velocity, float halfLife, float deltaTime)
        {
            float y = Utils.HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float j1 = velocity + value * y;
            float eyedt = Utils.FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            value = eyedt * (value + j1 * deltaTime);
            velocity = eyedt * (velocity - j1 * y * deltaTime);
        }
    }
}