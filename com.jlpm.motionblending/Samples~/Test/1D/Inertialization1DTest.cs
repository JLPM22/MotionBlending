using MotionBlending;
using UnityEngine;

[DefaultExecutionOrder(100)]
public class Inertialization1DTest : MonoBehaviour
{
    public bool TriggerTransition = false;
    public SinusoidalMovement Target;
    public float HalfLife = 0.5f;

    private InertializationFloat Inertialization;

    private float SourceVelocity;
    private float PrevSourcePos;
    private float TargetVelocity;
    private float PrevTargetPos;

    void Start()
    {
        Inertialization = new InertializationFloat(1);
    }

    private void Update()
    {
        if (TriggerTransition)
        {
            TriggerTransition = false;
            Transition();
        }

        TargetVelocity = (Target.transform.position.y - PrevTargetPos) / Time.deltaTime;
        Inertialization.Update(new float[] { Target.transform.position.y },
                               new float[] { TargetVelocity },
                               HalfLife, Time.deltaTime);

        transform.position = new Vector3(Target.transform.position.x, Inertialization.InertializedFloats[0], Target.transform.position.z);

        SourceVelocity = (transform.position.y - PrevSourcePos) / Time.deltaTime;
        PrevSourcePos = transform.position.y;
        PrevTargetPos = Target.transform.position.y;
    }

    private void Transition()
    {
        float currentTarget = Target.transform.position.y; // this is very important as inertialization assumes that sources changes to target after
                                                           // each transition, and the offset accounts for the difference, if we don't do that, the offset
                                                           // is not correctly added when applying multiple transitons while offset != 0.
        Target.AmplitudeY = Random.Range(1.0f, 10.0f);
        Target.Update();
        Inertialization.Transition(new float[] { currentTarget },
                                   new float[] { SourceVelocity },
                                   new float[] { Target.transform.position.y },
                                   new float[] { TargetVelocity });
    }
}
