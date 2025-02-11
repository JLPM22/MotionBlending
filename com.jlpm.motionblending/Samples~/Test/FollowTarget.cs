using UnityEngine;

public class FollowTarget : MonoBehaviour
{
    public Transform Target;

    private Vector3 LocalOffset;

    private void Start()
    {
        LocalOffset = transform.position - Target.position;
    }

    private void LateUpdate()
    {
        transform.position = new Vector3(Target.position.x + LocalOffset.x, LocalOffset.y, Target.position.z + LocalOffset.z);
    }
}
