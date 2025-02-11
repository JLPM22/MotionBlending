using UnityEngine;

public class SinusoidalMovement : MonoBehaviour
{
    public float VelocityX = 1.0f;
    public float AmplitudeY = 1.0f;
    public float FrequencyY = 1.0f;

    public void Update()
    {
        float x = Time.deltaTime * VelocityX;
        float y = AmplitudeY * Mathf.Sin(FrequencyY * (transform.position.x + x));
        transform.position = new Vector3(transform.position.x + x, y, 0.0f);
    }
}
