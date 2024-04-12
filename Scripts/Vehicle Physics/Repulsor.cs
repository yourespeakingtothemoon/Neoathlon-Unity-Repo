using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class Repulsor : MonoBehaviour
{
    float velocity = 0f;

    [SerializeField] private Rigidbody parentRigidbody;
 
    [Header("Repulsor Settings")]
    [SerializeField] private float repulseStrength = 100f;
    [SerializeField] private float restDistance = 1f;
    [SerializeField] private float maxDistance = 5f;
    [SerializeField] private float damper = 0.5f;


    [Header("Magnetic Friction Settings")]
    [SerializeField] private float magneticMass = 1f;
    //[SerializeField] private float magneticFrictionFactor = 0.5f;
    [SerializeField] private AnimationCurve frictionLookupCurve;

    [SerializeField] private float steerLimit = 0.5f;

    [Header("Thruster Settings")]
    [SerializeField] private AnimationCurve thrusterCurve;
    [SerializeField] private float maxSpeed = 1000f;
    [SerializeField] private float thrusterStrength = 100f;

    [Header("Drivetrain Settings")]
    [SerializeField] private bool isThruster = false;
    [SerializeField] private bool isSteerable = false;

    [Header ("Debug")]
    [SerializeField] private bool ControlDebug = false;
    [SerializeField] private bool ThrusterDebug = false;
    private float steer = 0f;
    private float pedal = 0f;

    private bool isGrounded = false;
    // Start is called before the first frame update
    void Start()
    {
        velocity = 0f;
    }

    void Awake()
    {
        Controls drive = new Controls();
        drive.Drive.Enable();
        drive.Drive.Accelerate.performed += ctx => Thrust(ctx.ReadValue<float>());
        drive.Drive.Accelerate.canceled += ctx => Thrust(0f);
        drive.Drive.Steer.performed += ctx => Steer(ctx.ReadValue<Vector2>());
       // drive.Drive.Steer.canceled += ctx => Steer(Vector2.zero);
    }

    // Update is called once per frame
    void Update()
    {
  if (isSteerable && steer != 0f)
        {
               //slerp
               float targetSteer = steer * steerLimit;
               

               float targetSteerChange = Mathf.Clamp(targetSteer, -steerLimit, steerLimit);

               Quaternion target = Quaternion.Euler(transform.up * targetSteerChange);

               transform.localRotation = Quaternion.Lerp(transform.localRotation, target, Time.deltaTime);
               
               //clamp local rotation to max steer

        }

        parentRigidbody.AddForceAtPosition(transform.up * calculateRepulsion(), transform.position, ForceMode.Force);
        
        if(isGrounded){
        parentRigidbody.AddForceAtPosition(calculateMagneticFriction(), transform.position, ForceMode.Force);
       
      


         if(isThruster && pedal > 0f)
       {
        float currentSpeed = Vector3.Dot( transform.forward, parentRigidbody.velocity);
        float cappedSpeed = Mathf.Clamp(Mathf.Abs(currentSpeed), 0, maxSpeed);

        float target = thrusterCurve.Evaluate(cappedSpeed) * thrusterStrength * pedal;

        if (ThrusterDebug)
        {
            Debug.Log("Current Speed: " + currentSpeed);
            Debug.Log("Capped Speed: " + cappedSpeed);
            Debug.Log("Target: " + target);
        }

        parentRigidbody.AddForceAtPosition(transform.forward * target, transform.position, ForceMode.Force);
       }
        }
    
    }


    public float calculateRepulsion()
    {
        float repulsion = 0f;
        //raycast down from transform
        RaycastHit hit;
        if(Physics.Raycast(transform.position, -transform.up, out hit, maxDistance))
        {
            isGrounded = true;
           Vector3 worldVelocity = parentRigidbody.GetPointVelocity(transform.position);
           float offset = restDistance - hit.distance;
           float velocity = Vector3.Dot(worldVelocity, transform.up);
           repulsion = (repulseStrength * offset) - (damper * velocity);
        }
        else
        {
            isGrounded = false;
        }

        return repulsion;
    }

    public Vector3 calculateMagneticFriction()
    {
        //steering
        Vector3 worldVelocity = parentRigidbody.GetPointVelocity(transform.position);

        float steerVelocity = Vector3.Dot(transform.right, worldVelocity);
        float magneticFrictionFactor = frictionLookupCurve.Evaluate(steerVelocity);
        float targetSteerChange = -steerVelocity * magneticFrictionFactor;
        float targetAcceleration = targetSteerChange / Time.fixedDeltaTime;

      return transform.right * magneticMass * targetAcceleration;
    }

    public void Thrust(float input)
    {
        if(ControlDebug)
        {
            Debug.Log("Thrust: " + pedal);
        }
        if(isThruster)
        {
            pedal = input;
        }
      
    }

    public void Steer(Vector2 wheel)
    {
        if(ControlDebug)
        {
            Debug.Log("Steer: " + wheel.x);
        }

        if(isSteerable)
        {
        steer = wheel.x;
        }
    }

}
