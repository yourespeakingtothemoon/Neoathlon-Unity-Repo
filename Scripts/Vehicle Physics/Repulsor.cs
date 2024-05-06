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
    [SerializeField] private float friction = 0.5f;

    [Header("Rolling Friction Settings")]
    [SerializeField] private AnimationCurve rollingFriction;
    [SerializeField] private float rollingFrictionFactor = 2f;

    [Header("Steering Settings")]

    [SerializeField] private AnimationCurve steeringCurve;
    [SerializeField] private float steerLimit = 45;
    [SerializeField] private float steerSpeed = 100f;

    [Header("Thruster Settings")]
    [SerializeField] private AnimationCurve thrusterCurve;
    [SerializeField] private float maxSpeed = 1000f;
    [SerializeField] private float thrusterStrength = 100f;

    [Header("BrakeSettings")]
    [SerializeField] private float brakeStrength = 100f;
    [SerializeField] private AnimationCurve brakeCurve;

    [Header("Drivetrain Settings")]
    [SerializeField] private bool isThruster = false;
    [SerializeField] private bool isSteerable = false;
    [SerializeField] private bool isBrake = false;

    [Header ("Debug")]
    [SerializeField] private bool ControlDebug = false;
    [SerializeField] private bool ThrusterDebug = false;
    private float steer = 0f;
    private float pedal = 0f;
    private float brake = 0f;
    private float steeringAngle = 0f;
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
        drive.Drive.Steer.canceled += ctx => Steer(Vector2.zero);
        drive.Drive.Brake.performed += ctx => Brake(ctx.ReadValue<float>());
        drive.Drive.Brake.canceled += ctx => Brake(0f);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        
              Vector3 worldVelocity = parentRigidbody.GetPointVelocity(transform.position);
        float normalizedVelocity = Vector3.Dot(worldVelocity, transform.forward)/maxSpeed ;

        if(isSteerable)
        {
        //Debug.Log("Normalized Velocity: " + normalizedVelocity);
            float maxSteerFactor = steeringCurve.Evaluate(normalizedVelocity);
        
            float targetSteer = steer * steerLimit * maxSteerFactor;
            float updateSpeed = steerSpeed * Time.fixedDeltaTime;
            if(steer == 0f)
            {
                updateSpeed = (steerSpeed/1.5f) * Time.fixedDeltaTime;
            }
            steeringAngle = Mathf.MoveTowardsAngle(steeringAngle, targetSteer, updateSpeed);
        
            //math.clamp(steeringAngle, -steerLimit, steerLimit);
        }
        transform.localRotation = Quaternion.Euler(0, steeringAngle, 0);
 

        parentRigidbody.AddForceAtPosition(transform.up * calculateRepulsion(), transform.position, ForceMode.Force);
        
        if(isGrounded){
       // Debug.Log(calculateMagneticFriction());
        parentRigidbody.AddForceAtPosition(calculateMagneticFriction(), transform.position);
        //constantly add rolling friction

        
        parentRigidbody.AddForceAtPosition(-transform.forward * rollingFriction.Evaluate(normalizedVelocity) * rollingFrictionFactor, transform.position);
        }
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

         if(isBrake && brake > 0f)
         {
                
                float target = brakeCurve.Evaluate(normalizedVelocity) * brakeStrength;
                parentRigidbody.AddForceAtPosition(-transform.forward * target, transform.position, ForceMode.Force);
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
       Vector3 worldVelocity = parentRigidbody.GetPointVelocity(this.transform.position);

        float steerVelocity = Vector3.Dot(this.transform.right, worldVelocity);
        //Debug.Log("Steer Velocity: " + steerVelocity/40);
        float magneticFrictionFactor = frictionLookupCurve.Evaluate(math.abs(steerVelocity)/40);
        Debug.Log("Magnetic Friction Factor: " + magneticFrictionFactor);
        float targetSteerChange = -steerVelocity * friction;
        float targetAcceleration = targetSteerChange / Time.fixedDeltaTime;
       // Debug.Log("Target Acceleration: " + targetAcceleration);
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

    public void Brake(float input)
    {
        if(ControlDebug)
        {
            Debug.Log("Brake");
        }
        if(isBrake)
        {
            brake = input;
        }
    }

}
