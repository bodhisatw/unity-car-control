using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Drivetrain))]
public class carhandeler2 : MonoBehaviour
{
	// Add all wheels of the car here, so brake and steering forces can be applied to them.
	[SerializeField] private Wheel br;
	[SerializeField] private Wheel bl;
	[SerializeField] private Wheel fl;
	[SerializeField] private Wheel fr;
	[SerializeField] private float steerradius;
	private Wheel[] wheels ;
	// A transform object which marks the car's center of gravity.
	// Cars with a higher CoG tend to tilt more in corners.
	// The further the CoG is towards the rear of the car, the more the car tends to oversteer. 
	// If this is not set, the center of mass is calculated from the colliders.
	public Transform centerOfMass;

	// A factor applied to the car's inertia tensor. 
	// Unity calculates the inertia tensor based on the car's collider shape.
	// This factor lets you scale the tensor, in order to make the car more or less dynamic.
	// A higher inertia makes the car change direction slower, which can make it easier to respond to.
	public float inertiaFactor = 0.8f;

	// current input state
	float brake;
	float throttle;
	float throttleInput;
	public float  steering =0;
	float lastShiftTime = -1;
	float handbrake;

	// cached Drivetrain reference
	Drivetrain drivetrain;

	// How long the car takes to shift gears
	public float shiftSpeed = 0.8f;


	// These values determine how fast throttle value is changed when the accelerate keys are pressed or released.
	// Getting these right is important to make the car controllable, as keyboard input does not allow analogue input.
	// There are different values for when the wheels have full traction and when there are spinning, to implement 
	// traction control schemes.

	// How long it takes to fully engage the throttle
	public float throttleTime = 0.5f;
	// How long it takes to fully engage the throttle 
	// when the wheels are spinning (and traction control is disabled)
	public float throttleTimeTraction = 10.0f;
	// How long it takes to fully release the throttle
	public float throttleReleaseTime = 0.5f;
	// How long it takes to fully release the throttle 
	// when the wheels are spinning.
	public float throttleReleaseTimeTraction = 0.1f;

	// Turn traction control on or off
	public bool tractionControl = true;

	public Transform carbody;

	// These values determine how fast steering value is changed when the steering keys are pressed or released.
	// Getting these right is important to make the car controllable, as keyboard input does not allow analogue input.

	// How long it takes to fully turn the steering wheel from center to full lock
	public float steerTime = 1.2f;
	// This is added to steerTime per m/s of velocity, so steering is slower when the car is moving faster.
	public float veloSteerTime = 0.1f;

	// How long it takes to fully turn the steering wheel from full lock to center
	public float steerReleaseTime = 0.8f;
	// This is added to steerReleaseTime per m/s of velocity, so steering is slower when the car is moving faster.
	public float veloSteerReleaseTime = 0.05f;
	// When detecting a situation where the player tries to counter steer to correct an oversteer situation,
	// steering speed will be multiplied by the difference between optimal and current steering times this 
	// factor, to make the correction easier.
	public float steerCorrectionFactor = 4.0f;

	// Used by SoundController to get average slip velo of all wheels for skid sounds.
	public float slipVelo
	{
		get
		{
			float val = 0.0f;
			foreach (Wheel w in wheels) {
				val += w.slipVelo / wheels.Length;
				if (w == br && w == fr)
					w.lefttyre = false;
			}
			return val;
		}
	}

	// Initialize
	void Start()
	{
		
		wheels = new Wheel[] { br, bl, fr, fl };
		br.lefttyre = false;
		fl.lefttyre = false;
		//if (centerOfMass != null)
		//GetComponent<Rigidbody>().centerOfMass = centerOfMass.localPosition;
		GetComponent<Rigidbody>().centerOfMass = new Vector3(GetComponent<Rigidbody>().centerOfMass.x, GetComponent<Rigidbody>().centerOfMass.y -0.35f, GetComponent<Rigidbody>().centerOfMass.z);
		GetComponent<Rigidbody>().inertiaTensor *= inertiaFactor;
		drivetrain = GetComponent(typeof(Drivetrain)) as Drivetrain;

	}

	void Update()
    {
		float lenth = Mathf.Abs(wheels[0].transform.localPosition.z - wheels[2].transform.localPosition.z);
		float widt = Mathf.Abs(wheels[3].transform.localPosition.x - wheels[2].transform.localPosition.x);
		carbody.transform.position = transform.position;
		Vector3 carDir = transform.forward;
		float fVelo = GetComponent<Rigidbody>().velocity.magnitude;
		Vector3 veloDir = GetComponent<Rigidbody>().velocity * (1 / fVelo);
		float angle = -Mathf.Asin(Mathf.Clamp(Vector3.Cross(veloDir,carDir).y,-1,1));
		float optimalSteering = angle / (Mathf.Atan(lenth / steerradius) * Mathf.Deg2Rad);
		if (fVelo < 1)
			optimalSteering = 0;

		float steerInput = 0;
		if (Input.GetKey(KeyCode.LeftArrow))
			steerInput = -1;
		if (Input.GetKey(KeyCode.RightArrow))
			steerInput = 1;


				// Throttle/Brake

		bool accelKey = Input.GetKey(KeyCode.UpArrow);
		bool brakeKey = Input.GetKey(KeyCode.DownArrow);

		if (drivetrain.automatic && drivetrain.gear == 0)
		{
			accelKey = Input.GetKey(KeyCode.DownArrow);
			brakeKey = Input.GetKey(KeyCode.UpArrow);
		}

		if (Input.GetKey(KeyCode.LeftShift))
		{
			throttle += Time.deltaTime / throttleTime;
			throttleInput += Time.deltaTime / throttleTime;
		}
		else if (accelKey)
		{
			if (drivetrain.slipRatio < 0.10f)
				throttle += Time.deltaTime / throttleTime;
			else if (!tractionControl)
				throttle += Time.deltaTime / throttleTimeTraction;
			else
				throttle -= Time.deltaTime / throttleReleaseTime;

			if (throttleInput < 0)
				throttleInput = 0;
			throttleInput += Time.deltaTime / throttleTime;
			brake = 0;
		}
		else
		{
			if (drivetrain.slipRatio < 0.2f)
				throttle -= Time.deltaTime / throttleReleaseTime;
			else
				throttle -= Time.deltaTime / throttleReleaseTimeTraction;
		}
		throttle = Mathf.Clamp01(throttle);

		if (brakeKey)
		{
			if (drivetrain.slipRatio < 0.2f)
				brake += Time.deltaTime / throttleTime;
			else
				brake += Time.deltaTime / throttleTimeTraction;
			throttle = 0;
			throttleInput -= Time.deltaTime / throttleTime;
		}
		else
		{
			if (drivetrain.slipRatio < 0.2f)
				brake -= Time.deltaTime / throttleReleaseTime;
			else
				brake -= Time.deltaTime / throttleReleaseTimeTraction;
		}
		brake = Mathf.Clamp01(brake);
		throttleInput = Mathf.Clamp(throttleInput, -1, 1);

		// Handbrake
		handbrake = Mathf.Clamp01(handbrake + (Input.GetKey(KeyCode.Space) ? Time.deltaTime : -Time.deltaTime));

		// Gear shifting
		float shiftThrottleFactor = Mathf.Clamp01((Time.time - lastShiftTime) / shiftSpeed);
		drivetrain.throttle = throttle * shiftThrottleFactor;
		drivetrain.throttleInput = throttleInput;

		if (Input.GetKeyDown(KeyCode.A))
		{
			lastShiftTime = Time.time;
			drivetrain.ShiftUp();
		}
		if (Input.GetKeyDown(KeyCode.Z))
		{
			lastShiftTime = Time.time;
			drivetrain.ShiftDown();
		}

		// Apply inputs
		{
			
			foreach (Wheel w in wheels)
			{
				w.brake = brake;
			

			}
            // hand brake
            {
				br.handbrake = handbrake;
				bl.handbrake = handbrake;

			}

			float steerSpeed = 0;
			if (steerInput < steering)
				{
					steerSpeed = (steering > 0) ? (1 / (steerReleaseTime + veloSteerReleaseTime * fVelo)) : (1 / (steerTime + veloSteerTime * fVelo));
		
					steering -= steerSpeed * Time.deltaTime;

					if (steerInput > steering)
							steering = steerInput;
				}
				else if (steerInput > steering)
				{
					steerSpeed = (steering < 0) ? (1 / (steerReleaseTime + veloSteerReleaseTime * fVelo)) : (1 / (steerTime + veloSteerTime * fVelo));
					steering += steerSpeed * Time.deltaTime;
					if (steerInput < steering)
							steering = steerInput;
				}
			float steeranglel=0, steerangler=0;
			if (steerInput < 0)
			{
				
				br.driveTorque = drivetrain.Drivetorque *((steerradius   + widt/2)/ (steerradius ));
				bl.driveTorque = drivetrain.Drivetorque * ((steerradius - widt / 2) / (steerradius));
				steerangler = Mathf.Rad2Deg * (Mathf.Atan(lenth / (steerradius + (widt / 2)))) * steering;
				steeranglel = Mathf.Rad2Deg * (Mathf.Atan(lenth / (steerradius - (widt / 2)))) * steering;
			}
			else if (steerInput > 0)
			{
				bl.driveTorque = drivetrain.Drivetorque * ((steerradius+ widt/2) / (steerradius ));
				br.driveTorque = drivetrain.Drivetorque * ((steerradius - widt / 2) / (steerradius));
				steeranglel = Mathf.Rad2Deg * (Mathf.Atan(lenth / (steerradius + (widt / 2)))) * steering;
				steerangler = Mathf.Rad2Deg * (Mathf.Atan(lenth / (steerradius - (widt / 2)))) * steering;
			}
			else
			{
			
				steerangler =  Mathf.Lerp(steeranglel,0,Time.deltaTime*steerReleaseTime);
				steeranglel = Mathf.Lerp(steerangler, 0, Time.deltaTime *steerReleaseTime);
			}

			{
				fr.newAngle = steerangler;
				fl.newAngle = steeranglel;

				float ratio_on_grip = ((steerradius - (widt / 2)) * Mathf.Abs(steering)) / ((steerradius - (widt / 2)) * Mathf.Abs(steering) + widt);

				fr.grip = 2f - ratio_on_grip * steering + (fVelo / 30);
				br.grip = 2f - ratio_on_grip * steering + (fVelo / 30);
				fl.grip = 2f + ratio_on_grip * steering + (fVelo / 30);
				bl.grip = 2f + ratio_on_grip * steering + (fVelo / 30);

				bl.suspensionForceInput = - ratio_on_grip * steering * fVelo*10;
				fl.suspensionForceInput = -ratio_on_grip * steering * fVelo*10;
				br.suspensionForceInput = ratio_on_grip * steering * fVelo*10;
				fr.suspensionForceInput = ratio_on_grip * steering * fVelo*10;
			}
		}
	}

	// Debug GUI. Disable when not needed.
	void OnGUI()
	{
		GUI.Label(new Rect(0, 60, 100, 200), "km/h: " + GetComponent<Rigidbody>().velocity.magnitude * 3.6f);
		tractionControl = GUI.Toggle(new Rect(0, 80, 300, 20), tractionControl, "Traction Control (bypassed by shift key)");
	}
}

