using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wheel : MonoBehaviour
{
	// Wheel Specifications
	public Transform arrow;

	// Wheel radius in meters
	public float radius = 0.34f;
	// Wheel suspension travel in meters
	public float suspensionTravel = 0.2f;
	// Damper strength in kg/s
	public float damping = 5000;
	// Wheel angular inertia in kg * m^2
	public float weight = 8;
	private float inertia = 2.2f;
	// Coeefficient of grip - this is simly multiplied to the resulting forces, 
	// so it is not quite realitic, but an easy way to quickly change handling characteritics
	public float grip = 2.0f;
	public float latlstiffnes = 1;
	public float longstiffnes = 1;

	// Maximal braking torque (in Nm)
	public float brakeFrictionTorque = 4000;
	// Maximal handbrake torque (in Nm)
	public float handbrakeFrictionTorque = 0;
	// Base friction torque (in Nm)
	public float frictionTorque = 10;
	// Maximal steering angle (in degrees)
	public float maxSteeringAngle = 28f;
	// Graphical wheel representation (to be rotated accordingly)
	public GameObject model;
	// Fraction of the car's mass carried by this wheel
	public float massFraction = 0.25f;
	// Pacejka coefficients
	public float[] a = { 1.0f, -60f, 1688f, 4140f, 6.026f, 0f, -0.3589f, 1f, 0f, -6.111f / 1000f, -3.244f / 100f, 0f, 0f, 0f, 0f ,0f,0f,0f};
	public float[] b = { 1.0f, -60f, 1588f, 0f, 229f, 0f, 0f, 0f, -10f, 0f, 0f,0f,0f,0f,0f };

	// inputs
	// engine torque applied to this wheel
	public float driveTorque = 0;
	// engine braking and other drivetrain friction torques applied to this wheel
	public float driveFrictionTorque = 0;
	// brake input
	public float brake = 0;
	// handbrake input
	public float handbrake = 2000;
	// steering input
	public float steering = 0;
	// drivetrain inertia as currently connected to this wheel
	public float drivetrainInertia = 0;
	// suspension force externally applied (by anti-roll bars)
	public float suspensionForceInput = 0;
	//update angle
	public float newAngle ;
	// output
	public float angularVelocity;
	public float slipRatio;
	public float slipVelo;
	public float compression;

	// state
	public bool lefttyre = true;
	float fullCompressionSpringForce;
	Vector3 wheelVelo;
	Vector3 localVelo;
	Vector3 groundNormal;
	float rotation;
	float normalForce;
	Vector3 suspensionForce;
	Vector3 roadForce;
	Vector3 up, right, forward;
	Quaternion localRotation = Quaternion.identity;
	Quaternion inverseLocalRotation = Quaternion.identity;
	float slipAngle;
	int lastSkid = -1;

	// cached values
	Rigidbody body;
	float maxSlip;
	float maxAngle;
	float oldAngle;
	Skidmarks skid;

	float CalcLongitudinalForce(float Fz, float slip)
	{
		Fz *= 0.001f;//convert to kN
		slip *= 100f; //covert to %
		float uP = b[1] * Fz + b[2];
		float D = uP * Fz;
		float B = ((b[3] * Fz * Fz + b[4] * Fz) * Mathf.Exp(-b[5] * Fz)) / (b[0] * D);
		float S = slip + b[9] * Fz + b[10];
		float E = (b[6] * Fz * Fz + b[7] * Fz + b[8])*(1-b[13]*Mathf.Sign(S));
		float Fx = D * Mathf.Sin(b[0] * Mathf.Atan(S * B + E * (Mathf.Atan(S * B) - S * B)))+b[11]*Fz+b[12];
		return Fx;
	}

	float CalcLateralForce(float Fz, float slipAngle,float y=0)
	{
		Fz *= 0.001f;//convert to kN
		slipAngle *= (360f / (2 * Mathf.PI)); //convert angle to deg
		float uP = (a[1] * Fz + a[2])*(1-a[15]*y*y);
		float D = uP * Fz;
		float B = (a[3] * Mathf.Sin(2 * Mathf.Atan(Fz / a[4])))*(1-a[5]*Mathf.Abs(y)) / (a[0] * D);
		float S = slipAngle + a[8]*Fz+a[9] + a[10]*y;
		float E = (a[6] * Fz + a[7])*(1-(a[16]*y+a[17])*Mathf.Sign(S));
		float Sv = a[11]*Fz+a[12] +( a[13]*Fz+a[14])*y*Fz;
		float Fy = D * Mathf.Sin(a[0] * Mathf.Atan(S * B + E * (Mathf.Atan(S * B) - S * B))) + Sv;
		return Fy;
	}

	float CalcLongitudinalForceUnit(float Fz, float slip)
	{
		return CalcLongitudinalForce(Fz, slip * maxSlip);
	}

	float CalcLateralForceUnit(float Fz, float slipAngle)
	{
		return CalcLongitudinalForce(Fz, slipAngle * maxAngle);
	}

	Vector3 CombinedForce(float Fz, float slip, float slipAngle)
	{
		float unitSlip = slip / maxSlip;
		float unitAngle = slipAngle / maxAngle;
		float p = Mathf.Sqrt(unitSlip * unitSlip + unitAngle * unitAngle);
		if (p > Mathf.Epsilon)
		{
			if (slip < -0.8f)
            {
				return -localVelo.normalized * (Mathf.Abs(unitAngle / p * CalcLateralForceUnit(Fz, p) * latlstiffnes) + Mathf.Abs(unitSlip / p * CalcLongitudinalForceUnit(Fz, p) * longstiffnes));
			}
			else
			{
				Vector3 forward = new Vector3(0, -groundNormal.z, groundNormal.y);
				if (lefttyre == true)
					return Vector3.right * unitAngle / p * CalcLateralForceUnit(Fz, p)* latlstiffnes + forward * unitSlip / p * CalcLongitudinalForceUnit(Fz, p)*longstiffnes;
				else
					return Vector3.right * unitAngle / p * CalcLateralForceUnit(Fz, p) * latlstiffnes + forward * unitSlip / p * CalcLongitudinalForceUnit(Fz, p) * longstiffnes;
			}
		}
		else
			return Vector3.zero;
	}

	void InitSlipMaxima()
	{
		const float stepSize = 0.001f;
		const float testNormalForce = 4000f;
		float force = 0;
		for (float slip = stepSize; ; slip += stepSize)
		{
			float newForce = CalcLongitudinalForce(testNormalForce, slip);
			if (force < newForce)
				force = newForce;
			else
			{
				maxSlip = slip - stepSize;
				break;
			}
		}
		force = 0;
		for (float slipAngle = stepSize; ; slipAngle += stepSize)
		{
			float newForce = CalcLateralForce(testNormalForce, slipAngle);
			if (force < newForce)
				force = newForce;
			else
			{
				maxAngle = slipAngle - stepSize;
				break;
			}
		}
	}

	void Start()
	{
		Transform trs = transform;
		while (trs != null && trs.GetComponent<Rigidbody>() == null)
			trs = trs.parent;
		if (trs != null)
			body = trs.GetComponent<Rigidbody>();

		inertia = weight * 10 * radius * radius * 0.5f;
		InitSlipMaxima();
		skid = FindObjectOfType(typeof(Skidmarks)) as Skidmarks;
		fullCompressionSpringForce = body.mass * massFraction * 2.0f * -Physics.gravity.y;
	}

	Vector3 SuspensionForce()
	{
		float springForce = compression * fullCompressionSpringForce;
		normalForce = springForce;

		float damperForce = Vector3.Dot(localVelo, groundNormal) * damping;

		return (springForce - damperForce + suspensionForceInput) * up;
	}

	float SlipRatio()
	{
		const float fullSlipVelo = 4.0f;

		float wheelRoadVelo = Vector3.Dot(wheelVelo, forward);
		if (wheelRoadVelo == 0)
			return 0;

		float absRoadVelo = Mathf.Abs(wheelRoadVelo);
		float damping = Mathf.Clamp01(absRoadVelo / fullSlipVelo);

		float wheelTireVelo = angularVelocity * radius;
		return (wheelTireVelo - wheelRoadVelo) / absRoadVelo * damping;
	}

	float SlipAngle()
	{
		const float fullAngleVelo = 2.0f;

		Vector3 wheelMotionDirection = localVelo;
		wheelMotionDirection.y = 0;

		if (wheelMotionDirection.sqrMagnitude < Mathf.Epsilon)
			return 0;

		float sinSlipAngle = wheelMotionDirection.normalized.x;
		Mathf.Clamp(sinSlipAngle, -1, 1); // To avoid precision errors.

		float damping = Mathf.Clamp01(localVelo.magnitude / fullAngleVelo);

		return -Mathf.Asin(sinSlipAngle) * damping * damping;
	}

	Vector3 RoadForce()
	{
		int slipRes = (int)((100.0f - Mathf.Abs(angularVelocity)) / (10.0f));
		if (slipRes < 1)
			slipRes = 1;
		float invSlipRes = (1.0f / (float)slipRes);

		float totalInertia = inertia + drivetrainInertia;
		float driveAngularDelta = driveTorque * Time.deltaTime * invSlipRes/ totalInertia;
		float totalFrictionTorque = brakeFrictionTorque * brake + handbrakeFrictionTorque * handbrake + frictionTorque + driveFrictionTorque;
		float frictionAngularDelta = totalFrictionTorque * Time.deltaTime * invSlipRes / totalInertia;
		Vector3 totalForce = Vector3.zero;
		for (int i = 0; i < slipRes; i++)
		{
			float f = i * 1.0f / (float)slipRes;
			localRotation = Quaternion.Euler(0, oldAngle + (newAngle - oldAngle) * f, 0);
			inverseLocalRotation = Quaternion.Inverse(localRotation);
			forward = transform.TransformDirection(localRotation * Vector3.forward);
			right = transform.TransformDirection(localRotation * Vector3.right);

			slipRatio = SlipRatio();
			slipAngle = SlipAngle();
			Vector3 force = invSlipRes * grip * CombinedForce(normalForce, slipRatio, slipAngle);
			Vector3 worldForce = transform.TransformDirection(localRotation * force);
			angularVelocity -= (force.z * radius * Time.deltaTime) / totalInertia;
			angularVelocity += driveAngularDelta;
			if (Mathf.Abs(angularVelocity) > frictionAngularDelta)
				angularVelocity -= frictionAngularDelta * Mathf.Sign(angularVelocity);
			else
				angularVelocity = 0;

			wheelVelo += worldForce * (1 / body.mass) * Time.deltaTime * invSlipRes;
			totalForce += worldForce;
		}

		float longitunalSlipVelo = Mathf.Abs(angularVelocity * radius - Vector3.Dot(wheelVelo, forward));
		float lateralSlipVelo = Vector3.Dot(wheelVelo, right);
		slipVelo = Mathf.Sqrt(longitunalSlipVelo * longitunalSlipVelo + lateralSlipVelo * lateralSlipVelo);
		oldAngle = newAngle;
		return totalForce;
	}

	void FixedUpdate()
	{

		Vector3 pos = transform.position;
		up = transform.up;
		RaycastHit hit;
		bool onGround = Physics.Raycast(pos, -up, out hit, suspensionTravel + radius);

		if (onGround && hit.collider.isTrigger)
		{
			onGround = false; float dist = suspensionTravel + radius;
			RaycastHit[] hits = Physics.RaycastAll(pos, -up, suspensionTravel + radius);
			foreach (RaycastHit test in hits)
			{
				if (!test.collider.isTrigger && test.distance <= dist)
				{
					hit = test;
					onGround = true;
					dist = test.distance;
				}
			}
		}

		if (onGround)
		{
			groundNormal = transform.InverseTransformDirection(inverseLocalRotation * hit.normal);
			compression = 1.0f - ((hit.distance - radius) / suspensionTravel);
			wheelVelo = body.GetPointVelocity(pos);
			localVelo = transform.InverseTransformDirection(inverseLocalRotation * wheelVelo);
			suspensionForce = SuspensionForce();
			roadForce = RoadForce();
			Vector3 total = suspensionForce + roadForce;
			body.AddForceAtPosition(suspensionForce + roadForce, pos);
			arrow.transform.position = new Vector3(transform.position.x,transform.position.y + 2,transform.position.z);
			arrow.transform.eulerAngles = total;
		}
		else
		{
			compression = 0.0f;
			suspensionForce = Vector3.zero;
			roadForce = Vector3.zero;
			float totalInertia = inertia + drivetrainInertia;
			float driveAngularDelta = driveTorque * Time.deltaTime / totalInertia;
			float totalFrictionTorque = brakeFrictionTorque * brake + handbrakeFrictionTorque * handbrake + frictionTorque + driveFrictionTorque;
			float frictionAngularDelta = totalFrictionTorque * Time.deltaTime / totalInertia;
			angularVelocity += driveAngularDelta;
			if (Mathf.Abs(angularVelocity) > frictionAngularDelta)
				angularVelocity -= frictionAngularDelta * Mathf.Sign(angularVelocity);
			else
				angularVelocity = 0;
			slipRatio = 0;
			slipVelo = 0;
		}

		if (skid != null && Mathf.Abs(slipRatio) > 0.2)
			lastSkid = skid.AddSkidMark(hit.point, hit.normal, Mathf.Abs(slipRatio) - 0.2f, lastSkid);
		else
			lastSkid = -1;

		compression = Mathf.Clamp01(compression);
		rotation += angularVelocity * Time.deltaTime;
		if (model != null)
		{
			model.transform.localPosition = Vector3.up * (compression - 1.0f) * suspensionTravel;
			model.transform.localRotation = Quaternion.Euler(Mathf.Rad2Deg * rotation, maxSteeringAngle * steering, 0);
		}
	}
}/*
	 // Simple formula for peacjka model
    //longetudnal
	public float LOstiffness_4__12 = 10;
	public float LOshape_1__2 = 1.65f;
	public float LOpeak_0_1__1_9 = 1;
	public float LOcurvature_minus10__1 = 0.97f;

	//Lateral
	public float LAstiffness_4__12 = 10;
	public float LAshape_1__2 = 1.3f;
	public float LApeak_0_1__1_9 = 1;
	public float LAcurvature_minus10__1 = 0.97f;


	float CalcLateralForce(float Fz, float slipAngle)
	{
		slipAngle *= (360f / (2 * Mathf.PI)); //convert angle to deg

		float Fy = Fz*LApeak_0_1__1_9 * Mathf.Sin(LAshape_1__2* Mathf.Atan(LAstiffness_4__12 *slipAngle + LAcurvature_minus10__1 * (Mathf.Atan(LAstiffness_4__12 * slipAngle) - LAstiffness_4__12 * slipAngle))) ;
		return Fy;
	}

	float CalcLongitudinalForce(float Fz, float slip)
	{
		slip *= 100f; //covert to %
		float Fx = Fz * LOpeak_0_1__1_9 * Mathf.Sin(LOshape_1__2 * Mathf.Atan(LOstiffness_4__12 * slip + LOcurvature_minus10__1 * (Mathf.Atan(LOstiffness_4__12 * slip) - LOstiffness_4__12 * slip)));
		return Fx;
	}


	*/