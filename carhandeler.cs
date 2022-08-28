using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class carhandeler : MonoBehaviour
{

    public float breakForce;
    public float enginetorq;
    private float currentbrakforce;
    private const string Horizontal = "Horizontal";
    private const string Vertical = "Vertical";
    private float HorizontalInput;
    private float VerticalInput;
    private bool isbraking;
    public AnimationCurve enginpower;
    public AnimationCurve engintorque;
    [SerializeField] private WheelCollider br;
    [SerializeField] private WheelCollider bl;
    [SerializeField] private WheelCollider fl;
    [SerializeField] private WheelCollider fr;
    [SerializeField] private float steerradius;
    [SerializeField] private float steerTime;
    [SerializeField] private Transform tbr;
    [SerializeField] private Transform tbl;
    [SerializeField] private Transform tfl;
    [SerializeField] private Transform tfr;
    [SerializeField] private float downforce = 50;
    [SerializeField] private int gearno;
    [SerializeField] private float[] GearRatio;
    public float torq = 0;
    public int gear = 1;
    float lenth, widt;
    public float wheelrpm = 0;
    public float velocity = 0;
    public float velocityrigid = 0;
    float steeranglel = 0,steerangler = 0;
    bool gate = true;
    private void Start()
    {
        lenth = Mathf.Abs(tbr.localPosition.z - tfr.localPosition.z);
        widt = Mathf.Abs(tfl.localPosition.x - tfr.localPosition.x);
        
        Vector3 cm = GetComponent<Rigidbody>().centerOfMass;
        GetComponent<Rigidbody>().centerOfMass = new Vector3(cm.x, cm.y - 0.6f, cm.z);
    }
    private void FixedUpdate()
    {
        wheelrpm = Mathf.Abs((fl.rpm + br.rpm + bl.rpm + fr.rpm) / 4);

        velocityrigid = GetComponent<Rigidbody>().velocity.magnitude*3.6f;
        torq =( br.motorTorque+bl.motorTorque)/2;
     
        GetComponent<Rigidbody>().AddForce(-transform.up * downforce*velocityrigid);
        GetInput();
        HandeleMoter();
        HandeleStearing();
        UpdateWHeel();
    }
    void ShiftUp()
    {
        if (gear < GearRatio.Length - 1)
            gear++;
    }

    void ShiftDown()
    {
        if (gear > 0)
            gear--;
    }
 


   
    private void GetInput(){
        HorizontalInput = Input.GetAxis(Horizontal);
        VerticalInput = Input.GetAxis(Vertical);

        if( VerticalInput < 0)
        {
            gear = 0;
        }
        if (Input.GetKeyDown(KeyCode.Space)){
            isbraking = true;
        }
        
        if (Input.GetKeyUp(KeyCode.Space))
        {
            isbraking = false;
        }
        if (Input.GetKeyDown(KeyCode.Z) || Input.GetKeyDown(KeyCode.X))
        {
            gate = true;
        }
        if (Input.GetKeyUp(KeyCode.Z)&&gate)
        {
            ShiftUp();
            gate = false;

        }

        if (Input.GetKeyUp(KeyCode.X) && gate )
        {
            ShiftDown();
            gate = false;

        }
    }

    private void HandeleMoter(){

        br.motorTorque =( enginetorq) * VerticalInput;
        bl.motorTorque =( enginetorq)* VerticalInput;
        currentbrakforce = isbraking ? breakForce : 0f;
        Applybraking();
    }

    private void Applybraking()
    {
        br.brakeTorque = currentbrakforce;
        fr.brakeTorque = currentbrakforce;
        fl.brakeTorque = currentbrakforce;
        bl.brakeTorque = currentbrakforce;
    }
    private void HandeleStearing(){
       
        if  (HorizontalInput < 0)
        {
            steerangler = Mathf.Rad2Deg * (Mathf.Atan(lenth / (steerradius + (widt / 2))))*HorizontalInput;
            steeranglel = Mathf.Rad2Deg*(Mathf.Atan(lenth/(steerradius-(widt/2)))) * HorizontalInput;
        }
        else if (HorizontalInput>0)
        {
            steeranglel = Mathf.Rad2Deg * (Mathf.Atan(lenth / (steerradius + (widt / 2))))* HorizontalInput;
            steerangler = Mathf.Rad2Deg * (Mathf.Atan(lenth / (steerradius - (widt / 2)))) * HorizontalInput;
        }
        else
        {
            steerangler = Mathf.Abs(fr.steerAngle) *HorizontalInput;
            steeranglel = Mathf.Abs(fl.steerAngle) *HorizontalInput;
            if (Mathf.Abs(steerangler) < 1)
            {
                steerangler = 0;
                steeranglel = 0;
            }
        }
       
            fr.steerAngle = Mathf.Lerp(fr.steerAngle, steerangler, steerTime* Time.deltaTime);
            fl.steerAngle = Mathf.Lerp(fl.steerAngle, steeranglel, steerTime * Time.deltaTime);
    }

    private void  UpdateWHeel(){
        UpdateSingleWheel(fr, tfr);
        UpdateSingleWheel(fl, tfl);
        UpdateSingleWheel(bl, tbl);
        UpdateSingleWheel(br, tbr);
    }

    private void UpdateSingleWheel(WheelCollider c, Transform t)
    {
        Vector3 pos;
        Quaternion rot;
        c.GetWorldPose(out pos, out rot);
        t.rotation = rot;
        t.position = pos;
    }

}
