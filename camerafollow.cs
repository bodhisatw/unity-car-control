using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class camerafollow : MonoBehaviour
{

    [SerializeField] private GameObject car;
    [SerializeField] private Vector3 Offset;
    public Transform target;
    public float translatespeed;
    public float rotationspeed;
    public float newangle = 0;


    private void FixedUpdate()
    {
        HandleTranslation();
        HandleRotation();
    }

    private void HandleRotation()
    {
        var direction = target.position - transform.position;
        var rotation = Quaternion.LookRotation(direction,Vector3.up);
        transform.rotation = Quaternion.Lerp(transform.rotation, rotation , rotationspeed * Time.deltaTime);
        float Angle = Vector3.Angle(direction, transform.forward);
        ;
        Angle = Mathf.Clamp(Angle, -15f, 15f)*Mathf.Sign(car.GetComponent<carhandeler2>().steering);
        Debug.Log(Angle);
        newangle= Mathf.Lerp(newangle, Angle, rotationspeed * Time.deltaTime);
        //transform.eulerAngles = new Vector3(transform.rotation.eulerAngles.x, transform.rotation.eulerAngles.y, newangle);
    }

    private void HandleTranslation()    
    {
        var targetPosition = target.TransformPoint(Offset);
        transform.position = Vector3.Lerp(transform.position, targetPosition, translatespeed*Time.deltaTime);
    }

}
