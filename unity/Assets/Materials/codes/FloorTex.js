#pragma strict
// this is a texture script
// right now only set the texture to repeat instead of scale in update function
function Start () {

}

function Update () {
	var XScale = 10;
	var YScale = 10;
	transform.GetComponent.<Renderer>().material.mainTextureScale = new Vector2(XScale , YScale );
}