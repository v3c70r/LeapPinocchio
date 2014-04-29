#include "defMesh.h"

vector<Vector3> DefMesh::getSkel() const
{
    //modify transformation here
    vector<Vector3> out = match;
    vector<Transform<> > t;
    t  = computeTransforms();

    for(int i = 0; i < (int)out.size(); ++i) {
        out[i] = t[max(0, i - 1)] * out[i];
    }
    
    return out;
}
vector<Transform<> > DefMesh::computeTransforms() const
{
    vector<Transform<> > out;
    vector<Transform<> > rotations;
    out.resize(origSkel.fPrev().size() - 1);
    rotations.resize(origSkel.fPrev().size() - 1);

    //Translate according to three fingers
    Quaternion<> trans1(dir.originDir[0], dir.curDir[0]);
    Quaternion<> trans2(dir.originDir[1], dir.curDir[1]);
    Quaternion<> trans3(dir.originDir[2], dir.curDir[2]);
    
    //Debug
    //Vector3 axi(1,0,0);
    //Quaternion<> trans1(axi, angle);
    //Quaternion<> trans2(axi, angle);
    //Quaternion<> trans3(axi, -angle);
    //EndDebug

    rotations[13] =  Transform<>(trans1.inverse()).linearComponent() ;        //Lefthand
    rotations[16] =  Transform<>(trans3.inverse()).linearComponent() ;        //Righthand
    rotations[2] =  Transform<>(trans2.inverse()).linearComponent() ;         //Head

    vector<Vector3> tm;
    tm.push_back(match[0]);
    Vector3 trans;

    for(int times = 0; times < 2; ++times) {

	  if(times == 1)
	    trans += (out[0] * match[0] - out[1] * match[2]);

	  out.clear();
	  vector<Vector3> tm;
	  tm.push_back(match[0]);
	  for(int i = 0; i < (int)rotations.size(); ++i) {
            int prevV = origSkel.fPrev()[i + 1];
            out.push_back(Transform<>(tm[prevV]) * rotations[i] * Transform<>(-match[prevV]));
            tm.push_back(out.back() * match[i + 1]);
	  }
	  
	  for(int i = 0; i < (int)out.size(); ++i)
            out[i] = Transform<>(trans ) * out[i];
	}
        
        return out;

    //for(int i = 1; i < (int)out.size(); ++i) {
    //    int prevV = origSkel.fPrev()[i];

    //    Transform<> cur = out[prevV];
    //    out[i] = cur * Transform<>(match[prevV]) * Transform<>(transforms[i - 1]) * Transform<>(-match[prevV]);

    //    //out[i]=(Transform<>(tm[prevV]) * rotations[i] * Transform<>(-match[prevV]));
    //    //tm.push_back(out[i] * match[i]);
    //}
}
void DefMesh::updateMesh() 
{
    angle += 0.001;
    curMesh = attachment.deform(origMesh, computeTransforms());
}
