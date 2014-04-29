
#ifndef DEFMESH_H
#define DEFMESH_H

#include "../Pinocchio/attachment.h"
#include <Leap.h>

class LeapDir
{
    public:
        vector<Vector3> originDir;
        vector<Vector3> curDir;
        LeapDir()
        {
            //Init finger dirs
            originDir.resize(3);
            curDir.resize(3);
            originDir[0][0] =-0.00194761; 
            originDir[0][1] = -0.0822463; 
            originDir[0][2] = -0.99661;
            originDir[1][0] = -0.20835;
            originDir[1][1] = -0.070797;
            originDir[1][2] = -0.975489;
            originDir[2][0] = 0.312237;
            originDir[2][1] = -0.16735;
            originDir[2][2] = -0.935148;

            curDir[0][0] =-0.00194761; 
            curDir[0][1] = -0.0822463; 
            curDir[0][2] = -0.99661;
            curDir[1][0] = -0.20835;
            curDir[1][1] = -0.070797;
            curDir[1][2] = -0.975489;
            curDir[2][0] = 0.312237;
            curDir[2][1] = -0.16735;
            curDir[2][2] = -0.935148;

            cout<<originDir[0][0]<<" | "
            <<originDir[1][0]<<" | "
            <<originDir[2][0]<<" | \n";

            sorteVec(originDir);

            cout<<originDir[0][0]<<" | "
            <<originDir[1][0]<<" | "
            <<originDir[2][0]<<" | \n";
        }
        void setCurDir(Leap::FingerList fingers)
        {
            if (fingers.count() == 3){ //Have three fingers;
                for (int i=0; i<3; i++){
                    curDir[i][0] = fingers[i].direction().normalized().x;
                    curDir[i][1] = fingers[i].direction().normalized().y;
                    curDir[i][2] = fingers[i].direction().normalized().z;
                }
                sorteVec(curDir);

            }

           // cout<<curDir[0][0]<<" | "
           // <<curDir[1][0]<<" | "
           // <<curDir[2][0]<<" | \n";
                
        }

        //Sort vectors by x coordinate
        void sorteVec(vector<Vector3> &in)
        {
            if (in[0][0] < in[1][0])
                if (in[0][0]<in[2][0])
                    if (in[1][0]<in[2][0]);
                    else{
                        swap(in[1],in[2]);}
                else {swap(in[0],in[2]); swap(in[1],in[2]);}
            else
                if (in[1][0] < in[2][0])
                    if(in[0][0]<in[2][0])
                        swap(in[0], in[1]);
                    else{swap(in[0], in[2]); swap(in[0], in[1]);}
                else swap(in[0], in[2]);
        }

        void swap(Vector3 &a, Vector3 & b)
        {
            Vector3 temp  = a;
            a = b;
            b = temp;
        }
};

class DefMesh
{
public:
    double angle; //debug
    LeapDir dir;
    DefMesh(const Mesh inMesh, const Skeleton &inOrigSkel, const vector<Vector3> &inMatch, const Attachment &inAttachment)
        :origSkel(inOrigSkel),  attachment(inAttachment),  origMesh(inMesh), match(inMatch)
    {
        transforms.resize(origSkel.fPrev().size() - 1);
        angle = 0;
    }


    vector<Vector3> getSkel() const ;
    const Skeleton &getOrigSkel() const { return origSkel; }

    const Attachment &getAttachment() const { return attachment; }

    const Mesh &getMesh() {  
        return curMesh;
    }
    void updateMesh() ;

private:
    vector<Transform<> > computeTransforms() const;
    Skeleton origSkel;
    Attachment attachment;
    Mesh origMesh;
    mutable Mesh curMesh;
    vector<Quaternion<> > transforms;
    vector<Vector3> match;
};

#endif

