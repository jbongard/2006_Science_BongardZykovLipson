/* ---------------------------------------------------
   FILE:     meJoint.h
	AUTHOR:   Josh Bongard
	DATE:     March 22, 2001
	FUNCTION: This class contains all information for
				 a generic mathEngine joint.
 -------------------------------------------------- */

#include "stdafx.h"
#include "fstream.h"

#include <ode/ode.h>

#include "meobject.h"
#include "agent.h"

#ifndef _ME_JOINT_H
#define _ME_JOINT_H

class ME_JOINT {

public:
	int ID;
	int agentID;
	char name[50];
	int					sensorNeuronID;
	int					motorNeuronID;
	int					isBinaryMotor;
	int					jointType;
	dJointID			joint;
	double				lowerLimit;
	double				upperLimit;

private:
	double				position[3];
	class ME_OBJECT		*firstBody;
	class ME_OBJECT		*secondBody;
	class ME_OBJECT		*objectContainingJoint;
	ME_JOINT			*relatedJoint;
	char				relatedJointName[50];
	AGENT				*containingAgent;
	double				jointPosition[3];
	double				jointNormal[3];
	double				universalAxis1[3];
	double				universalAxis2[3];
	double				coneLimitAngle;
	double				restitution;
	double				stiffness;
	int					limitsSpecified;
	double				motorForce;
	double				motorSpeed;
	int					isWheel;
	double				originalRelativeDistance;
	double				myTorque;
	int					recordTorque;
	int					forceControlled;
	double				compressionForce;

public:
	ME_JOINT(void);
	ME_JOINT(int myID, int aID, ME_OBJECT *firstObj, ME_OBJECT *secondObj);
	ME_JOINT(int aID, ME_OBJECT **objs, int numObjs, double *disp,
		      ME_JOINT *templateJoint, NEURAL_NETWORK *network);
	ME_JOINT(int aID, ME_OBJECT **objs, int numObjs,
		ifstream *envFile, int inputID, class NEURAL_NETWORK *network);
	~ME_JOINT(void);
	void	Actuate(double motorPosition);
	void	Attach(dWorldID world);
	void    AttachToWorld(dWorldID world, ME_OBJECT *o);
	int		AttachesTo(int firstAgentID, int secondAgentID);
	void	Break(void);
	int		BreakageDesired(NEURAL_NETWORK *network);
	ME_OBJECT *ComplementaryObj(ME_OBJECT *obj);
	void	CreateODEStuff(dWorldID world);
	void	DestroyODEStuff(void);
	void	Display(void);
	void	DrawNormals(void);
	void	Draw(void);
	void	DrawMotorNeurons(double maxHeight, double heightOffset);
	void	DrawSensorNeurons(double maxHeight, double heightOffset);
	void    FindPressureSensor(void);
	ME_OBJECT *GeomTransformPossible(void);
	double	GetAngle(void);
	ME_JOINT *GetRelatedJoint(void);
	char	*GetRelatedJointName(void);
	int		IsBreaking(void);
	int		IsMyName(char *jointName);
	void	RecordPressure(void);
	int		RelatedToOtherJoint(void);
	void	Reset(dWorldID world);
	void	ResetPosition(void);
	int		Separated(void);
	void    SetOrientationSensor(void);
	void	SetRelatedJoint(ME_JOINT *otherJoint);
	void	SwitchOnPressureSensor(dWorldID world);
	void	UpdateSpring(void);

private:
	void			ActuateByForce(double forceToApply);
	void			ActuateByPosition(double motorPosition);
	void			ActuateByVelocity(double motorVel);
	void			ActuateHinge(double motorPosition);
	void			ActuateSlider(double motoroPosition);
	ME_OBJECT*		FindObject(ME_OBJECT **objs, int numObjs, char *name);
	int				NumberEntered(char *name);
	void			ReadIn(ifstream *envFile, class ME_OBJECT **objs, int numObjs,
								NEURAL_NETWORK *network);
	void			RecordTorque(void);
};

#endif