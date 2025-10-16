/*
	Copyright 2011-2025 Daniel S. Buckstein

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

/*
	animal3D SDK: Minimal 3D Animation Framework
	By Daniel S. Buckstein
	
	a3_Kinematics.c
	Implementation of kinematics solvers.
*/

#include "../a3_Kinematics.h"


//-----------------------------------------------------------------------------

// single FK helpers
static inline void a3kinematicsSolveForwardSingle(const a3_HierarchyState* hierarchyState, const a3ui32 index, const a3ui32 parentIndex)
{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------

	// T[this_object] = T[parent_object] * T[this_local]
	a3real4x4Product(
		hierarchyState->objectSpace->hpose_base[index].transformMat.m,		// Result: this node object-space.
		hierarchyState->objectSpace->hpose_base[parentIndex].transformMat.m,// Left-hand: parent node object-space.
		hierarchyState->localSpace->hpose_base[index].transformMat.m		// Right-hand: this node local space.
	);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
}
static inline void a3kinematicsSolveForwardRoot(const a3_HierarchyState* hierarchyState, const a3ui32 index)
{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------

	// T[root_object] = T[root_local]
	hierarchyState->objectSpace->hpose_base[index] = hierarchyState->localSpace->hpose_base[index];

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
}

// partial FK solver
a3i32 a3kinematicsSolveForwardPartial(const a3_HierarchyState* hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount)
{
	if (hierarchyState && hierarchyState->hierarchy &&
		firstIndex < hierarchyState->hierarchy->numNodes && nodeCount)
	{
		// implement forward kinematics algorithm: 
		//	- for all nodes starting at first index
		//		- if node is not root (has parent node)
		//			- object matrix = parent object matrix * local matrix
		//		- else
		//			- copy local matrix to object matrix
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------

		const a3_HierarchyNode* itr = hierarchyState->hierarchy->nodes + firstIndex;
		const a3_HierarchyNode* const end = itr + nodeCount;
		for (; itr < end; ++itr)
		{
			if (itr->parentIndex >= 0)
				a3kinematicsSolveForwardSingle(hierarchyState, itr->index, itr->parentIndex);
			else
				a3kinematicsSolveForwardRoot(hierarchyState, itr->index);
		}
		return (a3i32)(end - itr);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
	}
	return -1;
}


//-----------------------------------------------------------------------------

// single IK helpers
static inline void a3kinematicsSolveInverseSingle(const a3_HierarchyState* hierarchyState, const a3ui32 index, const a3ui32 parentIndex)
{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------

	a3real4x4Product(
		hierarchyState->localSpace->hpose_base[index].transformMat.m,		// Result: this node local-space.
		hierarchyState->objectSpaceInv->hpose_base[parentIndex].transformMat.m,// Left-hand: parent node object-space inverse.
		hierarchyState->objectSpace->hpose_base[index].transformMat.m		// Right-hand: this node object space.
	);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
}
static inline void a3kinematicsSolveInverseRoot(const a3_HierarchyState* hierarchyState, const a3ui32 index)
{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------

	hierarchyState->localSpace->hpose_base[index] = hierarchyState->objectSpace->hpose_base[index];

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
}

// partial IK solver
a3i32 a3kinematicsSolveInversePartial(const a3_HierarchyState* hierarchyState, const a3ui32 firstIndex, const a3ui32 nodeCount)
{
	if (hierarchyState && hierarchyState->hierarchy &&
		firstIndex < hierarchyState->hierarchy->numNodes && nodeCount)
	{
		// implement inverse kinematics algorithm: 
		//	- for all nodes starting at first index
		//		- if node is not root (has parent node)
		//			- local matrix = inverse parent object matrix * object matrix
		//		- else
		//			- copy object matrix to local matrix
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------
		const a3_HierarchyNode* itr = hierarchyState->hierarchy->nodes + firstIndex;
		const a3_HierarchyNode* const end = itr + nodeCount;
		for (; itr < end; ++itr)
		{
			if (itr->parentIndex >= 0)
				a3kinematicsSolveInverseSingle(hierarchyState, itr->index, itr->parentIndex);
			else
				a3kinematicsSolveInverseRoot(hierarchyState, itr->index);
		}
		return (a3i32)(end - itr);


//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
	}
	return -1;
}


//-----------------------------------------------------------------------------

void a3kinematicsUpdateHierarchyStateFK(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup)
{
	if (activeHS->hierarchy == baseHS->hierarchy &&
		activeHS->hierarchy == poseGroup->hierarchy)
	{
		// FK pipeline
		//	-> concatenate base pose
		//	-> convert poses to local-space matrices
		//	-> perform recursive FK
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------

		a3hierarchyPoseConcat(activeHS->localSpace,	// local: goal to calculate
			activeHS->animPose,						// holds current sample pose
			baseHS->localSpace,						// holds base pose (animPose is all identity poses)
			activeHS->hierarchy->numNodes);
		a3hierarchyPoseConvert(activeHS->localSpace,
			activeHS->hierarchy->numNodes,
			poseGroup->channel,
			poseGroup->order);
		a3kinematicsSolveForward(activeHS);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
	}
}

void a3kinematicsUpdateHierarchyStateIK(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup)
{
	if (activeHS->hierarchy == baseHS->hierarchy &&
		activeHS->hierarchy == poseGroup->hierarchy)
	{
		// IK pipeline
		//	-> perform recursive IK
		//	-> restore local-space matrices to poses
		//	-> deconcatenate base pose
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------

		a3kinematicsSolveInverse(activeHS);
		a3hierarchyPoseRestore(activeHS->localSpace,
			activeHS->hierarchy->numNodes,
			poseGroup->channel,
			poseGroup->order);
		a3hierarchyPoseDeconcat(
			activeHS->animPose, //result is animation pose
			activeHS->localSpace, //LH local
			baseHS->localSpace, //subtract base local
			activeHS->hierarchy->numNodes);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
	}
}

void a3kinematicsUpdateHierarchyStateSkin(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS)
{
	if (activeHS->hierarchy == baseHS->hierarchy)
	{
		// FK pipeline extended for skinning and other applications
		//	-> update local-space inverse matrices
		//	-> update object-space inverse matrices
		//	-> update transform from base to current
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------

		a3hierarchyStateUpdateLocalInverse(activeHS);
		a3hierarchyStateUpdateObjectInverse(activeHS);
		a3hierarchyStateUpdateObjectBindToCurrent(activeHS, baseHS);

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
	}
}


//-----------------------------------------------------------------------------

// helper to resolve single-joint IK after solver
static void a3kinematicsResolvePostIK(a3_HierarchyState* activeHS,
	a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup,
	a3ui32 const nodeIndex, a3real4x4 const j2obj)
{
	// post-IK resolution for single affected joint
	//	-> reassign resolved transform to object-space
	//	-> compute object-space inverse matrix
	//	-> compute local-space matrix
	//	-> restore local-space matrix to pose
	//	-> deconcatenate base pose
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------
	
	//1
	a3real4x4SetReal4x4(activeHS->objectSpace->hpose_base[nodeIndex].transformMat.m, j2obj);	

	//2 Doing the inverse of the active HS breaks it for some reason
	//a3real4x4GetInverse(activeHS->objectSpaceInv->hpose_base[nodeIndex].transformMat.m, j2obj);
	a3real4x4TransformInverse(activeHS->objectSpaceInv->hpose_base[nodeIndex].transformMat.m, j2obj);

	//3 --> go back to local space
	a3ui32 parentIndex = activeHS->hierarchy->nodes[nodeIndex].parentIndex;
	a3real4x4Product(
		activeHS->localSpace->hpose_base[nodeIndex].transformMat.m,
		activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.m, 
		activeHS->objectSpace->hpose_base[nodeIndex].transformMat.m);

	//4
	//do both so that we can do the deconcat
	a3spatialPoseRestore(activeHS->localSpace->hpose_base + nodeIndex, poseGroup->channel[nodeIndex], poseGroup->order[nodeIndex]);
	//a3spatialPoseRestore(baseHS->localSpace->hpose_base + nodeIndex, poseGroup->channel[0], poseGroup->order[0]);

	a3spatialPoseDeconcat(activeHS->animPose->hpose_base + nodeIndex, activeHS->localSpace->hpose_base + nodeIndex, baseHS->localSpace->hpose_base + nodeIndex);



//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
}

void a3kinematicsUpdateLookAtIK(a3_HierarchyState const* sceneGraphState,
	a3_HierarchyState* activeHS, a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup,
	a3ui32 const sceneGraphIndex_hierarchyObj, a3ui32 const sceneGraphIndex_effector,
	a3ui32 const hierarchyObjIndex_affected, a3_Basis const basis_hierarchyObj, a3_Basis const basis_affected)
{
	a3mat3 m_hierarchyObj, m_affected;
	if (!a3basisToMat3(m_hierarchyObj.m, basis_hierarchyObj))
		return;
	if (!a3basisToMat3(m_affected.m, basis_affected))
		return;

	if ((!sceneGraphState || !activeHS || !baseHS || !poseGroup) ||
		(activeHS->hierarchy != baseHS->hierarchy) ||
		(activeHS->hierarchy != poseGroup->hierarchy))
		return;
	
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------
	//first step
	//move everything into the space of the skeleton/heiarcy
	//look at target

	//main step

	a3real4x4* hToRig = &sceneGraphState->localSpace->hpose_base[sceneGraphIndex_hierarchyObj].transformMat.m;
	a3real4x4* rigToH = &sceneGraphState->localSpaceInv->hpose_base[sceneGraphIndex_hierarchyObj].transformMat.m;

	a3vec4 effectorInH;

	//set to joint local space
	a3real4ProductTransform(effectorInH.v, &sceneGraphState->localSpace->hpose_base[sceneGraphIndex_effector].transformMat.v3.x, *rigToH);
	//move it to the space of our object

	//a3vec4 displacement;
	a3mat3 basis;
	a3vec4 temp;

	//Gets a vector from effector to joint --> joint local space
	a3real4Diff(&temp.x, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.x, effectorInH.v);
	
	//Normalize displacment vector to get direction --> join local space
	//a3real3Normalize(&temp.x);
	a3real3Set(&basis.v2.x, temp.x, temp.y, temp.z);

	//Gets right basis --> join localSpace by our local y axis
	a3real3Cross(&basis.v0.x, &m_affected.v2.x, &basis.v2.x);

	//3. up basis = Cross(direction and side)
	a3real3Cross(&basis.v1.x,
		&basis.v2.x,
		&basis.v0.x);

	//4. normailize
	a3real3Normalize(&basis.v0.x);
	a3real3Normalize(&basis.v1.x);
	a3real3Normalize(&basis.v2.x);

	a3mat4 finalJToObject;


	//Change the basis to the affected basis I think
	a3real3x3Product(basis.m, m_affected.m, basis.m);

	//Add the computed axes to the final joint to object mat (This is done wierdly where 
	// the z axis is stored in column 2, the y axis in column 3, and the x axis in column 0
	a3real4Set(&finalJToObject.v0.x, basis.v0.x, basis.v0.y, basis.v0.z, 0);
	a3real4Set(&finalJToObject.v2.x, basis.v1.x, basis.v1.y, basis.v1.z, 0);
	a3real4Set(&finalJToObject.v1.x, basis.v2.x, basis.v2.y, basis.v2.z, 0);

	//Set translation -->in joint local space
	a3real4Set(&finalJToObject.v3.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.x,
		activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.y,
		activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.z, 1);
	
	//set to object space
	//a3real4x4Product(finalJToObject.m, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.m, finalJToObject.m);

	a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected, finalJToObject.m);
	
//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
}

extern float acosf(float y_r);
extern float sinf(float y_r);
void a3kinematicsUpdateLimbIK(a3_HierarchyState const* sceneGraphState,
	a3_HierarchyState* activeHS, a3_HierarchyState const* baseHS, a3_HierarchyPoseGroup const* poseGroup,
	a3ui32 const sceneGraphIndex_hierarchyObj, a3ui32 const sceneGraphIndex_effector_end, a3ui32 const sceneGraphIndex_constraint,
	a3ui32 const hierarchyObjIndex_affected_end, a3ui32 const hierarchyObjIndex_affected_hinge, a3ui32 const hierarchyObjIndex_affected_base,
	a3_Basis const basis_hierarchyObj, a3_Basis const basis_affected_end, a3_Basis const basis_affected_hinge, a3_Basis const basis_affected_base)
{
	a3mat3 m_hierarchyObj, m_affected_end, m_affected_hinge, m_affected_base;
	if (!a3basisToMat3(m_hierarchyObj.m, basis_hierarchyObj))
		return;
	if (!a3basisToMat3(m_affected_end.m, basis_affected_end))
		return;
	if (!a3basisToMat3(m_affected_hinge.m, basis_affected_hinge))
		return;
	if (!a3basisToMat3(m_affected_base.m, basis_affected_base))
		return;

	if ((!sceneGraphState || !activeHS || !baseHS || !poseGroup) ||
		(activeHS->hierarchy != baseHS->hierarchy) ||
		(activeHS->hierarchy != poseGroup->hierarchy))
		return;
	
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-3: IMPLEMENT ME
//-----------------------------------------------------------------------------

	
	a3mat4 rigToH = sceneGraphState->localSpaceInv->hpose_base[sceneGraphIndex_hierarchyObj].transformMat;

	a3vec4 endEffectorPositionInH;
	a3vec4 constraintPositionInH;

	a3mat4 hToS = activeHS->localSpaceInv->hpose_base[hierarchyObjIndex_affected_base].transformMat;
	a3mat4 sToH = activeHS->localSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat;

	//this is right <3
	a3real4ProductTransform(endEffectorPositionInH.v, &sceneGraphState->localSpace->hpose_base[sceneGraphIndex_effector_end].transformMat.v3.x, rigToH.m);
	a3real4ProductTransform(constraintPositionInH.v, &sceneGraphState->localSpace->hpose_base[sceneGraphIndex_constraint].transformMat.v3.x, rigToH.m);

	//main step
	//solve joint-to-object for end, hinge, and base
	//check if we are in range?
	//->end  position*
	//->hinge position*

	//1. base joint to end effector vector distance
	a3vec3 baseToEnd;
	a3real3Diff(baseToEnd.v, endEffectorPositionInH.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x);

	//2. base joint to pole vector constraint
	a3vec3 baseToConstraint;
	a3real3Diff(baseToConstraint.v, constraintPositionInH.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x);

	//3. plane normal = cross(1,2)
	a3vec3 limbPlaneNormal;
	a3real3Cross(&limbPlaneNormal.x, baseToConstraint.v, baseToEnd.v);
	//a3real3Cross(&limbPlaneNormal.x, jToEnd.v, jToConstraint.v);

	a3real3Normalize(limbPlaneNormal.v);

	//a3real3Proj
	//4. LAW OF COSINES
	// -> solve elbow position
	
	//a3vec3 constraintToEndEffector; -- this could ve a issue 
	//a3real3Diff(baseToConstraint.v, endEffectorPositionInH.v, constraintPositionInH.v);


	//get the distance
	a3vec4 sholderToElbow;
	a3real4Diff(sholderToElbow.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v3.x, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x);

	a3vec4 elbowToWrist;
	a3real4Diff(elbowToWrist.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v3.x, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v3.x);


	//move wrist into sholder 
	a3vec4 wristPosition;
	a3real4ProductTransform(wristPosition.v, endEffectorPositionInH.v, hToS.m);

	//elbow position in sholder space;
	a3vec4 elbowPosition = activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v3;
	//a3real4ProductTransform(elbowPosition.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v3.x, hToS.m);


	//clamp wirst position :3

	a3real a = a3real3Length(sholderToElbow.v);
	a3real b = a3real3Length(elbowToWrist.v);
	a3real c = a3real3Length(baseToEnd.v);

	a3real denom = 2 * a * c;
	a3real cosAngle = 0;


	if (denom > 0.001)
	{
		cosAngle = ((a * a) + (c * c) - (b * b)) / denom;
	}

	a3real fullAngle = acosf(cosAngle);

	a3real x = a * cosAngle;
	a3real z = a * sinf(fullAngle);
	/*a3real x = a * a3rad2deg(cosAngle);
	a3real z = a * a3rad2deg(sinf(fullAngle));*/


	//set elbow position

	a3vec4 newElbowPos;//set the y to the effector y
	a3real4Set(newElbowPos.v, x, elbowPosition.y, z, 1);
	//a3real4Set(newElbowPos.v, elbowPosition.x, elbowPosition.y, elbowPosition.z, 1);

	//a3real4ProductTransform(elbowPosition.v, newElbowPos.v, sToH.m);

	//set new basis

	//a3real4ProductTransform(limbPlaneNormal.v, limbPlaneNormal.v, hToS.m);

	a3vec4 fwrd;
	a3real3Cross(fwrd.v, limbPlaneNormal.v, elbowToWrist.v);
	a3real4Normalize(fwrd.v);


	//GO BACK
	a3real4ProductTransform(fwrd.v, fwrd.v, sToH.m);
	a3real4ProductTransform(limbPlaneNormal.v, limbPlaneNormal.v, sToH.m);
	a3real4ProductTransform(elbowToWrist.v, elbowToWrist.v, sToH.m);
	a3real4ProductTransform(newElbowPos.v, newElbowPos.v, sToH.m);
	a3real4ProductTransform(sholderToElbow.v, sholderToElbow.v, sToH.m);

	a3real4Normalize(elbowToWrist.v);
	a3real4Normalize(sholderToElbow.v);

	//5.use the look at function formula and fix sholder and elbow rotations

	//last step
	//resolve every affected joint
	// ->do it in start closer to the root and go down

	//sholder
	a3mat4 finalJToObject;

	// up by forward --> get right
	a3vec3 right;
	a3real3Cross(right.v, &m_affected_base.v2.x, sholderToElbow.v);
	//right by forward -->forward
	a3vec3 up;
	a3real3Cross(up.v, sholderToElbow.v, right.v);

	a3real3Normalize(right.v);
	a3real3Normalize(up.v);
	a3real3Normalize(sholderToElbow.v);

	a3real4Set(&finalJToObject.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v0.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v0.z, 0);
	a3real4Set(&finalJToObject.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v2.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v2.z, 0);
	a3real4Set(&finalJToObject.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v1.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v1.z, 0);

	//set translation
	a3real4Set(&finalJToObject.v3.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x,
		activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.y,
		activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.z, 1);

	a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected_base, finalJToObject.m);


	//elbow
	a3mat4 finalJToObject2;
	a3real3Normalize(elbowToWrist.v);
	a3real3Normalize(limbPlaneNormal.v);
	a3real3Normalize(fwrd.v);

	a3real4Set(&finalJToObject2.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v0.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v0.z, 0);
	a3real4Set(&finalJToObject2.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v2.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v2.z, 0);
	a3real4Set(&finalJToObject2.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v1.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v1.z, 0);

	/*a3real4Set(&finalJToObject2.v3.x, 0,
		0,
		0, 1);*/
	a3real4Set(&finalJToObject2.v3.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x + elbowPosition.x,
		activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.y + elbowPosition.y,
		activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.z + elbowPosition.z, 1);


	//a3real4ProductTransform(&finalJToObject2.v3.x, &finalJToObject2.v3.x, hToS.m);
	a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected_hinge, finalJToObject2.m);

	//wirst
	a3real4Set(&finalJToObject2.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v0.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v0.z, 0);
	a3real4Set(&finalJToObject2.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v2.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v2.z, 0);
	a3real4Set(&finalJToObject2.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v1.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v1.z, 0);

	a3real4Set(&finalJToObject2.v3.x, endEffectorPositionInH.x,
		endEffectorPositionInH.y,
		endEffectorPositionInH.z, 1);

	//a3real4Set(&finalJToObject.v3.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v3.x,
	//	activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v3.y,
	//	activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v3.z, 1);

	a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected_end, finalJToObject2.m);
	
//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
}


//-----------------------------------------------------------------------------
