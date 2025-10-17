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

		/////THE FOLLOWING IS THE WAY I UNDERSTAND IT BUT LOOKS WRONG (compared to example)

		//move everything into the space of the skeleton/heiarcy
		//look at target

		//main step

		/*a3real4x4* hToRig = &sceneGraphState->localSpace->hpose_base[sceneGraphIndex_hierarchyObj].transformMat.m;
		a3real4x4* rigToH = &sceneGraphState->localSpaceInv->hpose_base[sceneGraphIndex_hierarchyObj].transformMat.m;

		a3vec4 effectorInH;

		//set to joint local space
		a3real4ProductTransform(effectorInH.v, &sceneGraphState->localSpace->hpose_base[sceneGraphIndex_effector].transformMat.v3.x, *rigToH);
		//move it to the space of our object

		a3mat3 jointBasis;
		a3real3Set(&jointBasis.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v0.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v0.z);
		a3real3Set(&jointBasis.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v1.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v1.z);
		a3real3Set(&jointBasis.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v2.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v2.z);

		//Puts the local basis of the affected joint into the space of the parent (from hierarchy space)
		//a3real3x3Product(m_affected.m, m_affected.m, jointBasis.m);

		//Puts basis relative to the current joint into hierarchy space
		a3real3x3Product(m_affected.m, jointBasis.m, m_affected.m);

		//a3vec4 displacement;
		a3mat3 basis;
		a3vec4 temp;

		//Gets a vector from effector to joint --> joint local space
		a3real4Diff(&temp.x, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.x, effectorInH.v);
		//a3real4ProductTransform(temp.v, temp.v, activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.m);

		//Normalize displacment vector to get direction --> join local space
		//a3real3Normalize(&temp.x);
		a3real3Set(&basis.v2.x, temp.x, temp.y, temp.z);
		a3real3Normalize(&basis.v2.x);

		//Gets right basis --> join localSpace by our local y axis
		a3real3Cross(&basis.v0.x, &m_affected.v2.x, &basis.v2.x);
		a3real3Normalize(&basis.v0.x);

		//3. up basis = Cross(direction and side)
		a3real3Cross(&basis.v1.x,
			&basis.v2.x,
			&basis.v0.x);

		//4. normailize
		a3real3Normalize(&basis.v0.x);
		a3real3Normalize(&basis.v1.x);
		a3real3Normalize(&basis.v2.x);

		a3mat4 finalJToObject;

		//Gets look at in relation to current joint basis by multiplying where botth basis are in heirarchy space
		a3real3x3Product(basis.m, m_affected.m, basis.m);

		//Set the new axes of the look matrix relative to the parent basis
		// (v1 and v2 are flipped because a3basisToMat3 stores up in v2 and fwd in v1)
		a3real4Set(&finalJToObject.v0.x, basis.v0.x, basis.v0.y, basis.v0.z, 0);
		a3real4Set(&finalJToObject.v2.x, basis.v1.x, basis.v1.y, basis.v1.z, 0);
		a3real4Set(&finalJToObject.v1.x, basis.v2.x, basis.v2.y, basis.v2.z, 0);

		//Transformes the new look at basis back to heirarchy space
		//a3real4x4Product(finalJToObject.m, activeHS->animPose->hpose_base[parentIndex].transformMat.m, finalJToObject.m);

		//Set translation -->in joint local space
		a3real4Set(&finalJToObject.v3.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.x,
			activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.y,
			activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.z, 1);

		//set to object space
		//a3real4x4Product(finalJToObject.m, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.m, finalJToObject.m);

		a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected, finalJToObject.m);*/




		/////THE FOLLOWING IS LOOKS RIGHT BUT I DON"T FULLY UNDERSTAND WHY (compared to example)

	a3real4x4* hToRig = &sceneGraphState->localSpace->hpose_base[sceneGraphIndex_hierarchyObj].transformMat.m;
	a3real4x4* rigToH = &sceneGraphState->localSpaceInv->hpose_base[sceneGraphIndex_hierarchyObj].transformMat.m;

	a3vec4 effectorInH;

	//set to joint local space
	a3real4ProductTransform(effectorInH.v, &sceneGraphState->localSpace->hpose_base[sceneGraphIndex_effector].transformMat.v3.x, *rigToH);
	//move it to the space of our object

	//a3vec4 displacement;
	a3mat3 basis;
	a3vec4 temp;

	a3ui32 parentIndex = activeHS->hierarchy->nodes[hierarchyObjIndex_affected].parentIndex;

	//Gets a vector from effector to joint --> joint local space
	a3real4Diff(&temp.x, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.x, effectorInH.v);

	//Normalize displacment vector to get direction --> join local space
	a3real3Set(&basis.v2.x, temp.x, temp.y, temp.z);
	a3real3Normalize(&basis.v2.x);

	//Gets right basis --> join localSpace by our local y axis
	a3real3Cross(&basis.v0.x, &a3vec3_y.x, &basis.v2.x);
	a3real3Normalize(&basis.v0.x);

	//3. up basis = Cross(direction and side)
	a3real3Cross(&basis.v1.x,
		&basis.v2.x,
		&basis.v0.x);

	//4. normailize
	a3real3Normalize(&basis.v0.x);
	a3real3Normalize(&basis.v1.x);
	a3real3Normalize(&basis.v2.x);

	a3mat4 finalJToObject;

	a3mat3 jointBasis;
	a3real3Set(&jointBasis.v0.x, activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.v0.x, activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.v0.y, activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.v0.z);
	a3real3Set(&jointBasis.v1.x, activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.v1.x, activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.v1.y, activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.v1.z);
	a3real3Set(&jointBasis.v2.x, activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.v2.x, activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.v2.y, activeHS->objectSpaceInv->hpose_base[parentIndex].transformMat.v2.z);

	//Puts the undo rotation of the parent joint into the space of the local 
	// joint basis (Makes the rotation of the parent reversable for the local basis) I think?? 
	a3real3x3Product(m_affected.m, m_affected.m, jointBasis.m);

	//Makes new look at basis relative to the local basis of the current 
	// joint (with no additional rotation from parent)
	a3real3x3Product(basis.m, m_affected.m, basis.m);

	//Set the new axes of the look matrix relative to the parent basis
	// (v1 and v2 are flipped because a3basisToMat3 stores up in v2 and fwd in v1)
	a3real4Set(&finalJToObject.v0.x, basis.v0.x, basis.v0.y, basis.v0.z, 0);
	a3real4Set(&finalJToObject.v2.x, basis.v1.x, basis.v1.y, basis.v1.z, 0);
	a3real4Set(&finalJToObject.v1.x, basis.v2.x, basis.v2.y, basis.v2.z, 0);

	//Transformes the new look at basis to heirarchy space with no rotation from the parent (maybe??)
	a3real4x4Product(finalJToObject.m, activeHS->objectSpace->hpose_base[parentIndex].transformMat.m, finalJToObject.m);

	//Set translation -->in joint local space
	a3real4Set(&finalJToObject.v3.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.x,
		activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.y,
		activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected].transformMat.v3.z, 1);
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
	//a3real3x3Product(m_affected_base.m, m_affected_base.m, axisRigToH.m);

	a3vec4 endEffectorPositionInH;
	a3vec4 constraintPositionInH;

	//a3mat4 hToS = activeHS->localSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat;
	//a3mat4 sToH = activeHS->localSpaceInv->hpose_base[parentIndex].transformMat;

	//this is right <3
	a3real4TransformProduct(endEffectorPositionInH.v, rigToH.m, &sceneGraphState->localSpace->hpose_base[sceneGraphIndex_effector_end].transformMat.v3.x);
	a3real4TransformProduct(constraintPositionInH.v, rigToH.m, &sceneGraphState->localSpace->hpose_base[sceneGraphIndex_constraint].transformMat.v3.x);


	//create plane normal
	a3vec3 sholderToEffector;
	a3real3Diff(sholderToEffector.v, endEffectorPositionInH.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x);
	a3real3Normalize(sholderToEffector.v);

	a3vec3 sholderToConstraint;
	a3real3Diff(sholderToConstraint.v, constraintPositionInH.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x);
	a3real3Normalize(sholderToConstraint.v);


	a3vec3 planeNormal;
	a3real3Cross(planeNormal.v, sholderToEffector.v, sholderToConstraint.v);
	a3real3Normalize(planeNormal.v);

	a3vec3 baseToHinge;
	a3real3Diff(baseToHinge.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v3.x, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x);

	a3vec3 hingeToEnd;
	a3real3Diff(hingeToEnd.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v3.x, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v3.x);


	//HARRONS FORUMLA
	a3real L1 = a3real3Length(baseToHinge.v);
	a3real L2 = a3real3Length(hingeToEnd.v);

	a3real add = L1 + L2;
	a3vec3 c;
	a3real3Diff(c.v, endEffectorPositionInH.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x);

	a3real realLenght = a3real3Length(c.v);

	if (realLenght > add)
	{
		//was too long :/
		return;
	}

	
	//base
	a3vec3 mag;
	a3real3Diff(mag.v, endEffectorPositionInH.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x);

	a3real B = a3real3Length(mag.v);
	a3real s = ((a3real)1/(a3real)2) * (B + L1 + L2);
	//area
	a3real A = s * (s - B) * (s - L1) * (s - L2);
	A = a3sqrtf(A);

	//height of tri
	a3real H = (2 * A) / B;

	a3real L1sqr = (L1 * L1);
	a3real Hsqr = (H * H);
	a3real D = a3sqrtf(L1sqr - Hsqr);

	//pythag
	a3vec3 h;
	a3real3Cross(h.v, planeNormal.v, sholderToEffector.v);

	a3vec3 Dd;
	a3real3ProductS(Dd.v, sholderToEffector.v, D);


	a3vec3 Hh;
	a3real3ProductS(Hh.v, h.v, H);

	a3vec3 HhDd;
	a3real3Sum(HhDd.v, Hh.v, Dd.v);

	a3vec3 newHingePosition;
	a3real3Sum(newHingePosition.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x, HhDd.v);


	//sholder	
	//create a up vector
	a3mat4 finalJToObject;
	

	a3vec3 hingeToBase;
	a3real3Diff(hingeToBase.v, &activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x, newHingePosition.v);
	a3real3Normalize(hingeToBase.v);

	a3vec3 up;
	a3real3Cross(up.v, planeNormal.v, hingeToBase.v);

	a3mat3 rotation;
	a3real3Set(&rotation.v0.x, hingeToBase.x, hingeToBase.y, hingeToBase.z);
	a3real3Set(&rotation.v1.x, up.x, up.y, up.z);
	a3real3Set(&rotation.v2.x, planeNormal.x, planeNormal.y, planeNormal.z);


	a3mat3 jointBasis;
	//a3ui32 parentIndex = activeHS->hierarchy->nodes[hierarchyObjIndex_affected_base].parentIndex;
	a3real3Set(&jointBasis.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v0.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v0.z);
	a3real3Set(&jointBasis.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v1.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v1.z);
	a3real3Set(&jointBasis.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v2.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v2.z);
	
	//a3real3x3Product(m_affected_base.m, jointBasis.m, m_affected_base.m);
	//a3real3x3Product(rotation.m, m_affected_base.m, rotation.m);

	a3real4Set(&finalJToObject.v0.x, rotation.v0.x, rotation.v0.y, rotation.v0.z, 0);
	a3real4Set(&finalJToObject.v2.x, rotation.v1.x, rotation.v1.y, rotation.v1.z, 0);
	a3real4Set(&finalJToObject.v1.x, rotation.v2.x, rotation.v2.y, rotation.v2.z, 0);
	
	//a3real4x4Product(finalJToObject.m, activeHS->objectSpace->hpose_base[parentIndex].transformMat.m, finalJToObject.m);

	//set translations
	a3real4Set(&finalJToObject.v3.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.x,
		activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.y,
		activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_base].transformMat.v3.z, 1);
	
	a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected_base, finalJToObject.m);

	//elbow
	a3vec3 wristToHinge;
	a3real3Diff(wristToHinge.v, newHingePosition.v, endEffectorPositionInH.v);
	a3real3Normalize(wristToHinge.v);

	a3real3Cross(up.v, planeNormal.v, wristToHinge.v);

	a3real3Set(&rotation.v0.x, wristToHinge.x, wristToHinge.y, wristToHinge.z );
	a3real3Set(&rotation.v1.x, up.x, up.y, up.z);
	a3real3Set(&rotation.v2.x, planeNormal.x, planeNormal.y, planeNormal.z);

	//parentIndex = activeHS->hierarchy->nodes[hierarchyObjIndex_affected_hinge].parentIndex;
	a3real3Set(&jointBasis.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v0.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v0.z);
	a3real3Set(&jointBasis.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v1.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v1.z);
	a3real3Set(&jointBasis.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v2.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_hinge].transformMat.v2.z);

	//a3real3x3Product(m_affected_hinge.m, jointBasis.m, m_affected_hinge.m);
	//a3real3x3Product(rotation.m, m_affected_hinge.m, rotation.m);

	a3real4Set(&finalJToObject.v0.x, rotation.v0.x, rotation.v0.y, rotation.v0.z, 0);
	a3real4Set(&finalJToObject.v2.x, rotation.v1.x, rotation.v1.y, rotation.v1.z, 0);
	a3real4Set(&finalJToObject.v1.x, rotation.v2.x, rotation.v2.y, rotation.v2.z, 0);

	a3real4Set(&finalJToObject.v3.x, 
		newHingePosition.x,
		newHingePosition.y,
		newHingePosition.z, 1);

	a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected_hinge, finalJToObject.m);

	
	//wrist

	a3real4Set(&finalJToObject.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v0.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v0.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v0.z,0);
	a3real4Set(&finalJToObject.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v1.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v1.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v1.z,0);
	a3real4Set(&finalJToObject.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v2.x, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v2.y, activeHS->objectSpace->hpose_base[hierarchyObjIndex_affected_end].transformMat.v2.z,0);

	a3real4Set(&finalJToObject.v3.x, endEffectorPositionInH.x,
		endEffectorPositionInH.y,
		endEffectorPositionInH.z, 1);


	a3kinematicsResolvePostIK(activeHS, baseHS, poseGroup, hierarchyObjIndex_affected_end, finalJToObject.m);
	
//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-3
//-----------------------------------------------------------------------------
}


//-----------------------------------------------------------------------------
