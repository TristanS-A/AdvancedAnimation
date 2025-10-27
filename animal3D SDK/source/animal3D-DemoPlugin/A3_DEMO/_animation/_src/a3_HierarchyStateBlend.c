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
	
	a3_HierarchyStateBlend.c
	Implementation of hierarchical blend operations.
*/

#include "../a3_HierarchyStateBlend.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-4: IMPLEMENT ME
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------

// create blend tree
a3ret a3spatialPoseBlendTreeCreate(a3_SpatialPoseBlendTree* blendTree, a3_Hierarchy const* blendTreeDescriptor)
{
	if (!blendTree || !blendTreeDescriptor)
		return -1;

	blendTree->blendTreeDescriptor = blendTreeDescriptor;

	//Allocates the amount of memory for the number of nodes
	const a3ui32 dataSize = sizeof(a3_SpatialPoseBlendNode) * blendTreeDescriptor->numNodes;
	blendTree->nodes = (a3_SpatialPoseBlendNode*)malloc(dataSize);
	memset(blendTree->nodes, 0, dataSize);

	return 0;
}

// release blend tree
a3ret a3spatialPoseBlendTreeRelease(a3_SpatialPoseBlendTree* blendTree)
{
	if (!blendTree)
		return -1;
	if (!blendTree->nodes)
		return -1;

	//Free's memory
	free(blendTree->nodes);

	//un set the herarchy and nodes
	blendTree->blendTreeDescriptor = 0;
	blendTree->nodes = 0;

	return 0;
}

// configure node internally; set pointers
a3ret a3spatialPoseBlendTreeConfigureNode(a3_SpatialPoseBlendTree const* blendTree, a3ui32 const nodeIndex,
	a3_SpatialPose const* inPose1, a3_SpatialPose const* inPose2, a3_SpatialPose* outPose, const a3_BlendOpSet* blendOpSet)
{
	if (!blendTree)
		return -1;

	blendTree->nodes[nodeIndex].blendOpSet = blendOpSet;

	//Configures input and output pointers for node
	blendTree->nodes[nodeIndex].pose_out = outPose; //for set up this shoudl be 
	blendTree->nodes[nodeIndex].pose_ctrl[0] = inPose1; //this should be pose out parent1
	blendTree->nodes[nodeIndex].pose_ctrl[1] = inPose2; //this should be pose out parrent2
	blendTree->nodes[nodeIndex].vCount = 2;


	return 0;
}

// does not execute tree from leaves to root
a3ret a3spatialPoseBlendTreeExecute(a3_SpatialPoseBlendTree const* blendTree, a3real u)
{
	if (!blendTree)
		return -1;

	//get the root node
	a3_SpatialPoseBlendNode* root = blendTree->nodes;
	a3_BlendOp op;
	for (a3ui32 i = 0; i < blendTree->blendTreeDescriptor->numNodes; i++)
	{
		blendTree->nodes[i].u[0] = &u;
		blendTree->nodes[i].uCount = 1;

		//create blend op for rotate
		
		op.op = blendTree->nodes[i].blendOpSet->op_rotate;
		op.v_out = &blendTree->nodes[i].pose_out->rotate.r;
		
		//fill conrols
		for (int j = 0; j < blendTree->nodes[i].vCount; j++)
		{
			op.v_ctrl[j] = &blendTree->nodes[i].pose_ctrl[j]->rotate.x;
		}

		for (int j = 0; j < blendTree->nodes[i].uCount; j++)
		{
			op.u[j] = blendTree->nodes[i].u[j];
		}

		op.vCount = blendTree->nodes[i].vCount;
		op.uCount = blendTree->nodes[i].uCount;

		op.exec = blendTree->nodes[i].blendOpSet->exec;
		blendTree->nodes[i].blendOpSet->exec(&op);
	}

	return 0;
}


//-----------------------------------------------------------------------------

a3real4r a3blendOpRET4(a3real4 v_out)
{
	return v_out;//already completed
}

a3real4r a3blendOpZERO4(a3real4 v_out)
{

	return v_out;
}

a3real4r a3blendOpONE4(a3real4 v_out)
{
	return v_out;
}

a3real4r a3blendOpID4(a3real4 v_out)
{
	return v_out;
}

a3real4r a3blendOpCOPY4(a3real4 v_out, a3real4 const v)
{
	return v_out;
}

a3real4r a3blendOpNEGATE4(a3real4 v_out, a3real4 const v)
{
	return v_out;
}

a3real4r a3blendOpRECIP4(a3real4 v_out, a3real4 const v)
{
	return v_out;
}

a3real4r a3blendOpCONJQ4(a3real4 v_out, a3real4 const v)
{
	return v_out;
}

a3real4r a3blendOpADD4(a3real4 v_out, a3real4 const v0, a3real4 const v1)
{
	return v_out;
}

a3real4r a3blendOpSUB4(a3real4 v_out, a3real4 const v0, a3real4 const v1)
{
	return v_out;
}

a3real4r a3blendOpMUL4(a3real4 v_out, a3real4 const v0, a3real4 const v1)
{
	return v_out;
}

a3real4r a3blendOpDIV4(a3real4 v_out, a3real4 const v0, a3real4 const v1)
{
	return v_out;
}

a3real4r a3blendOpMULQ4(a3real4 v_out, a3real4 const v0, a3real4 const v1)
{
	return v_out;
}

a3real4r a3blendOpMULCONJQ4(a3real4 v_out, a3real4 const v0, a3real4 const v1)
{
	return v_out;
}

a3real4r a3blendOpSCALE4(a3real4 v_out, a3real4 const v, a3real const u)
{
	return v_out;
}

a3real4r a3blendOpPOW4(a3real4 v_out, a3real4 const v, a3real const u)
{
	return v_out;
}

a3real4r a3blendOpNEAR4(a3real4 v_out, a3real4 const v0, a3real4 const v1, a3real const u)
{
	return v_out;
}

a3real4r a3blendOpLERP4(a3real4 v_out, a3real4 const v0, a3real4 const v1, a3real const u)
{
	 v_out[0] = (v1[0] - v0[0]) * u + v0[0];
	 v_out[1] = (v1[1] - v0[1]) * u + v0[1];
	 v_out[2] = (v1[2] - v0[2]) * u + v0[2];

	return v_out;
}

a3real4r a3blendOpNLERP4(a3real4 v_out, a3real4 const v0, a3real4 const v1, a3real const u)
{
	return v_out;
}

a3real4r a3blendOpSLERP4(a3real4 v_out, a3real4 const v0, a3real4 const v1, a3real const u)
{
	return v_out;
}


a3real4r a3blendOpRET4X2(a3real4 m_out)
{
	return m_out;//already completed
}

a3real4r a3blendOpID4X2(a3real4 m_out)
{
	a3blendOpID4(m_out + 0);
	a3blendOpZERO4(m_out + 4);
	return m_out;
}

a3real4r a3blendOpCOPY4X2(a3real4 m_out, a3real4 const m)
{
	a3blendOpCOPY4(m_out + 0, m + 0);
	a3blendOpCOPY4(m_out + 4, m + 4);
	return m_out;
}

a3real4r a3blendOpCONJDQ4X2(a3real4 m_out, a3real4 const m)
{
	a3blendOpCONJQ4(m_out + 0, m + 0);
	a3blendOpCOPY4(m_out + 4, m + 4);
	return m_out;
}

a3real4r a3blendOpMULDQ4X2(a3real4 m_out, a3real4 const m0, a3real4 const m1)
{
	// (a + bE)(c + dE)
	//	= ac + adE + bcE + bdE^2
	//	= ac + (ad + bc)E
	return m_out;
}

a3real4r a3blendOpMULCONJDQ4X2(a3real4 m_out, a3real4 const m0, a3real4 const m1)
{
	return m_out;
}

a3real4r a3blendOpSCALE4X2(a3real4 m_out, a3real4 const m, a3real const u)
{
	return m_out;
}

a3real4r a3blendOpNEAR4X2(a3real4 m_out, a3real4 const m0, a3real4 const m1, a3real const u)
{
	return m_out;
}

a3real4r a3blendOpLERP4X2(a3real4 m_out, a3real4 const m0, a3real4 const m1, a3real const u)
{
	return m_out;
}

a3real4r a3blendOpNLERP4X2(a3real4 m_out, a3real4 const m0, a3real4 const m1, a3real const u)
{
	return m_out;
}

a3real4r a3blendOpSCLERP4X2(a3real4 m_out, a3real4 const m0, a3real4 const m1, a3real const u)
{
	// sclerp(m0, m1, u)
	return m_out;
}


a3real4r a3blendOpRET4X4(a3real4 m_out)
{
	return m_out;//already completed
}

a3real4r a3blendOpID4X4(a3real4 m_out)
{
	return m_out;
}

a3real4r a3blendOpCOPY4X4(a3real4 m_out, a3real4 const m)
{
	return m_out;
}

a3real4r a3blendOpINVR4X4(a3real4 m_out, a3real4 const m)
{
	return m_out;
}

a3real4r a3blendOpMULM4X4(a3real4 m_out, a3real4 const m0, a3real4 const m1)
{
	return m_out;
}

a3real4r a3blendOpMULINVR4X4(a3real4 m_out, a3real4 const m0, a3real4 const m1)
{
	return m_out;
}

a3real4r a3blendOpSCALE4X4(a3real4 m_out, a3real4 const m, a3real const u)
{
	return m_out;
}

a3real4r a3blendOpNEAR4X4(a3real4 m_out, a3real4 const m0, a3real4 const m1, a3real const u)
{
	return m_out;
}

a3real4r a3blendOpLERP4X4(a3real4 m_out, a3real4 const m0, a3real4 const m1, a3real const u)
{
	return m_out;
}


//-----------------------------------------------------------------------------

// pointer-based reset/identity operation for single spatial pose
a3_SpatialPose* a3spatialPoseOpIdentity(a3_SpatialPose* pose_out)
{
	pose_out->transformMat = a3mat4_identity;
	// ...
	pose_out->rotate = a3vec4_one;
	pose_out->scale = a3vec4_one;
	pose_out->translate = a3vec4_one;

	// done
	return pose_out;
}

// pointer-based LERP operation for single spatial pose
a3_SpatialPose* a3spatialPoseOpLERP(a3_SpatialPose* pose_out, a3_SpatialPose const* pose0, a3_SpatialPose const* pose1, a3real const u)
{
	//will wrote
	a3spatialPoseLerp(pose_out, pose0, pose1, u);

	// done
	return pose_out;
}


//-----------------------------------------------------------------------------

// pointer-based reset/identity operation for hierarchical pose
a3_HierarchyPose* a3hierarchyPoseOpIdentity(a3_HierarchyPose* pose_out)
{
	//this might need to be for each hpose
	pose_out->hpose_base->transformMat = a3mat4_identity;
	// ...
	pose_out->hpose_base->rotate = a3vec4_one;
	pose_out->hpose_base->scale = a3vec4_one;
	pose_out->hpose_base->translate = a3vec4_one;
	// done
	return pose_out;
}

// pointer-based LERP operation for hierarchical pose
a3_HierarchyPose* a3hierarchyPoseOpLERP(a3_HierarchyPose* pose_out, a3_HierarchyPose const* pose0, a3_HierarchyPose const* pose1, a3real const u)
{
	//mising a node group?
	//a3hierarchyPoseLerp(pose_out, pose0, pose1, u)

	// angles: lerp is ok for the purposes of what we're doing
		// to-do: check channels
	//pose_out->hpose_base->rotate.v[0] = (pose1->hpose_base->rotate.v[0] - pose0->hpose_base->rotate.v[0]) * u + pose0->hpose_base->rotate.v[0];
	//pose_out->hpose_base->rotate.v[1] = (pose1->hpose_base->rotate.v[1] - pose0->hpose_base->rotate.v[1]) * u + pose0->hpose_base->rotate.v[1];
	//pose_out->hpose_base->rotate.v[2] = (pose1->hpose_base->rotate.v[2] - pose0->hpose_base->rotate.v[2]) * u + pose0->hpose_base->rotate.v[2];

	//// scale: log-lerp
	//// to-do: check channels
	//pose_out->hpose_base->scale.v[0] = powf(pose1->hpose_base->scale.v[0] / pose0->hpose_base->scale.v[0], u) * pose0->hpose_base->scale.v[0];
	//pose_out->hpose_base->scale.v[1] = powf(pose1->hpose_base->scale.v[1] / pose0->hpose_base->scale.v[1], u) * pose0->hpose_base->scale.v[1];
	//pose_out->hpose_base->scale.v[2] = powf(pose1->hpose_base->scale.v[2] / pose0->hpose_base->scale.v[2], u) * pose0->hpose_base->scale.v[2];

	//// translate: lerp
	//// to-do: check channels
	//pose_out->hpose_base->translate.v[0] = (pose1->hpose_base->translate.v[0] - pose0->hpose_base->translate.v[0]) * u + pose0->hpose_base->translate.v[0];
	//pose_out->hpose_base->translate.v[1] = (pose1->hpose_base->translate.v[1] - pose0->hpose_base->translate.v[1]) * u + pose0->hpose_base->translate.v[1];
	//pose_out->hpose_base->translate.v[2] = (pose1->hpose_base->translate.v[2] - pose0->hpose_base->translate.v[2]) * u + pose0->hpose_base->translate.v[2];

	// done
	return pose_out;
}


//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-4
//-----------------------------------------------------------------------------
