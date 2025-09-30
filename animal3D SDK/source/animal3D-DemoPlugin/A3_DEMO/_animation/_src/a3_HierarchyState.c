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
	
	a3_HierarchyState.c
	Implementation of transform hierarchy state.
*/

#include "../a3_HierarchyState.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>


//-----------------------------------------------------------------------------

// initialize pose set given an initialized hierarchy and key pose count
a3i32 a3hierarchyPoseGroupCreate(a3_HierarchyPoseGroup *poseGroup_out, const a3_Hierarchy *hierarchy, const a3ui32 poseCount)
{
	// validate params and initialization states
	//	(output is not yet initialized, hierarchy is initialized)
	if (poseGroup_out && hierarchy && !poseGroup_out->hierarchy && hierarchy->nodes)
	{
		// determine memory requirements
		a3ui32 const nodeCount = hierarchy->numNodes;
		a3ui32 const hposeCount = poseCount, hposeSpace = sizeof(a3_HierarchyPose) * hposeCount;
		a3ui32 const sposeCount = hposeCount * nodeCount, sposeSpace = sizeof(a3_SpatialPose) * sposeCount;
		a3ui32 const channelSpace = sizeof(a3_SpatialPoseChannel) * nodeCount;
		a3ui32 const orderSpace = sizeof(a3_SpatialPoseEulerOrder) * nodeCount;
		a3ui32 const memreq = hposeSpace + sposeSpace + channelSpace + orderSpace;
		a3index i;

		// allocate everything (one malloc)
		poseGroup_out->hpose = (a3_HierarchyPose*)malloc(memreq);
		poseGroup_out->hpose->hpose_base = poseGroup_out->pose = (a3_SpatialPose*)(poseGroup_out->hpose + hposeCount);
		poseGroup_out->channel = (a3_SpatialPoseChannel*)(poseGroup_out->pose + sposeCount);
		poseGroup_out->order = (a3_SpatialPoseEulerOrder*)(poseGroup_out->channel + nodeCount);

		// set pointers
		for (i = 1; i < hposeCount; ++i)
		{
			poseGroup_out->hpose[i].hpose_base = poseGroup_out->hpose[i - 1].hpose_base + nodeCount;
			poseGroup_out->hpose[i].hpose_index = i * nodeCount;
		}

		// reset all data
		a3hierarchyPoseReset(poseGroup_out->hpose, sposeCount);
		memset(poseGroup_out->channel, a3poseChannel_none, channelSpace);
		memset(poseGroup_out->order, a3poseEulerOrder_xyz, orderSpace);
		poseGroup_out->hierarchy = hierarchy;
		poseGroup_out->hposeCount = hposeCount;
		poseGroup_out->poseCount = sposeCount;

		// done
		return 1;
	}
	return -1;
}

// release pose set
a3i32 a3hierarchyPoseGroupRelease(a3_HierarchyPoseGroup *poseGroup)
{
	// validate param exists and is initialized
	if (poseGroup && poseGroup->hierarchy)
	{
		// release everything (one free)
		free(poseGroup->hpose);

		// reset pointers
		poseGroup->hierarchy = 0;
		poseGroup->hpose = 0;
		poseGroup->pose = 0;
		poseGroup->channel = 0;
		poseGroup->order = 0;

		// done
		return 1;
	}
	return -1;
}

// load binary
a3i32 a3hierarchyPoseGroupLoadBinary(a3_HierarchyPoseGroup* poseGroup, a3_FileStream const* fileStream)
{
	FILE* fp;
	a3ui32 ret = 0;
	a3ui32 dataSize = 0;
	if (poseGroup && fileStream)
	{
		if (poseGroup->hierarchy && !poseGroup->pose)
		{
			fp = fileStream->stream;
			if (fp)
			{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-OPTIONAL: IMPLEMENT ME
//-----------------------------------------------------------------------------
				


//-----------------------------------------------------------------------------
//****END-TO-DO-OPTIONAL
//-----------------------------------------------------------------------------
			}
			return ret;
		}
	}
	return -1;
}

// save binary
a3i32 a3hierarchyPoseGroupSaveBinary(a3_HierarchyPoseGroup const* poseGroup, a3_FileStream const* fileStream)
{
	FILE* fp;
	a3ui32 ret = 0;
	if (poseGroup && fileStream)
	{
		if (poseGroup->hierarchy && poseGroup->pose)
		{
			fp = fileStream->stream;
			if (fp)
			{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-OPTIONAL: IMPLEMENT ME
//-----------------------------------------------------------------------------
				
				//fread

//-----------------------------------------------------------------------------
//****END-TO-DO-OPTIONAL
//-----------------------------------------------------------------------------
			}
			return ret;
		}
	}
	return -1;
}


//-----------------------------------------------------------------------------

// initialize hierarchy state given an initialized hierarchy
a3i32 a3hierarchyStateCreate(a3_HierarchyState *state_out, const a3_Hierarchy *hierarchy)
{
	// validate params and initialization states
	//	(output is not yet initialized, hierarchy is initialized)
	if (state_out && hierarchy && !state_out->hierarchy && hierarchy->nodes)
	{
		// determine memory requirements
		a3ui32 const nodeCount = hierarchy->numNodes;
		a3ui32 const hposeCount = sizeof(state_out->hpose) / sizeof(a3_HierarchyPose);
		a3ui32 const sposeCount = hposeCount * nodeCount;
		a3ui32 const memreq = sizeof(a3_SpatialPose) * sposeCount;
		a3index i;

		// allocate everything (one malloc)
		state_out->hpose->hpose_base = (a3_SpatialPose*)malloc(memreq);

		// set pointers
		state_out->hierarchy = hierarchy;
		for (i = 1; i < hposeCount; ++i)
		{
			state_out->hpose[i].hpose_base = state_out->hpose[i - 1].hpose_base + nodeCount;
			state_out->hpose[i].hpose_index = i * nodeCount;
		}

		// reset all data
		a3hierarchyPoseReset(state_out->hpose, sposeCount);

		// done
		return 1;
	}
	return -1;
}

// release hierarchy state
a3i32 a3hierarchyStateRelease(a3_HierarchyState *state)
{
	// validate param exists and is initialized
	if (state && state->hierarchy)
	{
		a3ui32 const hposeCount = sizeof(state->hpose) / sizeof(a3_HierarchyPose);
		a3ui32 i;

		// release everything (one free)
		free(state->hpose->hpose_base);

		// reset pointers
		state->hierarchy = 0;
		for (i = 0; i < hposeCount; ++i)
		{
			state->hpose[i].hpose_base = 0;
			state->hpose[i].hpose_index = 0;
		}

		// done
		return 1;
	}
	return -1;
}


//-----------------------------------------------------------------------------

// update inverse local-space matrices
a3i32 a3hierarchyStateUpdateLocalInverse(const a3_HierarchyState* state)
{
	if (state && state->hierarchy)
	{
		a3index i = 0;
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------
		
		//Tristan added this
		for (i = 0; i < state->hierarchy->numNodes; i++)
		{
			a3real4x4TransformInverse(state->localSpaceInv->hpose_base[i].transformMat.m, state->localSpace->hpose_base[i].transformMat.m);
		}

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
		return i;
	}
	return -1;
}

// update inverse object-space matrices
a3i32 a3hierarchyStateUpdateObjectInverse(const a3_HierarchyState* state)
{
	if (state && state->hierarchy)
	{
		a3index i = 0;
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------
		
		for (i = 0; i < state->hierarchy->numNodes; i++)
		{
			a3real4x4TransformInverse(state->objectSpaceInv->hpose_base[i].transformMat.m, state->objectSpace->hpose_base[i].transformMat.m);
		}

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
		return i;
	}
	return -1;
}

// update inverse object-space bind-to-current matrices
a3i32 a3hierarchyStateUpdateObjectBindToCurrent(const a3_HierarchyState* state, const a3_HierarchyState* state_bind)
{
	if (state && state->hierarchy && state_bind && state_bind->hierarchy)
	{
		a3index i = 0;
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------
		
		for (i = 0; i < state->hierarchy->numNodes; i++)
		{
			a3real4x4Product(state->objectSpaceBindToCurrent->hpose_base[i].transformMat.m, state->objectSpace->hpose_base[i].transformMat.m, state_bind->objectSpaceInv->hpose_base[i].transformMat.m);
			
		}

//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
		return i;
	}
	return -1;
}

//TODO change to cammal case
a3byte ProcessHeaders(char* currLine, int currLineLength, FILE* pFile, a3_HierarchyPoseGroup* group, a3_Hierarchy* h, float* out_conversion)
{
	//Reset currLine determining char
	currLine[0] = ' ';

	//Defines header buffer to use sscanf properly
	char header[50];
	char value[50];

	float conversion = 1;
	float scaleFactor = 1;
	//Continues processing headers until next type of data is detected
	while (currLine[0] != '[' && currLine[0] != '#')
	{
		//Gets next line to process
		fgets(currLine, currLineLength, pFile);

		//Seperates the header from the value (this only works for a header with no spaces and a single value per header)
		//Can also have used sscanf(currLine, "%s %d", header, &fileV) instead of this
		int prevSpaceIndex = 0;
		header[0] = -52;  //Resets header and value processing
		value[0] = -52;
		for (int i = 0; i < currLineLength; i++)
		{
			if (currLine[i] == ' ' || currLine[i] == '\r' || currLine[i] == -52)
			{
				if (header[0] == -52)
				{
					strncpy(header, currLine + prevSpaceIndex, i - prevSpaceIndex);
				}
				else if (value[0] == -52)
				{
					strncpy(value, currLine + prevSpaceIndex, i - prevSpaceIndex);

				}
				else
				{
					break;
				}

				prevSpaceIndex = i;
			}
		}

		//Process headers
		if (strstr(header, "FileType"))
		{
			if (!strstr(value, "HTR"))
			{
				printf("not correct file type");
				return false;
			}
		}
		else if (strstr(header, "DataType"))
		{
			if (!strstr(value, "HTRS"))
			{
				//bad data
				return false;
			}
		}
		else if (strstr(header, "FileVersion"))
		{
			if (atoi(value) != 1)
			{
				//bad file
				return false;
			}
		}
		else if (strstr(header, "NumSegments"))
		{

			a3ui32 amount = (a3ui32)atoi(value);
			a3hierarchyCreate(h, amount, NULL);
			
		}
		else if (strstr(currLine, "NumFrames"))
		{
			a3hierarchyPoseGroupCreate(group, h, (a3ui32)atoi(value));

		}
		else if (strstr(currLine, "DataFrameRate"))
		{
			//TODO
		}
		else if (strstr(currLine, "EulerRotationOrder"))
		{
			if (strstr(currLine, "ZYX"))
			{
				*group->order = a3poseEulerOrder_zyx;
			}
			else if (strstr(currLine, "XYZ"))
			{
				*group->order = a3poseEulerOrder_xyz;
			}
		}
		else if (strstr(currLine, "CalibrationUnits"))
		{
			//TODO add more mesurements
			if (strstr(value, "mm"))
			{
				//convert to cm
				conversion = 1.0f / 10.0f;
			}
			else if (strstr(value, "cm"))
			{
				//no need
				conversion = 1.0f;
			}
			
		}
		else if (strstr(currLine, "RotationUnits"))
		{
			//TODO
			if (!strstr(currLine, "Degrees")) 
			{
			}
		}
		else if (strstr(currLine, "GlobalAxisofGravity"))
		{
			//set scale factor 
		}
		else if (strstr(currLine, "BoneLengthAxis"))
		{
			//set scale factor 
			*group->channel = a3poseChannel_scale_y;
		}
		else if (strstr(currLine, "ScaleFactor"))
		{
			//set scale factor
			scaleFactor = (a3real)atof(value);
		}
	}

	*out_conversion = scaleFactor * conversion;
	return true;
}

a3byte ProcessSegmentsAndHeirarchy(char* currLine, int currLintLength, FILE* pFile, a3_Hierarchy* h)
{
	char parent[a3node_nameSize];
	char child[a3node_nameSize];

	//Reset currLine determining char
	currLine[0] = ' ';

	a3i32 i = 0;
	a3i32 parentIndex = 0;
	while (currLine[0] != '[' && currLine[0] != '#')
	{
		//Gets next line to process
		fgets(currLine, currLintLength, pFile);

		//Reads child and parent names
		sscanf(currLine, "%s %s", &child, &parent);

		//Gets parent index
		parentIndex = a3hierarchyGetNodeIndex(h, parent);

		//Sets current node
		a3hierarchySetNode(h, i, parentIndex, child);

		i++;
	}

	return true;
}

a3byte ProcessBasePositions(char* currLine, int currLintLength, FILE* pFile, a3_HierarchyPoseGroup* group, a3_Hierarchy* h, float scalor)
{
	//Reset currLine determining char
	currLine[0] = ' ';

	char name[a3node_nameSize];
	a3vec3 pos;
	a3vec3 rot;
	a3f32 scale;

	int counter = 0;
	while (currLine[0] != '[' && currLine[0] != '#')
	{
		//Gets next line to process
		fgets(currLine, currLintLength, pFile);

		//Reads base pose data
		sscanf(currLine, "%s %f %f %f %f %f %f %f", name, &pos.x, &pos.y, &pos.z, &rot.x, &rot.y, &rot.z, &scale);

		//Gets index of node's spacial pose
		a3i32 j = a3hierarchyGetNodeIndex(h, name);

		if (j < 0)
		{
			//somthing bad happend
			break;
		}
		//Sets the spacial pose data
		//nodeIndex  = a3hierarchyPoseGroupGetNodePoseOffsetIndex(group, counter, nodeIndex);
		//a3i32 offset = a3hierarchyPoseGroupGetNodePoseOffsetIndex(group, 0, j);
		a3spatialPoseSetTranslation(&group->hpose[0].hpose_base[j], pos.x * scalor, pos.y * scalor, pos.z * scalor);
		a3spatialPoseSetRotation(&group->hpose[0].hpose_base[j], rot.x, rot.y, rot.z);
		a3spatialPoseSetScale(&group->hpose[0].hpose_base[j], scale, scale, scale);

		
	}

	//group->hposeCount = counter;

	return true;
}


a3byte ProcessPoses(char* currLine, int currLintLength, FILE* pFile, a3_HierarchyPoseGroup* group, a3_Hierarchy* h, float scalor)
{
	//currently gets all of the animataions and over writes them
	fgets(currLine, currLintLength, pFile);
	int j = 1;
	a3ui32 index = 0;
	a3ui32 extraOffset = 0;
	while (!feof(pFile))
	{
		if (currLine[0] == '#')
		{
			extraOffset += index;
			fgets(currLine, currLintLength, pFile);
			fgets(currLine, currLintLength, pFile);

			if (feof(pFile))
			{
				return 1;
			}
		}

		//get the animation name
		int count = 0;
		//this needs to be move to out side of this loop
		
		char read[a3node_nameSize];

		//scrub the name
		for (int i = 0; i < currLintLength; i++)
		{
			if (currLine[i] == '\r')
			{
				read[count] = '\0';
				break;
			}

			if (currLine[i] != '[' && currLine[i] != ']')
			{
				read[count] = currLine[i];
				count++;
			}
		}

		fgets(currLine, currLintLength, pFile);

		j = a3hierarchyGetNodeIndex(h, read);
		while (currLine[0] != '[' && currLine[0] != '#')
		{
			
			a3vec3 pos;
			a3vec3 rot;
			a3f32 scale;

			sscanf(currLine, "%i %f %f %f %f %f %f %f", &index, &pos.x, &pos.y, &pos.z, &rot.x, &rot.y, &rot.z, &scale);
			
			a3i32 offset = a3hierarchyPoseGroupGetNodePoseOffsetIndex(group, index - 1, j);
			a3i32 fullOffset = offset + extraOffset * group->hierarchy->numNodes;
			a3spatialPoseSetTranslation(group->pose + fullOffset, pos.x * scalor, pos.y * scalor, pos.z * scalor);
			a3spatialPoseSetRotation(group->pose + fullOffset, rot.x, rot.y, rot.z);
			a3spatialPoseSetScale(group->pose + fullOffset, scale, scale, scale);

			//progress line
			fgets(currLine, currLintLength, pFile);
			//testCounter++;
		}
		//the hpose is the currect pose group for a animation and base is a pointer to a animation
		
	}
	
		//get animation name

	return true;
}

//-----------------------------------------------------------------------------

// load HTR file, read and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupLoadHTR(a3_HierarchyPoseGroup* poseGroup_out, a3_Hierarchy* hierarchy_out, const a3byte* resourceFilePath)
{
	if (poseGroup_out && !poseGroup_out->hierarchy && hierarchy_out && !hierarchy_out->nodes && resourceFilePath && *resourceFilePath)
	{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-PROJECT-2: IMPLEMENT ME
//-----------------------------------------------------------------------------
		//TODO  BOTH
		//open file
		FILE* pFile = fopen(resourceFilePath, "rb");

		int lineLength = 4097;
		char currLine[4097];
		float modelScale = 1;
		if (pFile)
		{
			fgets(currLine, lineLength, pFile);
			while (!feof(pFile))
			{
				if (strstr(currLine, "[Header]"))
				{
					if (!ProcessHeaders(currLine, lineLength, pFile, poseGroup_out, hierarchy_out, &modelScale))
					{
						//Could not be loaded
						return -1;
					}
					
				}
				else if (strstr(currLine, "[SegmentNames&Hierarchy]"))
				{
					//Do segment names and hierarchy stuff
					if (!ProcessSegmentsAndHeirarchy(currLine, lineLength, pFile, hierarchy_out))
					{
						//Issue with name and segment loading
						return -1;
					}
				}
				else if (strstr(currLine, "[BasePosition]"))
				{
					//Do base position stuff
					if (!ProcessBasePositions(currLine, lineLength, pFile, poseGroup_out, hierarchy_out, modelScale))
					{
						//Issue with name and segment loading
						return -1;
					}
				}
				else if (currLine[0] == '#')
				{
					if (!ProcessPoses(currLine, lineLength, pFile, poseGroup_out, hierarchy_out, modelScale))
					{
						return -1;
					}
				}			
			}
		}

		fclose(pFile);


//-----------------------------------------------------------------------------
//****END-TO-DO-PROJECT-2
//-----------------------------------------------------------------------------
	}
	return 1;
}

// load BVH file, read and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupLoadBVH(a3_HierarchyPoseGroup* poseGroup_out, a3_Hierarchy* hierarchy_out, const a3byte* resourceFilePath)
{
	if (poseGroup_out && !poseGroup_out->hierarchy && hierarchy_out && !hierarchy_out->nodes && resourceFilePath && *resourceFilePath)
	{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-OPTIONAL: IMPLEMENT ME
//-----------------------------------------------------------------------------

		

//-----------------------------------------------------------------------------
//****END-TO-DO-OPTIONAL
//-----------------------------------------------------------------------------
	}
	return -1;
}

// save HTR file, read and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupSaveHTR(const a3_HierarchyPoseGroup* poseGroup_in, const a3_Hierarchy* hierarchy_in, const a3byte* resourceFilePath)
{
	if (poseGroup_in && poseGroup_in->hierarchy && hierarchy_in && hierarchy_in->nodes && resourceFilePath && *resourceFilePath)
	{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-OPTIONAL: IMPLEMENT ME
//-----------------------------------------------------------------------------

		

//-----------------------------------------------------------------------------
//****END-TO-DO-OPTIONAL
//-----------------------------------------------------------------------------
	}
	return -1;
}

// save BVH file, read and store complete pose group and hierarchy
a3i32 a3hierarchyPoseGroupSaveBVH(const a3_HierarchyPoseGroup* poseGroup_in, const a3_Hierarchy* hierarchy_in, const a3byte* resourceFilePath)
{
	if (poseGroup_in && poseGroup_in->hierarchy && hierarchy_in && hierarchy_in->nodes && resourceFilePath && *resourceFilePath)
	{
//-----------------------------------------------------------------------------
//****TO-DO-ANIM-OPTIONAL: IMPLEMENT ME
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
//****END-TO-DO-OPTIONAL
//-----------------------------------------------------------------------------
	}
	return -1;
}


//-----------------------------------------------------------------------------
