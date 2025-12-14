using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CommonVars;
using System; 
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
namespace TrueTrace {
    // [System.Serializable]
    unsafe public class BVH2Builder {

        public BVHNode2Data* BVH2Nodes;
        public int* DimensionedIndices;
        public int* FinalIndices;
        public bool* indices_going_left;
        public int PrimCount;
        private ObjectSplit split = new ObjectSplit();
        public float* SAH;
        public AABB* Primitives;
        public int* RadixPrefixSum;
        private int* LocalDimensions;
        
        public NativeArray<int> RadixPrefixArray;
        public NativeArray<BVHNode2Data> BVH2NodesArray;
        public NativeArray<int> DimensionedIndicesArray;
        public NativeArray<int> FinalIndicesArray;
        public NativeArray<bool> indices_going_left_array;
        public NativeArray<float> SAHArray;

        public void Dispose() {
            if(BVH2NodesArray.IsCreated) BVH2NodesArray.Dispose();
            if(DimensionedIndicesArray.IsCreated) DimensionedIndicesArray.Dispose();
            if(FinalIndicesArray.IsCreated) FinalIndicesArray.Dispose();
            if(indices_going_left_array.IsCreated) indices_going_left_array.Dispose();
            if(SAHArray.IsCreated) SAHArray.Dispose();
            if(RadixPrefixArray.IsCreated) RadixPrefixArray.Dispose();
        }

        public struct ObjectSplit {
            public int index;
            public float cost;
            public int dimension;
            public AABB aabb_left;
            public AABB aabb_right;
        }

        AABB aabb_right;
        void partition_sah(int first_index, int index_count) {
            split.cost = float.MaxValue;//SA * (float)index_count;
            split.index = -1;
            split.dimension = -1;
            split.aabb_right.init();

            float left_cost;
            int first_right;
            int i;
            for(int dimension = 0; dimension < 3; dimension++) {
                aabb_right.init();
                LocalDimensions = DimensionedIndices + (PrimCount * dimension + first_index);
                first_right = index_count;
                for(i = 1; i < index_count; i++) {
                    aabb_right.Extend(ref Primitives[LocalDimensions[i - 1]]);
                    SAH[i] = surface_area(ref aabb_right) * (float)i;
                    if(SAH[i] >= split.cost) {
                        first_right = i + 1;
                        break;
                    }
                }
                aabb_right.init();
                for(i = index_count - 1; i > 0; i--) {
                    aabb_right.Extend(ref Primitives[LocalDimensions[i]]);
                    if(i > first_right - 1) continue;
                    left_cost = surface_area(ref aabb_right) * (float)(index_count - i);
                    if(left_cost >= split.cost) break;
                    SAH[i] += left_cost;

                    if(SAH[i] < split.cost) {
                        split.cost = SAH[i];
                        split.index = first_index + i;
                        split.dimension = dimension;
                        split.aabb_right = aabb_right;
                    }
                }
            }
            LocalDimensions = DimensionedIndices + (split.dimension * PrimCount);
            int Index2 = split.index;
            aabb_right.init();
            for(i = first_index; i < Index2; i++) aabb_right.Extend(ref Primitives[LocalDimensions[i]]);
            split.aabb_left = aabb_right;
        }
        void BuildRecursive(int nodesi, ref int node_index, int first_index, int index_count) {
            if(index_count == 1) {
                BVH2Nodes[nodesi].left = first_index;
                BVH2Nodes[nodesi].count = (uint)index_count;
                return;
            }
            
            partition_sah(first_index, index_count);
            int IndexEnd = first_index + index_count;
            for(int i = first_index; i < IndexEnd; i++) indices_going_left[LocalDimensions[i]] = i < split.index;

            for(int dim = 0; dim < 3; dim++) {
                if(dim == split.dimension) continue;

                int index;
                int left = 0;
                int right = split.index - first_index;
                LocalDimensions = DimensionedIndices + dim * PrimCount;
                for(int i = first_index; i < IndexEnd; i++) {
                    index = LocalDimensions[i];
                    FinalIndices[indices_going_left[index] ? (left++) : (right++)] = index;
                }
          
                UnsafeUtility.MemCpy(destination: LocalDimensions + first_index, source: FinalIndices, size: index_count * 4);
            }
            BVH2Nodes[nodesi].left = node_index;

            BVH2Nodes[BVH2Nodes[nodesi].left].aabb = split.aabb_left;
            BVH2Nodes[BVH2Nodes[nodesi].left + 1].aabb = split.aabb_right;
            node_index += 2;
            int Index = split.index;
            BuildRecursive(BVH2Nodes[nodesi].left, ref node_index, first_index, Index - first_index);
            BuildRecursive(BVH2Nodes[nodesi].left + 1, ref node_index, Index, first_index + index_count - Index);
        }

        float surface_area(ref AABB aabb) {
            float dx = aabb.BBMax.x - aabb.BBMin.x;
            float dy = aabb.BBMax.y - aabb.BBMin.y;
            float dz = aabb.BBMax.z - aabb.BBMin.z;
            return (dx + dy) * dz + dx * dy; 
        }
        public int BVHNodeCount;
        public unsafe BVH2Builder(AABB* Triangles, int PrimCount) {//Bottom Level Acceleration Structure Builder
            this.PrimCount = PrimCount;
            Primitives = Triangles;
            DimensionedIndicesArray = new NativeArray<int>(PrimCount * 3, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            BVH2NodesArray = new NativeArray<BVHNode2Data>(PrimCount * 2, Unity.Collections.Allocator.Persistent, NativeArrayOptions.ClearMemory);
            BVH2Nodes = (BVHNode2Data*)NativeArrayUnsafeUtility.GetUnsafePtr(BVH2NodesArray);
            DimensionedIndices = (int*)NativeArrayUnsafeUtility.GetUnsafePtr(DimensionedIndicesArray);
            SAHArray = new NativeArray<float>(PrimCount, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            SAH = (float*)NativeArrayUnsafeUtility.GetUnsafePtr(SAHArray);
            FinalIndicesArray = new NativeArray<int>(PrimCount, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            FinalIndices = (int*)NativeArrayUnsafeUtility.GetUnsafePtr(FinalIndicesArray);
            RadixPrefixArray = new NativeArray<int>(6144, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            RadixPrefixSum = (int*)NativeArrayUnsafeUtility.GetUnsafePtr(RadixPrefixArray);
            BVH2Nodes[0].aabb.init();

            for(int i = 0; i < PrimCount; i++) {
                FinalIndices[i] = i;
                SAH[i] = (Triangles[i].BBMax.x + Triangles[i].BBMin.x) * 0.5f;
                BVH2Nodes[0].aabb.Extend(ref Triangles[i]);
            }
            UnsafeUtility.MemClear(RadixPrefixSum, 6144 * 4);
            CommonFunctions.RadixSort(FinalIndices, DimensionedIndices, ref SAH, ref RadixPrefixSum, PrimCount);
            UnsafeUtility.MemClear(RadixPrefixSum, 6144 * 4);
            for(int i = 0; i < PrimCount; i++) {FinalIndices[i] = i; SAH[i] = (Triangles[i].BBMax.y + Triangles[i].BBMin.y) * 0.5f;}
            CommonFunctions.RadixSort(FinalIndices, DimensionedIndices + PrimCount, ref SAH, ref RadixPrefixSum, PrimCount);
            UnsafeUtility.MemClear(RadixPrefixSum, 6144 * 4);
            for(int i = 0; i < PrimCount; i++) {FinalIndices[i] = i; SAH[i] = (Triangles[i].BBMax.z + Triangles[i].BBMin.z) * 0.5f;}
            CommonFunctions.RadixSort(FinalIndices, DimensionedIndices + (PrimCount * 2), ref SAH, ref RadixPrefixSum, PrimCount);
            RadixPrefixArray.Dispose();

            indices_going_left_array = new NativeArray<bool>(PrimCount, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            indices_going_left = (bool*)NativeArrayUnsafeUtility.GetUnsafePtr(indices_going_left_array);
            aabb_right = new AABB();
            int nodeIndex = 2;
            BuildRecursive(0, ref nodeIndex,0,PrimCount);

            indices_going_left_array.Dispose();
            SAHArray.Dispose();
            BVHNodeCount = nodeIndex;
            NativeArray<int>.Copy(DimensionedIndicesArray, 0, FinalIndicesArray, 0, PrimCount);
            DimensionedIndicesArray.Dispose();
        }
        NativeArray<AABB> PrimAABBs;
        public unsafe BVH2Builder(AABB[] MeshAABBs) {//Top Level Acceleration Structure
            PrimCount = MeshAABBs.Length;
            DimensionedIndicesArray = new NativeArray<int>(PrimCount * 3, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            SAHArray = new NativeArray<float>(PrimCount, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            SAH = (float*)NativeArrayUnsafeUtility.GetUnsafePtr(SAHArray);
            BVH2NodesArray = new NativeArray<BVHNode2Data>(PrimCount * 2, Unity.Collections.Allocator.Persistent, NativeArrayOptions.ClearMemory);
            BVH2Nodes = (BVHNode2Data*)NativeArrayUnsafeUtility.GetUnsafePtr(BVH2NodesArray);
            DimensionedIndices = (int*)NativeArrayUnsafeUtility.GetUnsafePtr(DimensionedIndicesArray);
            FinalIndicesArray = new NativeArray<int>(PrimCount, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            FinalIndices = (int*)NativeArrayUnsafeUtility.GetUnsafePtr(FinalIndicesArray);
            RadixPrefixArray = new NativeArray<int>(6144, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            RadixPrefixSum = (int*)NativeArrayUnsafeUtility.GetUnsafePtr(RadixPrefixArray);
            BVH2Nodes[0].aabb.init();

            for(int i = 0; i < PrimCount; i++) {
                FinalIndices[i] = i;
                SAH[i] = (MeshAABBs[i].BBMax.x + MeshAABBs[i].BBMin.x) * 0.5f;
                BVH2Nodes[0].aabb.Extend(ref MeshAABBs[i]);
            }
            UnsafeUtility.MemClear(RadixPrefixSum, 6144 * 4);
            CommonFunctions.RadixSort(FinalIndices, DimensionedIndices, ref SAH, ref RadixPrefixSum, PrimCount);
            UnsafeUtility.MemClear(RadixPrefixSum, 6144 * 4);
            for(int i = 0; i < PrimCount; i++) {FinalIndices[i] = i; SAH[i] = (MeshAABBs[i].BBMax.y + MeshAABBs[i].BBMin.y) * 0.5f;}
            CommonFunctions.RadixSort(FinalIndices, DimensionedIndices + PrimCount, ref SAH, ref RadixPrefixSum, PrimCount);
            UnsafeUtility.MemClear(RadixPrefixSum, 6144 * 4);
            for(int i = 0; i < PrimCount; i++) {FinalIndices[i] = i; SAH[i] = (MeshAABBs[i].BBMax.z + MeshAABBs[i].BBMin.z) * 0.5f;}
            CommonFunctions.RadixSort(FinalIndices, DimensionedIndices + (PrimCount * 2), ref SAH, ref RadixPrefixSum, PrimCount);

            indices_going_left_array = new NativeArray<bool>(PrimCount, Unity.Collections.Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            indices_going_left = (bool*)NativeArrayUnsafeUtility.GetUnsafePtr(indices_going_left_array);
            PrimAABBs = new NativeArray<AABB>(MeshAABBs, Unity.Collections.Allocator.Persistent);
            Primitives = (AABB*)NativeArrayUnsafeUtility.GetUnsafePtr(PrimAABBs);
            aabb_right = new AABB();
            int nodeIndex = 2;
            BuildRecursive(0, ref nodeIndex,0,PrimCount);
            BVHNodeCount = nodeIndex;
            NativeArray<int>.Copy(DimensionedIndicesArray, 0, FinalIndicesArray, 0, PrimCount);

        }

        public void ClearAll() {
            SAHArray.Dispose();
            indices_going_left_array.Dispose();
            DimensionedIndicesArray.Dispose();
            PrimAABBs.Dispose();
            BVH2NodesArray.Dispose();
            RadixPrefixArray.Dispose();
            FinalIndicesArray.Dispose();
        }

        public unsafe void NoAllocRebuild(AABB[] MeshAABBs) {//Top Level Acceleration Structure
            PrimCount = MeshAABBs.Length;
            SAH = (float*)NativeArrayUnsafeUtility.GetUnsafePtr(SAHArray);
            BVH2Nodes = (BVHNode2Data*)NativeArrayUnsafeUtility.GetUnsafePtr(BVH2NodesArray);
            DimensionedIndices = (int*)NativeArrayUnsafeUtility.GetUnsafePtr(DimensionedIndicesArray);
            FinalIndices = (int*)NativeArrayUnsafeUtility.GetUnsafePtr(FinalIndicesArray);
            RadixPrefixSum = (int*)NativeArrayUnsafeUtility.GetUnsafePtr(RadixPrefixArray);
            BVH2Nodes[0].aabb.init();
            for(int i = 0; i < PrimCount; i++) {//Treat Bottom Level BVH Root Nodes as triangles
                FinalIndices[i] = i;
                SAH[i] = (MeshAABBs[i].BBMax.x + MeshAABBs[i].BBMin.x) * 0.5f;
                BVH2Nodes[i] = new BVHNode2Data();
                BVH2Nodes[i].aabb.init();
                BVH2Nodes[i + PrimCount] = new BVHNode2Data();
                BVH2Nodes[0].aabb.Extend(ref MeshAABBs[i]);
            }

            UnsafeUtility.MemClear(RadixPrefixSum, 6144 * 4);
            CommonFunctions.RadixSort(FinalIndices, DimensionedIndices, ref SAH, ref RadixPrefixSum, PrimCount);
            UnsafeUtility.MemClear(RadixPrefixSum, 6144 * 4);
            for(int i = 0; i < PrimCount; i++) {FinalIndices[i] = i; SAH[i] = (MeshAABBs[i].BBMax.y + MeshAABBs[i].BBMin.y) * 0.5f;}
            CommonFunctions.RadixSort(FinalIndices, DimensionedIndices + PrimCount, ref SAH, ref RadixPrefixSum, PrimCount);
            UnsafeUtility.MemClear(RadixPrefixSum, 6144 * 4);
            for(int i = 0; i < PrimCount; i++) {FinalIndices[i] = i; SAH[i] = (MeshAABBs[i].BBMax.z + MeshAABBs[i].BBMin.z) * 0.5f;}
            CommonFunctions.RadixSort(FinalIndices, DimensionedIndices + (PrimCount * 2), ref SAH, ref RadixPrefixSum, PrimCount);

            indices_going_left = (bool*)NativeArrayUnsafeUtility.GetUnsafePtr(indices_going_left_array);
            NativeArray<AABB>.Copy(MeshAABBs, 0, PrimAABBs, 0, PrimCount);
            Primitives = (AABB*)NativeArrayUnsafeUtility.GetUnsafePtr(PrimAABBs);
            aabb_right = new AABB();
            int nodeIndex = 2;
            BuildRecursive(0, ref nodeIndex,0,PrimCount);

            BVHNodeCount = nodeIndex;
            NativeArray<int>.Copy(DimensionedIndicesArray, 0, FinalIndicesArray, 0, PrimCount);

        }

    }
}