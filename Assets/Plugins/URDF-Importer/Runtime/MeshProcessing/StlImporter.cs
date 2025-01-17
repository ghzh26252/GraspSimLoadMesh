﻿/*
© Siemens AG, 2017-18
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;
using System.Collections.Generic;
using System.IO;

namespace Unity.Robotics.UrdfImporter
{
    public static class StlImporter
    {
        public static Mesh ImportMesh(string path)
        {
            IList<StlReader.Facet> facets;
            if (IsBinary(path))
                facets = StlReader.ReadBinaryFile(path);
            else
                facets = StlReader.ReadAsciiFile(path);
            return CreateMesh(facets);
        }

        private static bool IsBinary(string path)
        {
            int maxCharsToCheck = 100;

            using (StreamReader reader = new StreamReader(path))
                for (int i = 0; i < maxCharsToCheck; i++)
                    if (reader.Read() == '\0')
                        return true;

            return false;
        }

        private static Mesh CreateMesh(IList<StlReader.Facet> facets)
        {
            int totalFacetIndex = 0;
            int[] order = new int[] { 0, 2, 1 };

            Vector3[] vertices;
            Vector3[] normals;
            int[] triangles;

            int verticeSize = facets.Count * 3;

            vertices = new Vector3[verticeSize];
            normals = new Vector3[verticeSize];
            triangles = new int[verticeSize];
            for (int facetIndex = 0; facetIndex < verticeSize; facetIndex += 3)
            {
                for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
                {
                    vertices[facetIndex + vertexIndex] = facets[totalFacetIndex].vertices[order[vertexIndex]];
                    normals[facetIndex + vertexIndex] = facets[totalFacetIndex].normal;
                    triangles[facetIndex + vertexIndex] = facetIndex + vertexIndex;
                }
                totalFacetIndex++;
            }

            Mesh mesh = new Mesh();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
            mesh.name = "default";
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.normals = normals;
            return mesh;
        }
    }
}