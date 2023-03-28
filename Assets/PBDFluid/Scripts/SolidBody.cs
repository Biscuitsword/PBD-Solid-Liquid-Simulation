using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace PBDFluid
{

    public class SolidBody : IDisposable
    {
        private const int THREADS = 128;
        public int NumParticles { get; private set; }

        public Bounds Bounds;

        public float Density { get; set; }

        public float Viscosity { get; set; }

        public float Dampning { get; set; }

        public float ParticleRadius { get; private set; }

        public float ParticleDiameter { get { return ParticleRadius * 2.0f; } }

        public float ParticleMass { get; set; }

        public float ParticleVolume { get; private set; }

        public ComputeBuffer Pressures { get; private set; }

        public ComputeBuffer Densities { get; private set; }

        public ComputeBuffer Positions { get; private set; }

        public ComputeBuffer[] Predicted { get; private set; }

        public ComputeBuffer[] Velocities { get; private set; }

        private ComputeBuffer m_argsBuffer;

        public ComputeBuffer Core { get; set; }

        public SolidBody(ParticleSource source, float radius, float density, Matrix4x4 RTS)
        {
            NumParticles = source.NumParticles;
            Density = density;
            Viscosity = 0.002f;
            Dampning = 0.0f;

            ParticleRadius = radius;
            ParticleVolume = (4.0f / 3.0f) * Mathf.PI * Mathf.Pow(radius, 3);
            ParticleMass = ParticleVolume * Density;

            Densities = new ComputeBuffer(NumParticles, sizeof(float));
            Pressures = new ComputeBuffer(NumParticles, sizeof(float));
            Core = new ComputeBuffer(NumParticles, sizeof(float));

            CreateParticles(source, RTS);
            CreateBoundryPsi();
        }

        public void UpdateCore()
        {
            //Matrix4x4 A = new Matrix4x4();
            //Matrix A = new Matrix(4, 4);

            //Matrix A_1 = new Matrix(4, 4);
            //Matrix A_2 = new Matrix(4, 4);
            //for (int i =0; i < NumParticles; i++)
            //{
            //    Vector4 ri = tmp[i] - new Vector4(Core.x, Core.y, Core.z, 0);
            //    Matrix mt_1 = new Matrix(4, 4);
            //    Matrix mt_2 = new Matrix(4, 4);
            //    mt_1.SetRow(0, ri);
            //    mt_2.SetColumn(0, ri);

            //    A_1 = MatrixOperator.MatrixAdd(MatrixOperator.MatrixMulti(mt_1, mt_2), A_1);
            //}

            //for (int i = 0; i < NumParticles; i++)
            //{
            //    Vector4 ri = tmp[i] - new Vector4(Core.x, Core.y, Core.z, 0);
            //    Matrix mt_1 = new Matrix(4, 4);
            //    Matrix mt_2 = new Matrix(4, 4);
            //    mt_1.SetRow(0, ri);
            //    mt_2.SetColumn(0, ri);

            //    A_2 = MatrixOperator.MatrixAdd(MatrixOperator.MatrixMulti(mt_1, mt_2), A_2);
            //}

            //A = MatrixOperator.MatrixMulti(A_1, MatrixOperator.MatrixInvByCom(A_2));

        }

        public void UpdateBounds(Matrix4x4 RTS)
        {
            float inf = float.PositiveInfinity;
            Vector3 min = new Vector3(inf, inf, inf);
            Vector3 max = new Vector3(-inf, -inf, -inf);

            Vector4[] tmp = new Vector4[NumParticles];
            Positions.GetData(tmp);
            // 从Source中逐个获取位置信息(过RTS转换)
            for (int i = 0; i < NumParticles; i++)
            {                
                Vector4 pos = tmp[i];

                // 维护极值
                if (pos.x < min.x) min.x = pos.x;
                if (pos.y < min.y) min.y = pos.y;
                if (pos.z < min.z) min.z = pos.z;

                if (pos.x > max.x) max.x = pos.x;
                if (pos.y > max.y) max.y = pos.y;
                if (pos.z > max.z) max.z = pos.z;
            }

            // 极值扩张出粒子半径
            min.x -= ParticleRadius;
            min.y -= ParticleRadius;
            min.z -= ParticleRadius;

            max.x += ParticleRadius;
            max.y += ParticleRadius;
            max.z += ParticleRadius;

            // 依照极值初始化包围框
            Bounds = new Bounds();
            Bounds.SetMinMax(min, max);
        }

        /// <summary>
        /// Draws the mesh spheres when draw particles is enabled.
        /// </summary>
        public void Draw(Camera cam, Mesh mesh, Material material, int layer)
        {
            if (m_argsBuffer == null)
                CreateArgBuffer(mesh.GetIndexCount(0));

            material.SetBuffer("positions", Positions);
            material.SetColor("color", Color.white);
            material.SetFloat("diameter", ParticleDiameter);

            ShadowCastingMode castShadow = ShadowCastingMode.On;
            bool recieveShadow = true;

            Graphics.DrawMeshInstancedIndirect(mesh, 0, material, Bounds, m_argsBuffer, 0, null, castShadow, recieveShadow, layer, cam);
        }

        public void Dispose()
        {

            if (Positions != null)
            {
                Positions.Release();
                Positions = null;
            }

            if (Densities != null)
            {
                Densities.Release();
                Densities = null;
            }

            if (Pressures != null)
            {
                Pressures.Release();
                Pressures = null;
            }

            if (Core != null)
            {
                Core.Release();
                Core = null;
            }

            CBUtility.Release(Predicted);
            CBUtility.Release(Velocities);
            CBUtility.Release(ref m_argsBuffer);
        }

        private void CreateParticles(ParticleSource source, Matrix4x4 RTS)
        {
            // 初始化数组备用， 每个粒子准备对应参数
            Vector4[] positions = new Vector4[NumParticles];
            Vector4[] predicted = new Vector4[NumParticles];
            Vector4[] velocities = new Vector4[NumParticles];

            // 初始化最大最小值
            float inf = float.PositiveInfinity;
            Vector3 min = new Vector3(inf, inf, inf);
            Vector3 max = new Vector3(-inf, -inf, -inf);

            // 从Source中逐个获取位置信息(过RTS转换)
            for (int i = 0; i < NumParticles; i++)
            {
                // Soure 事实上只负责点位生成，信息会持久化到compute buff中
                Vector4 pos = RTS * source.Positions[i];
                positions[i] = pos;
                predicted[i] = pos;

                // 维护极值
                if (pos.x < min.x) min.x = pos.x;
                if (pos.y < min.y) min.y = pos.y;
                if (pos.z < min.z) min.z = pos.z;

                if (pos.x > max.x) max.x = pos.x;
                if (pos.y > max.y) max.y = pos.y;
                if (pos.z > max.z) max.z = pos.z;
            }

            // 极值扩张出粒子半径
            min.x -= ParticleRadius;
            min.y -= ParticleRadius;
            min.z -= ParticleRadius;

            max.x += ParticleRadius;
            max.y += ParticleRadius;
            max.z += ParticleRadius;

            // 依照极值初始化包围框
            Bounds = new Bounds();
            Bounds.SetMinMax(min, max);

            // ComputeBuffer初始化并赋值(经过初始化的数组)
            Positions = new ComputeBuffer(NumParticles, 4 * sizeof(float));
            Positions.SetData(positions);

            //Predicted and velocities use a double buffer as solver step
            //needs to read from many locations of buffer and write the result
            //in same pass. Could be removed if needed as long as buffer writes 
            //are atomic. Not sure if they are.

            Predicted = new ComputeBuffer[2];
            Predicted[0] = new ComputeBuffer(NumParticles, 4 * sizeof(float));
            Predicted[0].SetData(predicted);
            Predicted[1] = new ComputeBuffer(NumParticles, 4 * sizeof(float));
            Predicted[1].SetData(predicted);

            Velocities = new ComputeBuffer[2];
            Velocities[0] = new ComputeBuffer(NumParticles, 4 * sizeof(float));
            Velocities[0].SetData(velocities);
            Velocities[1] = new ComputeBuffer(NumParticles, 4 * sizeof(float));
            Velocities[1].SetData(velocities);
        }

        private void CreateArgBuffer(uint indexCount)
        {
            uint[] args = new uint[5] { 0, 0, 0, 0, 0 };
            args[0] = indexCount;
            args[1] = (uint)NumParticles;

            m_argsBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);
            m_argsBuffer.SetData(args);
        }

        private void CreateBoundryPsi()
        {

            float cellSize = ParticleRadius * 4.0f;
            SmoothingKernel K = new SmoothingKernel(cellSize);

            ComputeShader shader = Resources.Load("FluidBoundary") as ComputeShader;

            int kernel = shader.FindKernel("ComputePsi");

            shader.SetFloat("Density", Density);
            shader.SetFloat("KernelRadiuse", K.Radius);
            shader.SetFloat("KernelRadius2", K.Radius2);
            shader.SetFloat("Poly6", K.POLY6);
            shader.SetFloat("Poly6Zero", K.Poly6(Vector3.zero));
            shader.SetInt("NumParticles", NumParticles);
            

            shader.SetBuffer(kernel, "Boundary", Positions);

            int groups = NumParticles / THREADS;
            if (NumParticles % THREADS != 0) groups++;

            //Fills the boundarys psi array so the fluid can
            //collide against it smoothly. The original computes
            //the phi for each boundary particle based on the
            //density of the boundary but I find the fluid 
            //leaks out so Im just using a const value.

            shader.Dispatch(kernel, groups, 1, 1);
            

        }

    }


}