using System;
using System.Collections.Generic;
using UnityEngine;

namespace PBDFluid
{

    public class SolidSolver : IDisposable
    {

        private const int THREADS = 128;
        private const int READ = 0;
        private const int WRITE = 1;

        public int Groups { get; private set; }

        // 容器信息
        public FluidBoundary Boundary { get; private set; }

        public SolidBody Body { get; private set; }

        public FluidBody fluid_Body { get; private set; }

        public GridHash Hash { get; private set; }


        public int SolverIterations { get; set; }

        public int ConstraintIterations { get; set; }

        public SmoothingKernel Kernel { get; private set; }

        private ComputeShader m_shader;

        public SolidSolver(SolidBody body, FluidBoundary boundary, FluidBody fluid)
        {
            // 初始化时赋予body和bound
            SolverIterations = 2;
            ConstraintIterations = 2;

            fluid_Body = fluid; // 沿用boundary的方式加入流体因素
            Body = body;
            // 最外包围盒（容器包围盒）
            Boundary = boundary;

            float cellSize = Body.ParticleRadius * 4.0f;
            int total = Body.NumParticles + Boundary.NumParticles;
            Hash = new GridHash(Boundary.Bounds, total, cellSize, fluid_Body.NumParticles);
            Kernel = new SmoothingKernel(cellSize);

            // 根据总粒子数和线程数分派计算组Groups
            int numParticles = Body.NumParticles;
            Groups = numParticles / THREADS;
            if (numParticles % THREADS != 0) Groups++;

            // 装载ComputeShader
            // 对数组进行分别计算就是对所有粒子进行计算，由ID决定
            m_shader = Resources.Load("SolidSolver") as ComputeShader;

        }

        public void Dispose()
        {
            Hash.Dispose();
        }

        public void MoveTowards(float dx, float dy, float dz)
        {
            if (dx == 0 && dy == 0 && dz == 0) return;
            Debug.Log("Moving");

            int kernel = m_shader.FindKernel("MoveTowardsKey");

            m_shader.SetInt("NumParticles", Body.NumParticles);
            m_shader.SetVector("Gravity", new Vector3(0.0f, -9.81f, 0.0f));
            m_shader.SetFloat("Dampning", Body.Dampning);
            m_shader.SetFloat("Density", Body.Density);
            m_shader.SetFloat("Viscosity", Body.Viscosity);
            m_shader.SetFloat("ParticleMass", Body.ParticleMass);
            m_shader.SetFloat("ParticleRadius", Body.ParticleRadius);

            m_shader.SetFloat("KernelRadius", Kernel.Radius);
            m_shader.SetFloat("KernelRadius2", Kernel.Radius2);
            m_shader.SetFloat("Poly6Zero", Kernel.Poly6(Vector3.zero));
            m_shader.SetFloat("Poly6", Kernel.POLY6);
            m_shader.SetFloat("SpikyGrad", Kernel.SPIKY_GRAD);
            m_shader.SetFloat("ViscLap", Kernel.VISC_LAP);

            m_shader.SetFloat("HashScale", Hash.InvCellSize);
            m_shader.SetVector("HashSize", Hash.Bounds.size);
            m_shader.SetVector("HashTranslate", Hash.Bounds.min);

            m_shader.SetVector("dxyz", new Vector3(dx, dy, dz));
            m_shader.SetBuffer(kernel, "Positions", Body.Positions);

            m_shader.Dispatch(kernel, Groups, 1, 1);
        }


        // 按照step推进物理模拟（上层Update中调用）
        public void StepPhysics(float dt)
        {

            if (dt <= 0.0) return;
            if (SolverIterations <= 0 || ConstraintIterations <= 0) return;

            dt /= SolverIterations;

            m_shader.SetInt("NumParticles", Body.NumParticles);
            m_shader.SetInt("BoundaryParticles", Boundary.NumParticles);
            m_shader.SetInt("ExtraParticles", fluid_Body.NumParticles);
            m_shader.SetVector("Gravity", new Vector3(0.0f, -9.81f, 0.0f));
            m_shader.SetFloat("Dampning", Body.Dampning);
            m_shader.SetFloat("DeltaTime", dt);
            m_shader.SetFloat("Density", Body.Density);
            m_shader.SetFloat("Viscosity", Body.Viscosity);
            m_shader.SetFloat("ParticleMass", Body.ParticleMass);
            m_shader.SetFloat("ParticleRadius", Body.ParticleRadius);

            m_shader.SetFloat("KernelRadius", Kernel.Radius);
            m_shader.SetFloat("KernelRadius2", Kernel.Radius2);
            m_shader.SetFloat("Poly6Zero", Kernel.Poly6(Vector3.zero));
            m_shader.SetFloat("Poly6", Kernel.POLY6);
            m_shader.SetFloat("SpikyGrad", Kernel.SPIKY_GRAD);
            m_shader.SetFloat("ViscLap", Kernel.VISC_LAP);

            m_shader.SetFloat("HashScale", Hash.InvCellSize);
            m_shader.SetVector("HashSize", Hash.Bounds.size);
            m_shader.SetVector("HashTranslate", Hash.Bounds.min);

            //Predicted and velocities use a double buffer as solver step
            //needs to read from many locations of buffer and write the result
            //in same pass. Could be removed if needed as long as buffer writes 
            //are atomic. Not sure if they are.

            for (int i = 0; i < SolverIterations; i++)
            {
                //PredictPositions(dt);

                //Hash.Process(Body.Predicted[READ], Boundary.Positions);

                //ConstrainPositions();

                //UpdateVelocities(dt);

                //SolveViscosity();

                //UpdatePositions();

                // Solid

                EstimatePositions(dt);

                Hash.Process(Body.Predicted[READ], Boundary.Positions, fluid_Body.Positions);

                UpdateVelocities(dt);

                ResolveCollisions();

                ShapeMatching(dt);

                UpdatePositions();

                //UpdateBounds();

            }

        }


        private void ApplyExternalForces(float dt)
        {
            // 根据外力修改速度(未应用，与compute shader中的重力和阻尼计算重复)
            // TODO: 实现shader计算过程，传入其他物体
            int kernel = m_shader.FindKernel("ApplyExternalForces");

            // Damping已经经过初始化，不用重复传入
            m_shader.SetBuffer(kernel, "Positions", Body.Positions);
            m_shader.SetBuffer(kernel, "VelocitiesREAD", Body.Velocities[READ]);
            m_shader.SetBuffer(kernel, "VelocitiesWRITE", Body.Velocities[WRITE]);

            m_shader.Dispatch(kernel, Groups, 1, 1);
            Swap(Body.Velocities);
        }

        private void EstimatePositions(float dt)
        {
            int kernel = m_shader.FindKernel("EstimatePositions");

            m_shader.SetBuffer(kernel, "Positions", Body.Positions);
            m_shader.SetBuffer(kernel, "PredictedWRITE", Body.Predicted[WRITE]);
            m_shader.SetBuffer(kernel, "VelocitiesREAD", Body.Velocities[READ]);
            m_shader.SetBuffer(kernel, "VelocitiesWRITE", Body.Velocities[WRITE]);

            // Group值将指定id范围（即全部粒子）
            m_shader.Dispatch(kernel, Groups, 1, 1);

            // 交换双层读写buff
            Swap(Body.Predicted);
            Swap(Body.Velocities);
        }

        private void UpdateBounds()
        {
            Body.UpdateBounds(Matrix4x4.identity);
        }

        private void ResolveCollisions()
        {
            int kernel = m_shader.FindKernel("ResolveCollisions");

            m_shader.SetBuffer(kernel, "Pressures", Body.Pressures);
            m_shader.SetBuffer(kernel, "Boundary", Boundary.Positions);
            m_shader.SetBuffer(kernel, "IndexMap", Hash.IndexMap);
            m_shader.SetBuffer(kernel, "Table", Hash.Table);

            m_shader.SetBuffer(kernel, "Extra", fluid_Body.Positions);

            //int computeKernel = m_shader.FindKernel("ComputeDensity");
            //m_shader.SetBuffer(computeKernel, "Densities", Body.Densities);
            //m_shader.SetBuffer(computeKernel, "Pressures", Body.Pressures);
            //m_shader.SetBuffer(computeKernel, "Boundary", Boundary.Positions);
            //m_shader.SetBuffer(computeKernel, "IndexMap", Hash.IndexMap);
            //m_shader.SetBuffer(computeKernel, "Table", Hash.Table);

            //for (int i = 0; i < ConstraintIterations; i++)
            {
                //m_shader.SetBuffer(computeKernel, "PredictedREAD", Body.Predicted[READ]);
                //m_shader.Dispatch(computeKernel, Groups, 1, 1);

                m_shader.SetBuffer(kernel, "PredictedREAD", Body.Predicted[READ]);
                m_shader.SetBuffer(kernel, "PredictedWRITE", Body.Predicted[WRITE]);
                m_shader.SetBuffer(kernel, "VelocitiesREAD", Body.Velocities[READ]);
                m_shader.SetBuffer(kernel, "VelocitiesWRITE", Body.Velocities[WRITE]);
                m_shader.Dispatch(kernel, Groups, 1, 1);

                Swap(Body.Predicted);
                Swap(Body.Velocities);
            }
        }


        private void PredictPositions(float dt)
        {
            // dt 在之前已经输入
            int kernel = m_shader.FindKernel("PredictPositions");

            m_shader.SetBuffer(kernel, "Positions", Body.Positions);
            m_shader.SetBuffer(kernel, "PredictedWRITE", Body.Predicted[WRITE]);
            m_shader.SetBuffer(kernel, "VelocitiesREAD", Body.Velocities[READ]);
            m_shader.SetBuffer(kernel, "VelocitiesWRITE", Body.Velocities[WRITE]);

            // Group值将指定id范围（即全部粒子）
            m_shader.Dispatch(kernel, Groups, 1, 1);

            Swap(Body.Predicted);
            Swap(Body.Velocities);
        }

        public void ConstrainPositions()
        {

            int computeKernel = m_shader.FindKernel("ComputeDensity");
            int solveKernel = m_shader.FindKernel("SolveConstraint");
            // TODO: 去除液体的密度压力约束，增加碰撞约束

            m_shader.SetBuffer(computeKernel, "Densities", Body.Densities);
            m_shader.SetBuffer(computeKernel, "Pressures", Body.Pressures);
            m_shader.SetBuffer(computeKernel, "Boundary", Boundary.Positions);
            m_shader.SetBuffer(computeKernel, "IndexMap", Hash.IndexMap);
            m_shader.SetBuffer(computeKernel, "Table", Hash.Table);

            m_shader.SetBuffer(solveKernel, "Pressures", Body.Pressures);
            m_shader.SetBuffer(solveKernel, "Boundary", Boundary.Positions);
            m_shader.SetBuffer(solveKernel, "IndexMap", Hash.IndexMap);
            m_shader.SetBuffer(solveKernel, "Table", Hash.Table);

            for (int i = 0; i < ConstraintIterations; i++)
            {
                m_shader.SetBuffer(computeKernel, "PredictedREAD", Body.Predicted[READ]);
                m_shader.Dispatch(computeKernel, Groups, 1, 1);


                //float[] tmp = new float[Body.NumParticles];
                //Body.Densities.GetData(tmp);
                //string str = "";
                //for(int a=0; a < 10; a++)
                //{
                //    str += tmp[a].ToString() + ", ";
                //}Debug.Log("Densities: " + str);
                
                //Body.Pressures.GetData(tmp);
                //str = "";
                //for (int a = 0; a < 10; a++)
                //{
                //    str += tmp[a].ToString() + ", ";
                //}
                //Debug.Log("Pressures: " + str);

                m_shader.SetBuffer(solveKernel, "PredictedREAD", Body.Predicted[READ]);
                m_shader.SetBuffer(solveKernel, "PredictedWRITE", Body.Predicted[WRITE]);
                m_shader.Dispatch(solveKernel, Groups, 1, 1);

                Swap(Body.Predicted);
            }
        }

        private void ShapeMatching(float dt)
        {
            // Update Core
            //Body.UpdateCore();
            //int kernel = m_shader.FindKernel("ComputeCore");

            //m_shader.SetBuffer(kernel, "Core", Body.Core);
            //m_shader.SetBuffer(kernel, "PredictedREAD", Body.Predicted[READ]);
            //m_shader.Dispatch(kernel, Groups, 1, 1);

            //float[] tmp = new float[Body.NumParticles];
            //Body.Core.GetData(tmp);

            //Debug.Log(tmp[0] +" " + tmp[1] + " " + tmp[2] + " " + tmp[3] + " " + tmp[4]);


            // Update R
            int shapKernel = m_shader.FindKernel("ShapeMatching");

            m_shader.SetBuffer(shapKernel, "Positions", Body.Positions);
            m_shader.SetBuffer(shapKernel, "PredictedREAD", Body.Predicted[READ]);
            m_shader.SetBuffer(shapKernel, "PredictedWRITE", Body.Predicted[WRITE]);
            m_shader.Dispatch(shapKernel, Groups, 1, 1);

            Swap(Body.Predicted);
        }


        private void UpdateVelocities(float dt)
        {
            int kernel = m_shader.FindKernel("UpdateVelocities");

            m_shader.SetBuffer(kernel, "Positions", Body.Positions);
            m_shader.SetBuffer(kernel, "PredictedREAD", Body.Predicted[READ]);
            m_shader.SetBuffer(kernel, "VelocitiesWRITE", Body.Velocities[WRITE]);

            m_shader.Dispatch(kernel, Groups, 1, 1);

            Swap(Body.Velocities);
        }

        private void SolveViscosity()
        {
            int kernel = m_shader.FindKernel("SolveViscosity");

            m_shader.SetBuffer(kernel, "Densities", Body.Densities);
            m_shader.SetBuffer(kernel, "Boundary", Boundary.Positions);
            m_shader.SetBuffer(kernel, "IndexMap", Hash.IndexMap);
            m_shader.SetBuffer(kernel, "Table", Hash.Table);

            m_shader.SetBuffer(kernel, "PredictedREAD", Body.Predicted[READ]);
            m_shader.SetBuffer(kernel, "VelocitiesREAD", Body.Velocities[READ]);
            m_shader.SetBuffer(kernel, "VelocitiesWRITE", Body.Velocities[WRITE]);

            m_shader.Dispatch(kernel, Groups, 1, 1);

            Swap(Body.Velocities);
        }

        private void UpdatePositions()
        {
            int kernel = m_shader.FindKernel("UpdatePositions");

            m_shader.SetBuffer(kernel, "Positions", Body.Positions);
            m_shader.SetBuffer(kernel, "PredictedREAD", Body.Predicted[READ]);

            m_shader.Dispatch(kernel, Groups, 1, 1);
        }

        private void Swap(ComputeBuffer[] buffers)
        {
            ComputeBuffer tmp = buffers[0];
            buffers[0] = buffers[1];
            buffers[1] = tmp;
        }
    }

}