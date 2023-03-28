using UnityEngine;
using System;
using System.Collections.Generic;
using UnityEngine.SceneManagement;

namespace PBDFluid
{

    public enum SIMULATION_SIZE {  LOW, MEDIUM, HIGH }

    public class FluidBodyDemo : MonoBehaviour
    {

        private const float timeStep = 1.0f / 60.0f;

        public Material m_fluidParticleMat;

        public Material m_boundaryParticleMat;

        public Material m_volumeMat;

        public bool m_drawLines = true;

        public bool m_drawGrid = false;

        public bool m_drawBoundaryParticles = false;

        public bool m_drawFluidParticles = false;

        public bool m_drawFluidVolume = true;

        public SIMULATION_SIZE m_simulationSize = SIMULATION_SIZE.MEDIUM;

        public bool m_run = true;

        public bool solid_run = false;

        public bool m_control = true;

        public Mesh m_sphereMesh;

        private FluidBody m_fluid;

        private FluidBoundary m_boundary;

        private FluidSolver m_solver;

        private RenderVolume m_volume;

        Bounds m_fluidSource, m_outerSource, m_innerSource, m_solidSource;

        private float dx, dy, dz;


        /// <summary>
        /// Solid PBD
        /// </summary>
        private SolidBody m_solid;

        private SolidSolver m_solidsolver;

        private bool wasError;

        private void Start()
        {
            float radius = 0.08f;
            float density = 1000.0f;

            dx = 0;
            dy = 0;
            dz = 0;

            //A smaller radius means more particles.
            //If the number of particles is to low or high
            //the bitonic sort shader will throw a exception 
            //as it has a set range it can handle but this 
            //can be manually changes in the BitonicSort.cs script.

            //A smaller radius may also requre more solver steps
            //as the boundary is thinner and the fluid many step
            //through and leak out.

            switch(m_simulationSize)
            {
                case SIMULATION_SIZE.LOW:
                    radius = 0.1f;
                    break;

                case SIMULATION_SIZE.MEDIUM:
                    radius = 0.08f;
                    break;

                case SIMULATION_SIZE.HIGH:
                    radius = 0.06f;
                    break;
            }

            try
            {
                // 创建最外包围盒
                CreateBoundary(radius, density);
                CreateFluid(radius, density);   // 初始化FluidBody 基本数据
                CreateSolid(radius, density);

                // 传递最外包围盒数据
                m_fluid.Bounds = m_boundary.Bounds; // FluidBody

                m_solver = new FluidSolver(m_fluid, m_boundary, m_solid);    //基于FluidBody创建求解器

                m_volume = new RenderVolume (m_boundary.Bounds, radius); //基于最外边界信息和粒子半径创建渲染器
                m_volume.CreateMesh(m_volumeMat);

                // Demo: 固体计算
                m_solid.Bounds = m_boundary.Bounds; // FluidBody
                m_solidsolver = new SolidSolver(m_solid, m_boundary, m_fluid);    //基于FluidBody创建求解器，传入容器包围盒信息

                //m_volume = new RenderVolume(m_boundary.Bounds, radius); //基于最外边界信息和粒子半径创建渲染器
                //m_volume.CreateMesh(m_volumeMat);
            }
            catch
            {
                wasError = true;
                throw;
            }
        }

        private void Update()
        {
            if(wasError) return;

            if (Input.GetKey(KeyCode.LeftArrow))
            {
                dx = -0.2f;
            }
            if (Input.GetKey(KeyCode.RightArrow))
            {
                dx = 0.2f;
            }
            if (Input.GetKey(KeyCode.UpArrow))
            {
                dy = 0.2f;
            }
            if (Input.GetKey(KeyCode.DownArrow))
            {
                dy = -0.2f;
            }

            if (Input.GetKeyDown(KeyCode.Space))
            {
                m_drawFluidVolume = !m_drawFluidVolume;
                m_drawFluidParticles = !m_drawFluidParticles;
            }

            if (Input.GetKeyDown(KeyCode.T))
            {
                solid_run = !solid_run;
            }

            if (Input.GetKeyDown(KeyCode.R))
            {
                SceneManager.LoadScene("scene");
            }

            if (m_run)
            {
                // 基于求解器刷新渲染器
                m_solver.StepPhysics(timeStep, m_solid);
                m_volume.FillVolume(m_fluid, m_solver.Hash, m_solver.Kernel);
            }

            if (solid_run)
            {
                m_solidsolver.StepPhysics(timeStep);
            }

            if (m_control)
            {
                m_solidsolver.MoveTowards(dx, dy, dz);
                dx = 0; dy = 0; dz = 0;
            }
            m_solid.Draw(Camera.main, m_sphereMesh, m_boundaryParticleMat, 0);

            m_volume.Hide = !m_drawFluidVolume;

            if (m_drawBoundaryParticles)
                m_boundary.Draw(Camera.main, m_sphereMesh, m_boundaryParticleMat, 0);

            if (m_drawFluidParticles)
                m_fluid.Draw(Camera.main, m_sphereMesh, m_fluidParticleMat, 0);
        }

        private void OnDestroy()
        {
            m_boundary.Dispose();
            m_fluid.Dispose();
            m_solid.Dispose();
            m_solver.Dispose();
            m_solidsolver.Dispose();
            m_volume.Dispose();
        }

        private void OnRenderObject()
        {
            Camera camera = Camera.current;
            if (camera != Camera.main) return;

            if (m_drawLines)
            {
                //DrawBounds(camera, Color.green, m_boundary.Bounds);
                //DrawBounds(camera, Color.blue, m_fluid.Bounds);

                DrawBounds(camera, Color.green, m_outerSource);
                DrawBounds(camera, Color.red, m_innerSource);
                DrawBounds(camera, Color.blue, m_fluidSource);
                DrawBounds(camera, Color.yellow, m_solidSource);
            }

            if(m_drawGrid)
            {
                m_solver.Hash.DrawGrid(camera, Color.yellow);
            }
        }

        private void CreateBoundary(float radius, float density)
        {
            Bounds innerBounds = new Bounds();
            Vector3 min = new Vector3(-8, 0, -2);
            Vector3 max = new Vector3(8, 10, 2);
            innerBounds.SetMinMax(min, max);

            //Make the boundary 1 particle thick.
            //The multiple by 1.2 adds a little of extra
            //thickness in case the radius does not evenly
            //divide into the bounds size. You might have
            //particles missing from one side of the source
            //bounds other wise.

            float thickness = 1;
            float diameter = radius * 2;
            min.x -= diameter * thickness * 1.2f;
            min.y -= diameter * thickness * 1.2f;
            min.z -= diameter * thickness * 1.2f;

            max.x += diameter * thickness * 1.2f;
            max.y += diameter * thickness * 1.2f;
            max.z += diameter * thickness * 1.2f;

            Bounds outerBounds = new Bounds();
            outerBounds.SetMinMax(min, max);

            // 复用方法来创建一层边界粒子
            //The source will create a array of particles
            //evenly spaced between the inner and outer bounds.
            ParticleSource source = new ParticlesFromBounds(diameter, outerBounds, innerBounds);
            Debug.Log("Boundary Particles = " + source.NumParticles);

            m_boundary = new FluidBoundary(source, radius, density, Matrix4x4.identity);

            // 保存创建用的bound边界数据
            m_innerSource = innerBounds;
            m_outerSource = outerBounds;
        }

        private void CreateFluid( float radius, float density)
        {
            // 直接设定边界
            Bounds bounds = new Bounds();
            Vector3 min = new Vector3(-8, 0, -1);
            Vector3 max = new Vector3(-4, 8, 2);

            min.x += radius;
            min.y += radius;
            min.z += radius;

            max.x -= radius;
            max.y -= radius;
            max.z -= radius;

            bounds.SetMinMax(min, max);

            //The source will create a array of particles
            //evenly spaced inside the bounds. 
            //Multiple the spacing by 0.9 to pack more
            //particles into bounds.
            float diameter = radius * 2;
            ParticlesFromBounds source = new ParticlesFromBounds(diameter * 0.9f, bounds);
            Debug.Log("Fluid Particles = " + source.NumParticles);

            m_fluid = new FluidBody(source, radius, density, Matrix4x4.identity);

            // 保存创建用的bound边界数据
            m_fluidSource = bounds;
        }

        private void CreateFluid(float radius, float density, Bounds RectBounds)
        {
            // 直接设定边界
            Bounds bounds = new Bounds();
            Vector3 min = RectBounds.min;
            Vector3 max = RectBounds.max;

            min.x += radius;
            min.y += radius;
            min.z += radius;

            max.x -= radius;
            max.y -= radius;
            max.z -= radius;

            bounds.SetMinMax(min, max);

            //The source will create a array of particles
            //evenly spaced inside the bounds. 
            //Multiple the spacing by 0.9 to pack more
            //particles into bounds.
            float diameter = radius * 2;
            ParticlesFromBounds source = new ParticlesFromBounds(diameter * 0.9f, bounds);
            Debug.Log("Fluid Particles = " + source.NumParticles);

            m_fluid = new FluidBody(source, radius, density, Matrix4x4.identity);

            // 保存创建用的bound边界数据
            m_solidSource = bounds;
        }

        private void CreateSolid(float radius, float density)
        {
            // 直接设定生成的边界
            Bounds bounds = new Bounds();
            Vector3 min = new Vector3(-4, 0, -1f);
            Vector3 max = new Vector3(-2, 2, 1f);
            //Vector3 min = new Vector3(-8, 0, -3);
            //Vector3 max = new Vector3(-4, 8, 3);

            min.x += radius;
            min.y += radius;
            min.z += radius;

            max.x -= radius;
            max.y -= radius;
            max.z -= radius;

            bounds.SetMinMax(min, max);
            bounds.center += new Vector3(6, 4, 0);

            //The source will create a array of particles
            //evenly spaced inside the bounds. 
            //Multiple the spacing by 0.9 to pack more
            //particles into bounds.
            float diameter = radius * 2;
            ParticlesFromBounds source = new ParticlesFromBounds(diameter * 0.9f, bounds);
            Debug.Log("Solid Particles = " + source.NumParticles);

            m_solid = new SolidBody(source, radius, density, Matrix4x4.identity);

            // 保存创建用的bound边界数据
            m_solidSource = bounds;
        }


        private static IList<int> m_cube = new int[]
        {
            0, 1, 1, 2, 2, 3, 3, 0,
            4, 5, 5, 6, 6, 7, 7, 4,
            0, 4, 1, 5, 2, 6, 3, 7
        };

        Vector4[] m_corners = new Vector4[8];
        public void GetCorners(Bounds b)
        {
            m_corners[0] = new Vector4(b.min.x, b.min.y, b.min.z, 1);
            m_corners[1] = new Vector4(b.min.x, b.min.y, b.max.z, 1);
            m_corners[2] = new Vector4(b.max.x, b.min.y, b.max.z, 1);
            m_corners[3] = new Vector4(b.max.x, b.min.y, b.min.z, 1);

            m_corners[4] = new Vector4(b.min.x, b.max.y, b.min.z, 1);
            m_corners[5] = new Vector4(b.min.x, b.max.y, b.max.z, 1);
            m_corners[6] = new Vector4(b.max.x, b.max.y, b.max.z, 1);
            m_corners[7] = new Vector4(b.max.x, b.max.y, b.min.z, 1);
        }

        private void DrawBounds(Camera cam, Color col, Bounds bounds)
        {
            GetCorners(bounds);
            DrawLines.LineMode = LINE_MODE.LINES;
            DrawLines.Draw(cam, m_corners, col, Matrix4x4.identity, m_cube);
        }

    }

}
