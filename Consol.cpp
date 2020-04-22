// Consol.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#define SILENCE_CXX17_RESULT_OF_DEPRECATION_WARNING
#include <bx/timer.h>
#include <bx/math.h>
#include<bx/allocator.h>
#include <tinystl/allocator.h>
#include <bx/file.h>
#include <bx/sort.h>

#include <bgfx/bgfx.h>
#include "common/entry/entry.h"
#include <bx\readerwriter.h>
#include <common\dbg.h>

#include <iostream>

#include"common/span.hpp"
#include "physics/physics.hpp"
#include"Test.hpp"
//#include"physics/ps.cpp"
using namespace Eigen;     // 改成这样亦可 using Eigen::MatrixXd; 
using namespace std;
using namespace bx;
using namespace entry;

using float2 = Vector2f;
using float3 = Vector3f;
using float4 = Vector4f;
using float4x4 = Matrix4f;
using bool3 = Array3<bool>;
using  zero = float2;
using quaternion = Eigen::Quaternionf;

using namespace nonstd;
namespace ins
{

	struct PosColorVertex
	{
		float m_x;
		float m_y;
		float m_z;
		uint32_t m_abgr;

		static void init()
		{
			ms_layout
				.begin()
				.add(bgfx::Attrib::Position, 3, bgfx::AttribType::Float)
				.add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
				.end();
		};

		static bgfx::VertexLayout ms_layout;
	};

	bgfx::VertexLayout PosColorVertex::ms_layout;

	bgfx::InstanceDataBuffer idb;
	//bgfx::DynamicVertexBufferHandle idb;
	bgfx::DynamicVertexBufferHandle m_culledInstanceBuffer;
	bgfx::IndirectBufferHandle m_indirectBuffer;
	const int m_noofProps = 1;
	static PosColorVertex s_cubeVertices[8] =
	{
		{-1.0f,  1.0f,  1.0f, 0xff000000 },
		{ 1.0f,  1.0f,  1.0f, 0xff0000ff },
		{-1.0f, -1.0f,  1.0f, 0xff00ff00 },
		{ 1.0f, -1.0f,  1.0f, 0xff00ffff },
		{-1.0f,  1.0f, -1.0f, 0xffff0000 },
		{ 1.0f,  1.0f, -1.0f, 0xffff00ff },
		{-1.0f, -1.0f, -1.0f, 0xffffff00 },
		{ 1.0f, -1.0f, -1.0f, 0xffffffff },
	};

	static const uint16_t s_cubeIndices[36] =
	{
		0, 1, 2, // 0
		1, 3, 2,
		4, 6, 5, // 2
		5, 6, 7,
		0, 2, 4, // 4
		4, 2, 6,
		1, 5, 3, // 6
		5, 7, 3,
		0, 4, 1, // 8
		4, 5, 1,
		2, 3, 6, // 10
		6, 3, 7,
	};

	void good(span<mtransfrom> arr)
	{

		for (size_t i = 0; i != arr.size(); ++i)
		{
			std::cout << (i == 0 ? "[" : "") << arr[i].pos << (i != arr.size() - 1 ? ", " : "]\n");
		}
	}
	const  uint32_t numInstances = 50;

	static	Vec3 rect((float)120, 60, 0);
	float2 offset;
	class ExampleInstancing : public entry::AppI
	{
		PosColorVertex* db;
		mtransfrom mtrans[numInstances];
		Collid cs[numInstances];
	public:
		ExampleInstancing(const char* _name, const char* _description, const char* _url)
			: entry::AppI(_name, _description, _url)
		{
		}
		void init(int32_t _argc, const char* const* _argv, uint32_t _width, uint32_t _height) override
		{
			{


				start(cs, mtrans);
			}
			m_width = _width;
			m_height = _height;
			m_debug = BGFX_DEBUG_TEXT;
			m_reset = BGFX_RESET_VSYNC;

			bgfx::Init init;
			init.type = bgfx::RendererType::Enum::Count;
			init.vendorId = 0;
			init.resolution.width = m_width;
			init.resolution.height = m_height;
			init.resolution.reset = m_reset;
			bgfx::init(init);

			// Enable debug text.
			bgfx::setDebug(m_debug);

			// Set view 0 clear state.
			bgfx::setViewClear(0
				, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH
				, 0x303030ff
				, 1.0f
				, 0
			);

			// Create vertex stream declaration.
			PosColorVertex::init();

			// Create static vertex buffer.
			m_vbh = bgfx::createVertexBuffer(
				bgfx::makeRef(s_cubeVertices, sizeof(s_cubeVertices))
				, PosColorVertex::ms_layout
			);

			// Create static index buffer.
			m_ibh = bgfx::createIndexBuffer(
				bgfx::makeRef(s_cubeIndices, sizeof(s_cubeIndices))
			);

			// Create program from shaders.
			m_program = loadProgram("vs_instancing", "fs_instancing");

			m_timeOffset = bx::getHPCounter();
			//we use one "drawcall" per prop to render all its instances
			m_indirectBuffer = bgfx::createIndirectBuffer(m_noofProps);
			bgfx::setBuffer(0, m_indirectBuffer, bgfx::Access::ReadWrite);
			//post occlusion buffer
			m_culledInstanceBuffer = bgfx::createDynamicVertexBuffer(4 * m_noofProps, PosColorVertex::ms_layout, BGFX_BUFFER_COMPUTE_WRITE);

			bgfx::setBuffer(1, m_culledInstanceBuffer, bgfx::Access::Write);

		}

		const uint16_t instanceStride = 80;
		int shutdown() override
		{
			exit();
			// Cleanup.
			bgfx::destroy(m_ibh);
			bgfx::destroy(m_vbh);
			bgfx::destroy(m_program);

			// Shutdown bgfx.
			bgfx::shutdown();

			return 0;
		}

		Collid cc[20000];
		int i = 0;


		entry::MouseState m_mouseState, o_mouseState;
		bool update() override
		{

			float time = (float)((bx::getHPCounter() - m_timeOffset) / double(bx::getHPFrequency()));
			var  bx = cs[0].getData<IBoxCollider>(); var  y = cs[1].getData<IBoxCollider>()[0][1],
				py = mtrans[1].pos[0];

			en();
			bgfx::dbgTextPrintf(0, 2, 0x0f, "box1 %dW x %dH in pixels, debug text %dW x %dH in characters."
				, bx.m_vertices[0]
				, y
				, py
				, 55
			);
			bgfx::dbgTextPrintf(0, 1, 0x0f, "Color can be changed with ANSI \x1b[9;me\x1b[10;ms\x1b[11;mc\x1b[12;ma\x1b[13;mp\x1b[14;me\x1b[0m code too.");

			bgfx::dbgTextPrintf(80, 1, 0x0f, "\x1b[;0m    \x1b[;1m    \x1b[; 2m    \x1b[; 3m    \x1b[; 4m    \x1b[; 5m    \x1b[; 6m    \x1b[; 7m    \x1b[0m");
			bgfx::dbgTextPrintf(80, 2, 0x0f, "\x1b[;8m    \x1b[;9m    \x1b[;10m    \x1b[;11m    \x1b[;12m    \x1b[;13m    \x1b[;14m    \x1b[;15m    \x1b[0m");
			o_mouseState = m_mouseState;
			if (!entry::processEvents(m_width, m_height, m_debug, m_reset, &m_mouseState))
			{
			
				/*LOGINFO("x {0} y {1} z {2} botton {3}", m_mouseState.m_mx, m_mouseState.m_my,
					m_mouseState.m_mz, fmt::join(m_mouseState.m_buttons, ","));*/
					// Set view 0 default viewport.
				bgfx::setViewRect(0, 0, 0, uint16_t(m_width), uint16_t(m_height));

				// This dummy draw call is here to make sure that view 0 is cleared
				// if no other draw calls are submitted to view 0.
				bgfx::touch(0);

				// Get renderer capabilities info.
				const bgfx::Caps* caps = bgfx::getCaps();

				// Check if instancing is supported.
				if (0 == (BGFX_CAPS_INSTANCING & caps->supported))
				{
					// When instancing is not supported by GPU, implement alternative
					// code path that doesn't use instancing.
					bool blink = uint32_t(time * 3.0f) & 1;
					bgfx::dbgTextPrintf(0, 0, blink ? 0x4f : 0x04, " Instancing is not supported by GPU. ");
				}
				else
				{
					const bx::Vec3 at = { 0.0f, 0.0f,   0.0f };
					const bx::Vec3 eye = { 0.0f, 0.0f, -35.0f };
					const bgfx::Memory* mem;
					// Set view and projection matrix for view 0.
					{
						float view[16];
						bx::mtxLookAt(view, eye, at);
						float proj[16];
						float2 dM((float)m_mouseState.m_mx - o_mouseState.m_mx, (float)m_mouseState.m_my - o_mouseState.m_my);
						if (!m_mouseState.m_buttons[entry::MouseButton::Right] && (!dM.isZero())) {
							dM.setConstant(0);
						}
						else
						{
							dM[0] = -dM[0];
						}
						offset += dM * 0.2;
						//bx::mtxOrtho(proj, 60.0f, float(m_width) / float(m_height), 0.1f, 100.0f, bgfx::getCaps()->homogeneousDepth
						bx::mtxOrtho(proj, -rect.x / 2 + offset[0], rect.x / 2 + offset[0], -rect.y / 2 + offset[1], rect.y / 2 + offset[1], -100, 100, 0, bgfx::getCaps()->homogeneousDepth);
						bgfx::setViewTransform(0, view, proj);

						// Set view 0 default viewport.
						bgfx::setViewRect(0, 0, 0, uint16_t(m_width), uint16_t(m_height));
					}
					//#if VG_CONFIG_MULTIDRAW_INDIRECT
					//				bgfx::VertexBufferHandle dummyVB = bgfx::createVertexBuffer(bgfx::copy(ctx->m_IndirectData, sizeof(IndirectData) * ctx->m_NumIndirect), ctx->m_IndirectDataLayout, BGFX_BUFFER_DRAW_INDIRECT);
					//				bgfx::IndirectBufferHandle indirectBuffer = { dummyVB.idx };
					//
					//				bgfx::InstanceDataBuffer idb;
					//				bgfx::allocInstanceDataBuffer(&idb, ctx->m_NumIndirect, sizeof(InstanceData));
					//				bx::memCopy(idb.data, ctx->m_InstanceData, sizeof(InstanceData) * ctx->m_NumIndirect);

									// 80 bytes stride = 64 bytes for 4x4 matrix + 16 bytes for RGBA color.
									/*bx::DefaultAllocator all;
									db = (PosColorVertex*)BX_ALLOC(&all, instanceStride * numInstances);
									idb = bgfx::createDynamicVertexBuffer(numInstances, PosColorVertex::ms_layout);*/
									// 11x11 cubes
							/*		auto nnn = bgfx::getAvailInstanceDataBuffer(numInstances, instanceStride);
									if (numInstances == nnn )*/
					{


						float time = (float)((bx::getHPCounter() - m_timeOffset) / double(bx::getHPFrequency()));

						bgfx::allocInstanceDataBuffer(&idb, numInstances, instanceStride);
						//bx::memCopy(idb,db,)
						var data = idb.data;

						const auto sqt = bx::sqrt(numInstances);
						int i = 0;
						// Write instance data for 11x11 cubes.
						for (uint32_t yy = 0; yy < sqt - 1; ++yy)
						{
							for (uint32_t xx = 0; xx < sqt; ++xx)
							{
								var& q = mtrans[i];

								float* mtx = (float*)data;
								//mtrans[i].rotation= AngleAxisf(time + xx * 0.21f,float3::UnitZ());
								memSet(mtx, 0, sizeof(float) * 16);
								bx::mtxRotateXY(mtx, 0, 0);
								(Map<Matrix4f>((mtx))).block<3, 3>(0, 0) = (mtrans[i].rotation.toRotationMatrix());

								//bx::mtxRotateXY(mtx, time + xx * 0.21f, time + yy * 0.37f);
								float* ppp = &mtx[12];
								*(float3*)((ppp)) = mtrans[i].pos;
								/*mtx[13] = -15.0f + float(yy) * 3.0f;
								mtx[14] = 0.0f;*/

								float* color = (float*)&data[64];
								color[0] = bx::sin(time + float(xx) / 11.0f) * 0.5f + 0.5f;
								color[1] = bx::cos(time + float(yy) / 11.0f) * 0.5f + 0.5f;
								color[2] = bx::sin(time * 3.0f) * 0.5f + 0.5f;
								color[3] = 1.0f;
								i++;
								data += instanceStride;
							}
						}
						/*mem = bgfx::makeRef(&data, instanceStride * numInstances);
						bgfx::update(idb, 0, mem);*/
						// Set vertex and index buffer.
						bgfx::setVertexBuffer(0, m_vbh);
						bgfx::setIndexBuffer(m_ibh);
						// Set instance data buffer.
						bgfx::setInstanceDataBuffer(&idb, 0, numInstances);

						// Set render states.
						bgfx::setState(BGFX_STATE_DEFAULT);

						// Submit primitive for rendering to view 0.
						bgfx::submit(0, m_program);//, m_indirectBuffer,0, m_noofProps,0,false);
					}
				}

				// Advance to next frame. Rendering thread will be kicked to
				// process submitted rendering primitives.
				bgfx::frame();

				return true;
			}

			return false;
		}
		static bgfx::ShaderHandle loadShader(bx::FileReaderI* _reader, const char* _name)
		{
			char filePath[512];

			const char* shaderPath = "???", * dir = "./";

			switch (bgfx::getRendererType())
			{
			case bgfx::RendererType::Noop:
			case bgfx::RendererType::Direct3D9:  shaderPath = "shaders/dx9/";   break;
			case bgfx::RendererType::Direct3D11:
			case bgfx::RendererType::Direct3D12: shaderPath = "shaders/dx11/";  break;
			case bgfx::RendererType::Gnm:        shaderPath = "shaders/pssl/";  break;
			case bgfx::RendererType::Metal:      shaderPath = "shaders/metal/"; break;
			case bgfx::RendererType::Nvn:        shaderPath = "shaders/nvn/";   break;
			case bgfx::RendererType::OpenGL:     shaderPath = "shaders/glsl/";  break;
			case bgfx::RendererType::OpenGLES:   shaderPath = "shaders/essl/";  break;
			case bgfx::RendererType::Vulkan:     shaderPath = "shaders/spirv/"; break;

			case bgfx::RendererType::Count:
				BX_CHECK(false, "You should not be here!");
				break;
			}

			bx::strCopy(filePath, BX_COUNTOF(filePath), dir);
			bx::strCat(filePath, BX_COUNTOF(filePath), shaderPath);
			bx::strCat(filePath, BX_COUNTOF(filePath), _name);
			bx::strCat(filePath, BX_COUNTOF(filePath), ".bin");

			bgfx::ShaderHandle handle = bgfx::createShader(loadMem(_reader, filePath));
			bgfx::setName(handle, _name);

			return handle;
		}

		static const bgfx::Memory* loadMem(bx::FileReaderI* _reader, const char* _filePath)
		{
			if (bx::open(_reader, _filePath))
			{
				uint32_t size = (uint32_t)bx::getSize(_reader);
				const bgfx::Memory* mem = bgfx::alloc(size + 1);
				bx::read(_reader, mem->data, size);
				bx::close(_reader);
				mem->data[mem->size - 1] = '\0';
				return mem;
			}

			DBG("Failed to load %s.", _filePath);
			return NULL;
		}
		bgfx::ProgramHandle loadProgram(const char* _vsName, const char* _fsName)
		{
			//var all = &DefaultAllocator();
			var	s_fileReader = entry::getFileReader();
			var p = loadProgram(s_fileReader, _vsName, _fsName);
			//BX_DELETE(all, s_fileReader);
			s_fileReader = NULL;
			return p;
		}

		bgfx::ProgramHandle loadProgram(bx::FileReaderI* _reader, const char* _vsName, const char* _fsName)
		{
			bgfx::ShaderHandle vsh = loadShader(_reader, _vsName);
			bgfx::ShaderHandle fsh = BGFX_INVALID_HANDLE;
			if (NULL != _fsName)
			{
				fsh = loadShader(_reader, _fsName);
			}

			return bgfx::createProgram(vsh, fsh, true /* destroy shaders when program is destroyed */);
		}

		uint32_t m_width;
		uint32_t m_height;
		uint32_t m_debug;
		uint32_t m_reset;
		bgfx::VertexBufferHandle m_vbh;
		bgfx::IndexBufferHandle  m_ibh;
		bgfx::ProgramHandle m_program;

		int64_t m_timeOffset;
	};
	ExampleInstancing a2("", "", "");
} // namespace

//using namespace std; struct my_type
//{
//    int i;
//    template<typename OStream>
//    friend OStream& operator<<(OStream& os, const my_type& c)
//    {
//        return os << "[my_type i=" << c.i << "]";
//    }
//};
//
//void user_defined_example()
//{
//    spdlog::stdout_logger_mt("console")->info("user defined type: {}", my_type{ 14 });
//}
struct float9
{
	float v[3];
	inline float& operator[](int i) {
		return v[i];
	}
	/*template<T>
	friend V<T> operator+(V<T> const& lhs, V<T> const& rhs) {
		...
	}*/
	inline operator float2& () {
		float2 f2(&v[0]);
		Ref<float2> v2 = Vector2f::MapAligned(&v[0]);
		return f2;
	}
	struct
	{
		float v;
		void operator=(float value) {

			v = 1 / value;
		}

		operator float& () { return v; }
	} InvMass;
	float& x = InvMass;
};

struct  kk {
	float4 vv;
	inline  float2 operator ()(const kk& k)const {
		return (k.vv.head(2));

	}
	kk(float a ...) {
		vv(a);
	}
	Ref<float2> f2 = vv.head(2);
}; template<typename T>
class zero_init
{
	T val;
public:
	zero_init() : val(static_cast<T>(0)) { }
	zero_init(T val) : val(val) { }
	operator T& () { return val; }
	operator T() const { return val; }
};

void init()
{

	float ii;

	float2 cw(1, 0), ff(0, 0);
	/* float3 dd(1, 2, 3);*/
	kk k(1, 2, 3, 4);
	float2 c;
	c += c;

	//c += kk;
	c.setConstant(0);
	float9 f9 = { 19,92,9,90 };


	float4 Acceleration(1, 2, 0, 0), AngularAcceleration(3, 4, 0, 0);
	//  var tsd =quaternion(1, 0, 0, 0)*c;



		//  Utility::Determinant(Acceleration.head(2), float2::Map(AngularAcceleration));
	float2 d;// = Utility::ProjectUnrestrict(float2(1, 2), float2(0, 0), c, ii);
	Array3d v(-1, 2, 1), w(-3, 2, 3);
	var bb = d == float2::Zero();
	IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	auto gg = ((v < 0)).format(CleanFmt);
	std::cout << "dd= \n" << d << "\n ";

	const float3 d3(c[0], c[1], 0);
	LOGINFO("x={0}", f9.x);
	f9.InvMass = 2;
	LOGINFO("x={0}", f9.x);
	// user_defined_example;
	 //spdlog::info("Welcome ");
	 //spdlog::error("Some error message with arg: {}", 1);

	 //spdlog::warn("Easy padding in numbers like {:08d}", 12);
	 //spdlog::critical("Support for int: {0:d};  hex: {0:x};  oct: {0:o}; bin: {0:b}", 42);
	 //spdlog::info("Support for floats {:03.2f}", 1.23456);
	 //spdlog::info("Positional args are {1} {0}..", "too", "supported");
	 //spdlog::info("{:<30}", "left aligned");

	 //spdlog::set_level(spdlog::level::debug); // Set global log level to debug
	 //spdlog::debug("This message should be displayed..");

	 //// change log pattern
	 //spdlog::set_pattern("[%H:%M:%S %z] [%n] [%^---%L---%$] [thread %t] %v");

	 // Compile time log levels
	 // define SPDLOG_ACTIVE_LEVEL to desired level
	 /*SPDLOG_TRACE("Some trace message with param {}", 42);
	 SPDLOG_DEBUG("Some debug message");*/

}
//#include <iostream>
//
//int main()
//{
//    std::cout << "Hello World!\n";
//}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
