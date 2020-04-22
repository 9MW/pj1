#pragma once
#include"physics/physics.hpp"
#include"bx/bx.h"
#include "common/Math.h"
#include"common/span.hpp"
#include <unordered_set>
using namespace nonstd;

struct Collid2
{


	UINT8 Mark;
	bool IsTrigger;
	float MaxVelocitySq, MaxAngularVeocity;
	float3 Acceleration, AngularAcceleration,
		Velocity, DisPlace,
		AngularVelocity, FinalAngularVelocity;
	float friction;
	struct
	{
		float v;

		template<typename T> constexpr	T operator *(const T& va) {
			return v * va;
		}

		operator float& () { return v; }
	private:
		void operator=(float value) {

			v = 1 / value;
		}
	} InvMass;
	float AngularFriction, InvInertia, ConvexR;;
	float& Mass = InvMass;


	void* data = NULL;
	template<typename T>
	T& getData() {
		return *(T*)(data);
	}


};
//
//inline void test(span<mtransfrom> arr, span<Collid> collids) {
//	//var ct=BXN;
//
//}
float4 inf4(float i) {
	return float4(i, i, i, i);
}
var lam1 = [=](int i) {
	return IBoxCollider::CreatCollid(float2(i, i), i, false);
};

var lam2 = [=](int i) {
	return ICircleCollider::CreatCollid(i, i, false);
};
template<typename T>
void setVector(std::vector<T> v, span<T> s) {
	v.data = s.data();
	v.size = s.size();
}

template< class T >
class ay
{
public:
	// constants and types

	T* p;
	int size;
	inline void operator=(span<T> sp) {
		p = &sp[0];
		size = sp.size();
	}
	inline T& operator[](int i) { return p[i]; }
};
ay<Collid> ss2; ay<mtransfrom> mtrans2;
std::unordered_set<float> map;
std::vector<contact> ct;
inline void start(span<Collid> ss, span<mtransfrom> mtrans) {
	var bxs = static_cast<IBoxCollider*>(malloc(sizeof(IBoxCollider) * ss.size()));
	//IOFormat CommaInitFmt(FullPrecision, DontAlignCols);
	for (size_t i = 0; i < ss.size(); i++)
	{
		mtransfrom& v = mtrans[i];

		v.rotation.setIdentity();
		float f = (i * 3);

		v.pos[1] = f;
		ss[i] = IBoxCollider::Creat(float2(2, 2), 10, false, bxs + i);
		ss[i].InvMass = i;
		ss[i].Mass = 1;
		ss[i].Velocity[1] = 0;
		//LOGINFO("i {0} inv {1} {2}", i, ss[i].InvMass, f);
	}
	for (size_t i = 1; i < 5; i++)
	{

		//ss[i].AngularVelocity[2] = i / 10.0;
		ss[i].Velocity[1] =  -0.1;
	}
	ss[0].Velocity[1] = 0;
	ss[0].Mass = 0;

	ss2 = ss,
		mtrans2 = mtrans;
}
std::vector<float2> pairs;
void en() {


	map.clear();
	pairs.clear();
	ct.clear();
	for (size_t i = 0; i < ss2.size; i++)
	{
		//ss2[i].Velocity = float3(0, -0.1, 0);
		IntegrateMotion(mtrans2[i], ss2[i], 0.2);
		
		var q = mtrans2[i].rotation;
		if(q.Identity().coeffs().cwiseNotEqual(q.coeffs()).any())
		LOGINFO("i={0} transfrom={1}", i, mtrans2[i].ToString());
	}
	//return;
	spdlog::set_pattern(" %^[%v]%$--%f");

	for (size_t i = 0; i < ss2.size; i++)
	{
		contact lt;
		Collid& c1 = ss2[i]; mtransfrom& t1 = mtrans2[i];
		for (size_t e = 0; e < ss2.size; e++)
		{
			if (e == i || (map.find(Utility::CantorPolynomialOrderLess(i, e)) != map.end()))continue;
			Collid& c2 = ss2[e];
			mtransfrom& t2 = mtrans2[e];
			BoxBox2(&lt, c1, c2, t1, t2, 0.2);
			if (lt.pointCount > 0) {
				ct.push_back(lt);
				var id = Utility::CantorPolynomialOrderLess(i, e);
				map.insert(id);
				pairs.push_back(float2(i, e));
				break;
			}
		}
	}
	LOGINFO("start");
	for (size_t i = 0; i < pairs.size(); i++)
	{
		LOGINFO("i : {0} p({1},{2})", i, pairs[i][0], pairs[i][1]);
	}

	for (size_t i = 0; i < 6; i++) {
		size_t M = 0;
		for (var& ele : ct)
		{
			if (ele.pointCount > 0) {
				if (SolvePositionConstraints(&ele))
				{
					M++;
				}
			}
		//	M++;
		}
		LOGINFO("iter :{0} M = {1}", i, M==ct.size()-1);
	}

}
void exit() {
	spdlog::shutdown();
}