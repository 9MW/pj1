#pragma once
#include "common/Math.h"
#include"Log/log.h"
#include"bx/bx.h"

constexpr float MaxObjVelocity = 5, maxLoop = 32, EpsilonSq = Epsilon * Epsilon;

enum class ColliderType :uint8_t
{
	Box, Circle
};
struct mtransfrom
{
	float3 pos;
	float4 Scale;
	quaternion rotation;
	mtransfrom() {};
	mtransfrom(float3 pos, quaternion b) :pos(pos), rotation(b) {}
	var ToString() {
		return fmt::format("pos({0}),rotation{1}", fmt::join(pos, ","),FmQuar(rotation));
	}
	/* void ToString()
	{
		LOGINFO("pos={0} rotation={1}", pos, rotation);
	}*/
};
template<typename T>
struct AssetReference
{
	T Value;
	AssetReference(T& v) :Value(v) {}
};
struct Collid
{

	ColliderType colliderType;
	uint8 Mark;
	bool IsTrigger;
	float MaxVelocitySq, MaxAngularVeocity;
	float3 Acceleration, AngularAcceleration,
		Velocity, DisPlace,
		AngularVelocity, FinalAngularVelocity;
	float friction;
	float InvMass;
	float AngularFriction, InvInertia;
	float Mass;
	/*void Mass(float v) {
		 InvMass=1/v;
	}*/
	void* data = NULL;
	float ConvexR;
	template<typename T>
	T& getData() {
		return *(T*)(data);
	}

	
};

struct ICircleCollider
{
	// ref Collid MyCollider.Head {get{fixed (ICircleCollider* ts= &this){ return ref ts->Head; } } }

	static Collid CreatCollid(float mass, float r, bool IsTrigger) {
		return Creat(mass, r, IsTrigger).Value;
	}
	static inline AssetReference<Collid> Creat(float mass, float r, bool IsTrigger)
	{
		Collid c;
		c.IsTrigger = IsTrigger;
		c.Mass = mass;
		c.ConvexR = r;
		c.colliderType = ColliderType::Circle;
		AssetReference<Collid> blob(c);
		return blob;
	}

};
struct IBoxCollider
{
	static std::vector<IBoxCollider> Pool;
	static const int m_count = 4;
	float2 m_vertices[m_count];
	const float2& max = m_vertices[0];
	float2 normals[m_count];
	inline static  Collid CreatCollid(const float2& size, const float mass, const bool IsTrigger)
	{
		return Creat(size, mass, IsTrigger).Value;
	}
	inline static AssetReference<Collid> Creat(const float2& size, const float mass, const bool IsTrigger)
	{
		float2 a = size * 0.5f, b = a.cwiseProduct(float2(-1, 1)), c = -a, d = float2(a[0], -a[1]);
		IBoxCollider box;
		var m_vertices = box.m_vertices;
		m_vertices[0]=(a);
		m_vertices[1]=(b);
		m_vertices[2]=(c);
		m_vertices[3]=(d);
		var sizesq = size.squaredNorm();
		Collid cd; cd.IsTrigger = IsTrigger;
		cd.InvInertia = 12 / (sizesq * mass);
		cd.Mass= (mass); cd.data = &box;
		cd.ConvexR = math::sqrt(sizesq) * 0.25f;
		cd.data = &box;
		cd.colliderType = ColliderType::Box;
		var m_normals = box.normals;

		m_normals[2]=float2(0.0f, -1.0f);
		m_normals[3]=float2(1.0f, 0.0f);
		m_normals[0]=float2(0.0f, 1.0f);
		m_normals[1]=float2(-1.0f, 0.0f);
		AssetReference<Collid> blob(cd);
		/*{
			Matrix2f a, b, c = a * b; }*/

		return blob;
	}
	inline static Collid Creat(const float2& size, const float mass, const bool IsTrigger,IBoxCollider* bx)

	{
		var& box = *bx;
		float2 a = size * 0.5f, b = a.cwiseProduct(float2(-1, 1)), c = -a, d = float2(a[0], -a[1]);
		
		var m_vertices = box.m_vertices;
		//box.m_vertices[0][0] = 100;
		m_vertices[0] = (a);
		m_vertices[1] = (b);
		m_vertices[2] = (c);
		m_vertices[3] = (d);
		var sizesq = size.squaredNorm();
		Collid cd; cd.IsTrigger = IsTrigger;
		cd.InvInertia = 12 / (sizesq * mass);
		cd.InvMass = 1 / mass;
		cd.Mass = (mass); cd.data = &box;
		cd.ConvexR = math::sqrt(sizesq) * 0.25f;
		cd.data = &box;
		cd.colliderType = ColliderType::Box;
		var m_normals = box.normals;

		m_normals[2] = float2(0.0f, -1.0f);
		m_normals[3] = float2(1.0f, 0.0f);
		m_normals[0] = float2(0.0f, 1.0f);
		m_normals[1] = float2(-1.0f, 0.0f);
		/*{
			Matrix2f a, b, c = a * b; }*/

		return cd;
	}

	var operator[](int i) {
		return m_vertices[i];
	}
};
void CalculCollision(float restitution, Collid& ca, Collid& cb,
	const float3& impact, mtransfrom& transforma, mtransfrom& transformb, const float3& normala);
inline mtransfrom Mul(const mtransfrom& a, const mtransfrom& b) {
	return mtransfrom(a.pos + b.pos, a.rotation * b.rotation);
}
inline mtransfrom B2A(const mtransfrom& a, const mtransfrom& b) {
	mtransfrom result;
	result.rotation = b.rotation.inverse() * a.rotation;
	result.pos = a.rotation.inverse() * (b.pos - a.pos);

	return result;
}
inline const float3& Trans2d(const float3& tg, const mtransfrom& q)
{
	return q.pos.head(3) + q.rotation * tg.head(3);
}

inline float2 Trans2d(const float2& tg, const mtransfrom& q)
{
	return q.pos.head(2) + math::mulf2q(tg, q.rotation);
}

inline const float2 Trans2d(const mtransfrom& q, const float2& tg)
{
	return Trans2d(tg, q);
}

inline float2 Trans2dIv(const mtransfrom& q, const float2& tg)
{
	var t = tg - q.pos.head(2);
	return  math::mulf2q(t, q.rotation.inverse());
}
inline float2 Trans2d(const float2& tg, const float2& pos, const quaternion& rotation)
{
	const float3 f(tg[0], tg[1], 0);
	return pos + (rotation * f).head(2);
}
inline quaternion IntegrateAngularVelocity(float3 angularVelocity, float timestep)
{

	float3 halfDeltaAngle = angularVelocity * timestep * 0.5f;
	quaternion q;
	q.w() = 1;
	q.vec() = halfDeltaAngle;
	return q;
}
inline void IntegrateOrientation(quaternion& orientation, float3 angularVelocity, float timestep)
{
	quaternion dq = IntegrateAngularVelocity(angularVelocity, timestep);
	orientation = (orientation * dq).normalized();
}
/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline auto b2Cross(const float2& a, float s)
{
	return float2(s * a[1], -s * a[0]);
}
/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float b2Cross(const float2& a, const float2& b)
{
	return a[0] * b[1] - a[1] * b[0];
}

inline void CalculCollision(float restitution, Collid& ca, Collid& cb, const float2& impact, mtransfrom& transforma, mtransfrom& transformb, float2& normala) {
	const float3 i(impact[0], impact[1], 0), n(normala[0], normala[1], 0);
	CalculCollision(restitution, ca, cb, i, transforma, transformb, n);
}

inline float b2FindMaxSeparation(int32* edgeIndex,
	const IBoxCollider* poly1, const mtransfrom& xf1,
	const IBoxCollider* poly2, const mtransfrom& xf2)
{
	int32 count1 = poly1->m_count;
	int32 count2 = poly2->m_count;
	const var n1s = poly1->normals;
	const var v1s = poly1->m_vertices;
	const var v2s = poly2->m_vertices;
	mtransfrom xf = B2A(xf2, xf1);

	int32 bestIndex = 0;
	float maxSeparation = -Infinityf;
	for (int32 i = 0; i < count1; ++i)
	{
		// Get poly1 normal in frame2.
		float2 n = math::mulf2q(n1s[i], xf.rotation);
		var v1 = Trans2d(v1s[i], xf);

		// Find deepest point for normal i.
		float si = Infinityf;
		for (int32 j = 0; j < count2; ++j)
		{
			float sij = math::dot(n, v2s[j] - v1);
			if (sij < si)
			{
				si = sij;
			}
		}

		if (si > maxSeparation)
		{
			maxSeparation = si;
			bestIndex = i;
		}
	}

	*edgeIndex = bestIndex;
	return maxSeparation;
}
// Sutherland-Hodgman clipping.
inline int32  b2ClipSegmentToLine(float2 vOut[2], const float2 vIn[2],
	const float2& normal, float offset, int32 vertexIndexA)
{
	// Start with no output points
	int32 numOut = 0;

	// Calculate the distance of end points to the line
	float distance0 = math::dot(normal, vIn[0]) - offset;
	float distance1 = math::dot(normal, vIn[1]) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		float interp = distance0 / (distance0 - distance1);
		vOut[numOut] = vIn[0] + interp * (vIn[1] - vIn[0]);

		// VertexA is hitting edgeB.
	  /*vOut[numOut].id.cf.indexA = static_cast<uint8>(vertexIndexA);
		vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
		vOut[numOut].id.cf.typeA = b2ContactFeature::e_vertex;
		vOut[numOut].id.cf.typeB = b2ContactFeature::e_face;*/
		++numOut;
	}

	return numOut;
}
inline void b2FindIncidentEdge(float2 c[2],
	const IBoxCollider* poly1, const mtransfrom& xf1, int32 edge1,
	const IBoxCollider* poly2, const mtransfrom& xf2)
{
	const var normals1 = poly1->normals;

	int32 count2 = poly2->m_count;
	const var* vertices2 = poly2->m_vertices;
	const var* normals2 = poly2->normals;

	assert(0 <= edge1 && edge1 < poly1->m_count);

	// Get the normal of the reference edge in poly2's frame.
	var normal1 = math::mulf2q(xf2.rotation.inverse(), math::mulf2q(xf1.rotation, normals1[edge1]));

	// Find the incident edge on poly2.
	int32 index = 0;
	float minDot = Infinityf;
	for (int32 i = 0; i < count2; ++i)
	{
		float dot = math::dot(normal1, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	int32 i1 = index;
	int32 i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0] = Trans2d(xf2, vertices2[i1]);

	c[1] = Trans2d(xf2, vertices2[i2]);

}

struct contact
{
	Collid* box1, *box2;
	mtransfrom* xfa, *xfb;
	float2 points[2];	///< the points of contact
	float2 localNormal;								///< not use for Type::e_points
	float2 localPoint;								///< usage depends on manifold type
	//Type type;
	int32 pointCount;
};
inline void BoxBox2(contact* manifold, Collid& box1,  Collid& box2,
	mtransfrom& xfa, mtransfrom& xfb, float dt) {
	const float EP = Epsilon, totalRadius = EP * 2;
	manifold->pointCount = 0;
	var* poly1 = &box1.getData<IBoxCollider>();
	var* poly2 = &box2.getData<IBoxCollider>();
	int32 edgeA = 0;
	float separationA = b2FindMaxSeparation(&edgeA, poly1, xfa, poly2, xfb);
	if (separationA > EP)
		return;

	int32 edgeB = 0;
	float separationB = b2FindMaxSeparation(&edgeB, poly2, xfb, poly1, xfa);
	if (separationB > EP)
		return;
	int edge1 = edgeA;
	var xf1 = &xfa, xf2 = &xfb;
	bool filp = false;
	if (separationB > separationA)
	{
		std::swap(poly1, poly2);
		std::swap(xf1, xf2);
		manifold->box1 = &box2;
		manifold->box2 = &box1;
		manifold->xfa = xf1;
		manifold->xfb = xf2;
		edge1 = edgeB;
		filp = true;
	}
	else
	{

		manifold->xfa = xf1;
		manifold->xfb = xf2;
		manifold->box1 = &box1;
		manifold->box2 = &box2;
	}
	float2 incidentEdge[2];
	b2FindIncidentEdge(incidentEdge, poly1, *xf1, edge1, poly2, *xf2);
	int32 count1 = poly1->m_count;
	const var* vertices1 = (*poly1).m_vertices;

	int32 iv1 = edge1;
	int32 iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

	var v11 = vertices1[iv1];
	var v12 = vertices1[iv2];

	var localTangent = (v12 - v11).normalized();

	const var localNormal = b2Cross(localTangent, 1.0f);
	const var planePoint = 0.5f * (vertices1[iv1] + vertices1[iv2]);

	var tangent = math::mulf2q(xf1->rotation, localTangent);
	var normal = b2Cross(tangent, 1.0f);

	v11 = Trans2d(*xf1, v11);
	v12 = Trans2d(*xf1, v12);

	// Face offset.
	float frontOffset = math::dot(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	float sideOffset1 = -math::dot(tangent, v11) + totalRadius;
	float sideOffset2 = math::dot(tangent, v12) + totalRadius;

	// Clip incident edge against extruded edge1 side edges.
	float2 clipPoints1[2];
	float2 clipPoints2[2];
	int np;

	// Clip to box side 1
	np = b2ClipSegmentToLine(clipPoints1, incidentEdge, -tangent, sideOffset1, iv1);

	if (np < 2)
		return;

	// Clip to negative box side 1
	np = b2ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2);

	if (np < 2)
	{
		return;
	}

	// Now clipPoints2 contains the clipped points.
	manifold->localNormal = localNormal;
	manifold->localPoint = planePoint;
	//LOGINFO("x1.py {0} x2.py {1}", xf1->pos[1], xf2->pos[1]);
	int32 pointCount = 0;
	for (int32 i = 0; i < 2; ++i)
	{
		float separation = math::dot(normal, clipPoints2[i]) - frontOffset;

		if (separation <= totalRadius)
		{
			var* cp = manifold->points + pointCount;
			*cp = Trans2dIv(*xf2, clipPoints2[i]);

			++pointCount;
		}
	}

	manifold->pointCount = pointCount;
}


const float b2_baumgarte = 0.2f, b2_linearSlop = 0.005f, b2_maxLinearCorrection = 0.2f;
const IOFormat CommaInitFmt(2, DontAlignCols);
// Sequential solver.
inline bool SolvePositionConstraints(contact* manifold)
{
	float minSeparation = 0.0f;
	var m_count = 1;
	//for (int32 i = 0; i < m_count; ++i)
	{
		//b2ContactPositionConstraint* pc = m_positionConstraints + i;

		float mA =manifold->box1->InvMass;
		float iA =manifold->box1->InvInertia;
		float mB =manifold->box2->InvMass;
		float iB =manifold->box2->InvInertia;
		int32 pointCount = manifold->pointCount;
		Ref<float2> cA = (*manifold->xfa).pos.head(2);
		var& aA = (*manifold->xfa).rotation;

		Ref<float2> cB = (*manifold->xfb).pos.head(2);
		var& aB = (*manifold->xfb).rotation;

		// Solve normal constraints
		for (int32 j = 0; j < pointCount; ++j)
		{
			var xfA = *manifold->xfa, xfB =* manifold->xfb;
			/*xfA.q.Set(aA);
			xfB.q.Set(aB);
			xfA.p = cA - Trans2d(xfA.q, localCenterA);
			xfB.p = cB - Trans2d(xfB.q, localCenterB);*/


			var normal = math::mulf2q(xfA.rotation, manifold->localNormal);
			var planePoint = Trans2d(xfA, manifold->localPoint);

			var clipPoint = Trans2d(xfB, manifold->points[j]);
			var separation = math::dot(clipPoint - planePoint, normal)-0.02f;
			/*var clipPoint = Trans2d(xfA, manifold->points[j]);
			var separation = math::dot(clipPoint - planePoint, normal) - 0.02f;*/
			var point = clipPoint;
			var rA = point - cA;
			var rB = point - cB;

			// Track max constraint error.
			minSeparation = math::min(minSeparation, separation);

			// Prevent large corrections and allow slop. b2_baumgarte * (separation + b2_linearSlop);//
			float C =  math::clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);

			// Compute the effective mass.
			float rnA = b2Cross(rA, normal);
			float rnB = b2Cross(rB, normal);
			float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			float impulse = K > 0.0f ? -C / K : 0.0f;

			var P = impulse * normal;
			//spdlog::default_logger()->sinks()[0];
			LOGINFO("impluse={0} separation={1} planePoint.y={2} localPoint.y={3} clip.y={4}",
				impulse,separation,planePoint[1],manifold->localPoint[1], clipPoint[1]);
			cA -= mA * P;
			aA = AngleAxisf(-iA * b2Cross(rA, P), float3::UnitZ()) * aA;

			cB += mB * P;
			aB = AngleAxisf(iB * b2Cross(rB, P), float3::UnitZ()) * aB;
		}


	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0f * b2_linearSlop;
}
inline  void IntegrateMotion(mtransfrom& motionData,  Collid& cd, float Timestep)
{
	{
		/*cd.Velocity += cd.Acceleration + cd.DisPlace;
		cd.Acceleration = 0;
		cd.AngularVelocity += cd.AngularAcceleration;
		cd.DisPlace = cd.AngularAcceleration = 0f;*/
		// center of mass
		motionData.pos += cd.Velocity * Timestep;

		// orientation
		if (!cd.AngularVelocity.isZero())
			IntegrateOrientation(motionData.rotation, cd.AngularVelocity, Timestep);
	}
}