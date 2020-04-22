#include<common/math.h>
#include<common/log.h>
namespace Phy2d
{

	struct IBoxCollider
	{
		Collid Head;
		operator Collid& ()constexpr {
			return  &Head;
		}
		float2 m_Vertices[8];
		float2 max = []() = > this[0];
		[MethodImpl(MethodImplOptions.AggressiveInlining)]
		static BlobAssetReference<Collid> Creat(float2 size, float mass, bool IsTrigger)
		{
			IBoxCollider box = { Collid { ConvexR = math.sqrt(math.dot(size, size) * 0.25f) } };
			float2 a = size * 0.5f, b = a * new float2(-1, 1), c = -a, d = new float2(a.x, -a.y);
			box->init(a, b, c, d);
			var sizesq = math.dot(size, size);
			box.Head.Mass = mass;
			box.InvInertia = 12 / (sizesq * mass);
			box.IsTrigger = IsTrigger;
			var blob = BlobAssetReference<Collid>.Create(&box, sizeof(IBoxCollider));
			return blob;
		}

		void init(float2 a, float2 b, float2 c, float2 d)
		{
			Head.colliderType = ColliderType.Box;
			// float3*  f3 =(float3*) UnsafeUtility.Malloc(sizeof(float3) * 4, UnsafeUtility.AlignOf<float3>(),Allocator.Persistent);
			this[0] = a;
			this[1] = b;
			this[2] = c;
			this[3] = d;
		}
		var& operator[](int i) constexpr {
			return m_Vertices[i];
		}
		string ToString()
		{
			string s = "";
			for (int i = 0; i < 4; i++)
			{
				s += this[i];
			}
			return s + Head;
		}
		bool HitBy(Weapon.DamageInfo HitBy)
		{
			throw new System.NotImplementedException();
		}

	};

	struct ICircleCollider
	{
		// ref Collid MyCollider.Head {get{fixed (ICircleCollider* ts= &this){ return ref ts->Head; } } }

		static BlobAssetReference<Collid> Creat(float mass, float r, bool IsTrigger)
		{
			Collid cir = { .IsTrigger = IsTrigger, .Mass = mass, .Velocity =0 };
			cir.init(r);
			var blob = BlobAssetReference<Collid>.Create(&cir, sizeof(ICircleCollider));
			return blob;
		}
		void init(float r)
		{
			Head = new Collid(){ ConvexR = r, colliderType = ColliderType.Circle };
		}
		bool HitBy(Weapon.DamageInfo HitBy)
		{
			throw new System.NotImplementedException();
		}
		/*	  Collid& ToCollid()
			{
				fixed(Collid * cd = &(Head))
				{
					return ref * cd;
				}
			}*/
	};
	struct mtransfrom
	{
		float3 pos;
		float3 Scale;
		quaternion rotation;
		/* override string ToString()
		 {
			 return pos.ToString() + "  " + rotation.ToString();
		 }
		 [MethodImpl(MethodImplOptions.AggressiveInlining)]
		 static implicit operator (float3, quaternion)(mtransfrom mtransfrom)
		 {
			 return mtransfrom;
		 }
		 [MethodImpl(MethodImplOptions.AggressiveInlining)]
		 static implicit operator mtransfrom((float3, quaternion) mtransfrom)
		 {
			 return new mtransfrom(){ pos = mtransfrom.Item1, rotation = mtransfrom.Item2 };
		 }
		 [MethodImpl(MethodImplOptions.AggressiveInlining)]
		 static implicit operator mtransfrom((quaternion, float3) mtransfrom)
		 {
			 return new mtransfrom(){ pos = mtransfrom.Item2, rotation = mtransfrom.Item1 };
		 }*/
	};
	enum  ColliderType :uint8_t
	{
		Box, Circle
	};
	struct Collid
	{
		ColliderType colliderType;
		byte Mark;
		bool IsTrigger;
		float MaxVelocitySq, MaxAngularVeocity;
		float3 Acceleration, AngularAcceleration;
		float3 Velocity, DisPlace;
		float3 AngularVelocity, FinalAngularVelocity;
		float friction;
		struct
		{
			float v;
			void operator=(float value) {
				v = 1 / value;
			}
			operator float() const { return v; }
		} InvMass;
		float AngularFriction, InvInertia;
		const float& Mass = InvMass;
		const void* data = NULL;
		float ConvexR;
		template<typename T>
		operator T& () {
			return *data;
		}
		/*float Mass(float value){  InvMass = 1 / value;  }

		float Mass() {  return InvMass / 1; }*/
		/* string ToString()
	   {
		   return $"AngularVelocity={AngularVelocity} Velocity={Velocity} AngularFriction, InvMass, InvInertia={AngularFriction + InvMass + InvInertia} friction={friction} ";
	   }*/
	};
	template<typename T>
	struct BlobAssetReference
	{
		struct
		{
			T v;
			explicit operator T& ()const { return &v; }

		}Value;
	};
	constexpr  var mul = math::mul;
	class IPhysics
	{
	public:

		 static float2 Mul(Matrix4x4 mt, float2 p) = >
		    ((float4)mt.GetColumn(0)).xy* p + ((float4)mt.GetColumn(1)).xy * p + ((float4)mt.GetColumn(3)).xy * p;

		
		 static float4x4 b2aSpace(mtransfrom& transforma, mtransfrom& transformb)
		{
		    var b2w = new float4x4(transformb.rotation, transformb.pos);
		    var w2a = math.inverse(new float4x4(transforma.rotation, transforma.pos));
		    var b2a = math.mul(w2a, b2w);
		    return b2a;
		}
		 static void IntegrateMotion(mtransfrom& motionData, ref Collid cd, float Timestep)
		 {
		     {
		         cd.Velocity += cd.Acceleration + cd.DisPlace;
		         cd.Acceleration = 0;
		         cd.AngularVelocity += cd.AngularAcceleration;
		         cd.DisPlace = cd.AngularAcceleration = 0f;
		         // center of mass
		         motionData.pos += cd.Velocity * Timestep;

		         // orientation
		         if (math.any(cd.AngularVelocity != 0))
		             IPhysics.IntegrateOrientation(ref motionData.rotation, cd.AngularVelocity, Timestep);
		     }

		     // Update velocities
		     {
		         if (math.any(cd.Velocity != float3.zero))
		         {
		             var dir = math.normalize(cd.Velocity);
		             // damping
		             var Velocity = math.clamp(1.0f - cd.friction, 0.0f, 1.0f);
		             cd.Velocity *= Velocity;// math.dot(Velocity, cd.Velocity) < 0 ? 0 : Velocity;
		             if (math.dot(cd.Velocity, cd.Velocity) > IPhysics.MaxVelocitySq)
		                 cd.Velocity = dir * IPhysics.MaxVelocity;
		         }
		         if (math.any(cd.AngularVelocity != float3.zero))
		         {
		             var af = cd.AngularVelocity - new float3(0, 0, cd.AngularFriction * Timestep);
		             cd.AngularVelocity *= math.clamp(1.0f - cd.AngularFriction, 0.0f, 1.0f);
		         }
		     }
		 }
		static bool Collision(BlobAssetReference<Collid> c1, BlobAssetReference<Collid> c2, mtransfrom& transforma, mtransfrom& transformb, float dt)
		{

			switch (c1.Value.colliderType)
			{
			case Box:
				switch (c2.Value.colliderType)
				{
				case Box:
					return BoxBox(ref UnsafeUtilityEx.AsRef<IBoxCollider>(c1.GetUnsafePtr()), ref UnsafeUtilityEx.AsRef<IBoxCollider>(c2.GetUnsafePtr()),
						ref transforma, ref transformb, dt);
					break;
				case Circle:
					return BoxCircle(ref UnsafeUtilityEx.AsRef<IBoxCollider>(c1.GetUnsafePtr()), ref UnsafeUtilityEx.AsRef<ICircleCollider>(c2.GetUnsafePtr()),
						ref transforma, ref transformb, dt);
					break;
				default: throw new Exception();
				}
				break;
			case Circle:
				switch (c2.Value.colliderType)
				{
				case Box:
					return BoxCircle(ref UnsafeUtilityEx.AsRef<IBoxCollider>(c2.GetUnsafePtr()), ref UnsafeUtilityEx.AsRef<ICircleCollider>(c1.GetUnsafePtr()),
						ref transforma, ref transformb, dt);
					break;
				case Circle:
					return CircleCircle(ref UnsafeUtilityEx.AsRef<ICircleCollider>(c2.GetUnsafePtr()), ref UnsafeUtilityEx.AsRef<ICircleCollider>(c1.GetUnsafePtr()),
						ref transforma, ref transformb, dt);
					break;
				default:
					break;
				}
				break;
			default:
				spdlog::error("Some error message with arg: {}", 1);;
			}
			return false;
		}
		//[MethodImpl(MethodImplOptions.AggressiveInlining)]
		// static float2 mul(quaternion q, float2 v)
		//{
		//    var t = 2 * cross(q.value.xyz, v);
		//    return v + q.value.w * t + cross(q.value.xyz, t);
		//}
		////(y1z2-y2z1,z1x2-z2x1,x1y2-x2y1)
		//[MethodImpl(MethodImplOptions.AggressiveInlining)]
		// static float2 Trans2d(float2 tg, mtransfrom mtransfrom)
		//{
		//    return Trans2d(tg, mtransfrom.pos.xy, mtransfrom.rotation);
		//}
		//[MethodImpl(MethodImplOptions.AggressiveInlining)]
		// static float2 Trans2d(mtransfrom mtransfrom, float2 tg)
		//{
		//    return Trans2d(tg, mtransfrom);
		//}
		//[MethodImpl(MethodImplOptions.AggressiveInlining)]
		// static float2 Trans2d(float2 tg, float2 POS, quaternion q)
		//{
		//    return POS + mul(q, tg);
		//}
		//[MethodImpl(MethodImplOptions.AggressiveInlining)]
		// static void Trans2d(ref float2 tg, mtransfrom mtransfrom)
		//{
		//    if (tg.x == float.NaN)
		//        throw new System.Exception();
		//    tg = tg + mtransfrom.pos.xy + mul(mtransfrom.rotation, tg);
		//}

		//[MethodImpl(MethodImplOptions.AggressiveInlining)]
		// static RTREE.Box2d CalculBBox(BlobAssetReference<Collid> c1, mtransfrom ts)
		//{
		//    return new RTREE.Box2d(ts.pos.xy, c1.Value.ConvexR);
		//}
		//x1=y1=0
		static float2 cross(float3 q, float2 f) = > new float2(q.z* f.y, -q.z * f.x);
		static void LinkedLoop(float2* f4, Action<float2, float2> a)
		{
			var v1 = f4[3];
			for (int i = 0; i < 4; i++)
			{
				var v2 = v1;
				v1 = f4[i];
				a(v1, v2);
			}
		}
		static bool CircleCircle(ICircleCollider& c1, ICircleCollider& c2, mtransfrom& transforma, mtransfrom& transformb, float dt)
		{
			var nm = transformb.pos - transforma.pos;
			var dissq = math.dot(nm, nm);
			var mindis = c1.R + c2.R;
			if (dissq > mindis* mindis)
				return false;
			var dv = c1.Velocity - c2.Velocity;
			ref var cd1 = ref c1.ToCollid();
			ref var cd2 = ref c2.ToCollid();
			nm = math.normalize(nm);
			var imp = math.dot(dv, nm) * (c1.Mass * c2.Mass / cd1.InvMass + cd2.InvMass) * nm;
			c1.Velocity -= imp * cd1.InvMass;
			c2.Velocity += imp * cd2.InvMass;
			return true;
		}
		static bool BoxCircle(IBoxCollider& c1, ICircleCollider& c2, mtransfrom& transforma, mtransfrom& transformb, float dt)
		{

			//var nm = transformb.pos - transforma.pos;
			//var dissq = math.dot(nm, nm);
			//var mindis = c1.R + c2.R;
			//if (dissq > mindis * mindis)
			//    return false;
			//var dv = c1.Velocity - c2.Velocity;
			//ref var cd1 = ref c1.ToCollid();
			//ref var cd2 = ref c2.ToCollid();
			//nm = math.normalize(nm);
			//var imp = math.dot(dv, nm) * (c1.Mass * c2.Mass / cd1.InvMass + cd2.InvMass) * nm;
			//c1.Velocity -= imp * cd1.InvMass;
			//c2.Velocity += imp * cd2.InvMass;
			return true;
		}
		static bool BoxBox(IBoxCollider& box1, IBoxCollider& box2,
			mtransfrom& transforma, mtransfrom& transformb, float dt)
		{

			// var b2a = b2aSpace(ref transforma, ref transformb);
			//var inside = stackalloc float2[4];
			float minDisSq = -1;

			float3 intsec = float3.zero;
			float2 intersect = default, impactnormal = float2.zero;
			int c1 = 0;
#if Debug
			float2[] transeda = new float2[4], transedb = new float2[4];
			var normals = new float2[4];
#else
			float2* transeda = stackalloc float2[4], transedb = stackalloc float2[4];
			var normals = stackalloc float2[4];
#endif
			for (int i = 0; i < 4; i++)
			{
				transeda[i] = Trans2d(box1[i], transforma);
				transedb[i] = Trans2d(box2[i], transformb);
			}
			float2 a, b = transeda[3], c, d = transedb[3];
			//check if already inside
			{
				bool inter = false;
				for (int l1 = 0; l1 < 4; l1++)
				{
					a = b;
					b = transeda[l1];
					d = transedb[3];
					for (int l2 = 0; l2 < 4; l2++)
					{
						c = d;
						d = transedb[l2];
						var Dis = math.min(minDisSq, segDisSq(ref a, ref b, ref c, ref d, ref intersect, ref impactnormal, out var intr));
						if (inter && minDisSq > Dis)
						{
							minDisSq = Dis;
							normals[c1++] = math.normalize(impactnormal);
							inter = true;
							intsec = new float3(intersect, 0);
						}
					}
				}
				if (inter)
				{
					BoxBoxIntersect(ref box1.ToCollid(), ref box2.ToCollid(), transeda, transedb, transforma, transformb);
					//    if (box1.IsTrigger == box2.IsTrigger == false)
					//{
					//    Debug.Log($"transforma={transforma}  transformb={transformb} ba={box1} bb={box2}");
					//    for (int i = 0; i < 4; i++)
					//    {
					//        Debug.Log($"i={i} a={transeda[i]} b={transedb[i]} normals={normals[i]}");
					//    }
					//}
					goto outSearch;
				}
			}
			quaternion q2, q1; float3 pos1, pos2;
			{
				if (math.any(box1.Velocity) || math.any(box2.Velocity) ||
					math.any(box1.AngularVelocity) || math.any(box2.AngularVelocity))
				{
					q1 = transforma.rotation;
					q2 = transformb.rotation;

					IntegrateOrientation(ref q1, box1.AngularVelocity, dt);
					IntegrateOrientation(ref q2, box2.AngularVelocity, dt);
					pos1 = transforma.pos + box1.Velocity * dt;
					pos2 = transformb.pos + box2.Velocity * dt;
					for (int i2 = 0; i2 < 4; i2++)
					{
						transeda[i2] = Trans2d(box1[i2], pos1.xy, q1);
						transedb[i2] = Trans2d(box2[i2], pos2.xy, q2);
					}
					b = transeda[3]; d = transedb[3];
					bool GoPrecise = false, outsch = false;
					for (int l1 = 0; l1 < 4; l1++)
					{
						a = b;
						b = transeda[l1];
						d = transedb[3];
						for (int l2 = 0; l2 < 4; l2++)
						{
							c = d;
							d = transedb[l2];
							var Dis = segDisSq(ref a, ref b, ref c, ref d, ref intersect, ref impactnormal, out var inter);

							if (inter)
							{

								if (Utility.Approximate(minDisSq, 0))
								{
									goto outSearch;
								}
								goto Precise;
								GoPrecise = true;
								if (minDisSq < EpsilonSq)
								{
									intsec.xy = intersect;
#if Debug
									Debug.Log($"transforma={transforma} \n transformb={transformb} \n  ba={box1} \n bb={box2} minDisSq={minDisSq}");
									for (int si = 0; si < 4; si++)
									{
										Debug.Log($"i={si} a={transeda[si]} b={transedb[si]} normals={normals[si]}");
									}
#endif
									outsch = true;
								}
							}
						}
					}
					//if (GoPrecise)
					//{
					//    goto Precise;
					//}
					//if (outsch)
					//{
					//    goto outSearch;
					//}
				}
				return false;
			}
		Precise:
			{ //find precise impact
				float pt = dt / 2, t1 = dt, t2 = 0;
				minDisSq = float.PositiveInfinity;

				//#if Debug
				//                Debug.Log($"transforma={transforma} \n transformb={transformb} \n  ba={box1} \n bb={box2} minDisSq={minDisSq}");
				//                for (int si = 0; si < 4; si++)
				//                {
				//                    Debug.Log($"i={si} a={transeda[si]} b={transedb[si]} normals={normals[si]}");
				//                }
				//#endif
				for (int i = 0; i < maxLoop; i++)
				{
					c1 = 0;
					bool inter = false;
					q1 = transforma.rotation;
					q2 = transformb.rotation;
					IntegrateOrientation(ref q1, box1.AngularVelocity, pt);
					IntegrateOrientation(ref q2, box2.AngularVelocity, pt);
					pos1 = transforma.pos + box1.Velocity * pt;
					pos2 = transformb.pos + box2.Velocity * pt;
					for (int i2 = 0; i2 < 4; i2++)
					{
						transeda[i2] = Trans2d(box1[i2], pos1.xy, q1);
						transedb[i2] = Trans2d(box2[i2], pos2.xy, q2);
					}
					b = transeda[3]; d = transedb[3];
					for (int l1 = 0; l1 < 4; l1++)
					{
						a = b;
						b = transeda[l1];
						d = transedb[3];
						for (int l2 = 0; l2 < 4; l2++)
						{
							c = d;
							d = transedb[l2];
							var Dis = segDisSq(ref a, ref b, ref c, ref d, ref intersect, ref impactnormal, out var isintersec);
							if (isintersec)
							{
								normals[c1++] = impactnormal;
								inter = true;
								if (Dis <= minDisSq)
								{
									minDisSq = Dis;
									intsec = new float3(intersect, 0);
								}
#if Debug

								Debug.Log("impactnormal" + impactnormal);
#endif

							}
						}
					}
					if (Utility.Approximate(minDisSq, 0))
					{
						goto outSearch;
					}
					if (inter)
					{
						t1 = pt;
						pt = t2 + (pt - t2) / 2;
					}
					else
					{
						t2 = pt;
						pt = pt + (t1 - pt) / 2;
					}

				}
			}
			throw new Exception();
		outSearch:
			if (box1.IsTrigger || box2.IsTrigger)
				return true;
			impactnormal = 0;
			var a1 = -(intsec.xy - transforma.pos.xy);
			for (int i = 0; i < c1; i++)
			{
				//normal of a
				if (math.sign(math.dot(a1, normals[i])) > 0)
				{
					impactnormal += -normals[i];
				}
				else
				{
					impactnormal += normals[i];
				}
			}
			impactnormal = math.normalize(impactnormal);
#if Debug
			Debug.Log($"intsec={intsec} impactnormal={impactnormal} a1={a1}");

#endif
			CalculCollision(0, ref box1.ToCollid(), ref box2.ToCollid(), intsec, transforma, transformb, new float3(impactnormal, 0));

			return true;

		inside:
			{
				if (box1.IsTrigger || box2.IsTrigger)
					return true;
				if (box2.Mass == float.PositiveInfinity)
				{
					if (math.any(box1.Velocity != float3.zero))
						box1.Velocity = math.normalize(box1.Velocity) * MaxObjVelocity;
				}
				else if (box1.Mass == float.PositiveInfinity)
				{
					if (math.any(box2.Velocity != float3.zero))
						box2.Velocity = math.normalize(box1.Velocity) * MaxObjVelocity;
				}
				else
				{
					if (math.any(box1.Velocity != float3.zero))
						box1.Velocity = math.normalize(box1.Velocity) * MaxObjVelocity;
					if (math.any(box2.Velocity != float3.zero))
						box2.Velocity = math.normalize(box1.Velocity) * MaxObjVelocity;
				}
			}
			return true;
		}
		static void CalculCollision(float restitution, ref Collid ca, ref Collid cb, float3 impact, in mtransfrom transforma, in mtransfrom transformb, float3 normala)
		{
			var la = impact - transforma.pos; var lb = impact - transformb.pos;
			var va = ca.Velocity - math.cross(ca.AngularVelocity, la);
			var vcp = math.dot(va, normala);

			var vb = cb.Velocity - math.cross(cb.AngularVelocity, lb);
			//            if (vcp < 0)
			//            {
			//#if Debug
			//                Debug.Log($"N={normala} va={va} dot={vcp} vb={vb}");
			//#endif
			//                return;
			//            }
			float r1 = math.dot(new float2(-la.y, la.x) * Utility.Determinant(la.xy, normala.xy) * ca.InvInertia, normala.xy),
				r2 = math.dot(new float2(-lb.y, lb.x) * Utility.Determinant(lb.xy, normala.xy) * cb.InvInertia, normala.xy);
			var imp = (restitution + 1) * math.dot((va - vb), normala) / (ca.InvMass + cb.InvMass + r1 + r2) * -normala;// *0.02f;
			var afl = ca.Velocity + math.dot(imp, la) * math.normalize(la) * ca.InvMass;
			cb.Acceleration -= math.dot(imp, lb) * math.normalize(lb) * cb.InvMass;
			ca.Acceleration += math.dot(afl, normala) < 0 ? afl : 0;
			var af = ca.AngularVelocity + math.cross(imp, la) * ca.InvInertia;
			var afd = math.dot(ca.AngularVelocity, af);
			if (afd > 0)
			{
#if Debug
				Debug.Log($"sameside imp={imp} normal={normala} aa={af} av={ca.AngularVelocity}");
#endif
				ca.AngularAcceleration -= ca.AngularVelocity;
			}
			else
			{
				ca.AngularAcceleration += af;
			}
			//ca.AngularVelocity = af;
			var bf = cb.AngularVelocity - math.cross(imp, lb) * cb.InvInertia;
			if (math.dot(bf, cb.AngularVelocity) > 0)
			{

#if Debug
				Debug.Log($"sameside imp={imp} normal={normala} bf={bf} av={cb.AngularVelocity}");
#endif
				cb.AngularAcceleration -= cb.AngularVelocity;
			}
			else
				ca.AngularAcceleration += bf;
		}
		static void IntegrateOrientation(ref quaternion orientation, float3 angularVelocity, float timestep)
		{
			quaternion dq = IntegrateAngularVelocity(angularVelocity, timestep);
			quaternion r = math.mul(orientation, dq);
			orientation = math.normalize(r);
		}

		// Returns a non-normalized quaternion that approximates the change in angle angularVelocity * timestep.
		static quaternion IntegrateAngularVelocity(float3 angularVelocity, float timestep)
		{
			float3 halfDeltaTime = new float3(timestep * 0.5f);
			float3 halfDeltaAngle = angularVelocity * halfDeltaTime;
			return new quaternion(new float4(halfDeltaAngle, 1.0f));
		}
		static float segDisSq(float2& a, float2& b, float2& c,
			float2& d, float2& intersect, float2& normal, bool& Intersect)
		{
			var ddd = math::dot(d, a);
			float minDisSq = NULL;
			Intersect = false;
			bool parallel;
			register float2 ab, cd, ac, normal;
			if (Utility::doIntersect(a, b, c, d, parallel, intersect, minDisSq, ab, cd, ac, normal))
			{
				Intersect = true;
			}
			else if (parallel)
			{
				float l1 = math::dot(ab, ab), l2 = math::dot(cd, cd), l3 = float::PositiveInfinity, pj1, pj2;
				normal = Utility::Rotate90(ab);

				Intersect = true;
				float2 pp1, pp2, aa = a, bb = b;
				if (math::dot(ab, cd) < 0)
				{
					aa = b;
					bb = a;
				}
				pp1 = Utility::ProjectUnrestrict(aa, c, d, out pj1);
				pp2 = Utility::ProjectUnrestrict(bb, c, d, out pj2);
				bool b1 = Epsilon0 < pj1 && pj1 < Epsilon1, b2 = Epsilon0 < pj2 && pj2 < Epsilon1;
				if (b1 && (l1 = math::distancesq(pp1, aa)) < EpsilonSq)
				{

					intersect = (pp1 + d) / 2;
				}
				else if (b2 && (l2 = math::distancesq(pp2, bb)) < EpsilonSq)
				{
					intersect = (pp2 + c) / 2;

				}
				else if ((pj1 * pj2) < 0)
				{
					pp1 = Utility::ProjectUnrestrict(c, aa, bb, out pj1);
					l3 = math::distancesq(pp1, c);
					if (l3 < EpsilonSq)
					{
						intersect = (c + d) / 2;
					}
					else
					{
						Intersect = false;
					}
				}
				else
				{
					Intersect = false;
				}
				minDisSq = math::min(l1, math::min(l2, l3));
			}
			normal = math::normalize(normal);
			return minDisSq;
		}
		static void BoxBoxIntersect(Collid box1, Collid box2, float2* transeda, float2* transedb, mtransfrom transforma, mtransfrom transformb)
		{
			float minDisSq = -1;
			int c1 = 0;
			float2 a, b = transeda[3], c, d = transedb[3], impactnormal = float2::zero, DisplaceVec = 0, normal = 0, intersect = default; bool inter = false;
			for (int l1 = 0; l1 < 4; l1++)
			{
				a = b;
				b = transeda[l1];
				d = transedb[3];
				for (int l2 = 0; l2 < 4; l2++)
				{
					c = d;
					d = transedb[l2];
					var Dis = math::min(minDisSq, segDisSq(a, b, c, d, intersect, impactnormal, intr));
					if (inter && minDisSq >= Dis)
					{
						minDisSq = Dis;
						inter = true;
						var a1 = -(intersect::xy - transforma::pos::xy);
						if (math::sign(math::dot(a1, impactnormal)) > 0)
						{
							impactnormal = -impactnormal;
						}
						normal += impactnormal;
						DisplaceVec += normal * Dis;
					}
				}
			}
			CalculCollision(0, box1, box2, new float3(intersect, 0), transforma, transformb, new float3(normal, 0));
			float m1 = box1::Mass, m2 = box2::Mass, m12 = m1 + m2;
			box1::DisPlace::xy += m1 * DisplaceVec / m12;
			box2::DisPlace::xy -= m2 * DisplaceVec / m12;
			var d2 = m2 * DisplaceVec / m12;

		}
		static void BoxBoxIntersect(Collid box1, Collid box2, float2[] transeda, float2[] transedb, in mtransfrom transforma, in mtransfrom transformb)
		{
			float minDisSq = -1;
			int c1 = 0;
			float2& a, b = transeda[3], c, d = transedb[3], impactnormal = float2::zero, DisplaceVec = 0, normal = 0, intersect = default; bool inter = false;
			for (int l1 = 0; l1 < 4; l1++)
			{
				a = b;
				b = transeda[l1];
				d = transedb[3];
				for (int l2 = 0; l2 < 4; l2++)
				{
					c = d;
					d = transedb[l2];
					var Dis = math::min(minDisSq, segDisSq(a, b, c, d, intersect, impactnormal, intr));
					if (inter && minDisSq >= Dis)
					{
						minDisSq = Dis;
						inter = true;
						var a1 = -(intersect::xy - transforma::pos::xy);
						if (math::sign(math::dot(a1, impactnormal)) > 0)
						{
							impactnormal = -impactnormal;
						}
						normal += impactnormal;
						DisplaceVec += normal * Dis;
					}
				}
			}
			CalculCollision(0, box1, box2, new float3(intersect, 0), transforma, transformb, new float3(normal, 0));
			float m1 = box1::Mass, m2 = box2::Mass, m12 = m1 + m2;
			box1.DisPlace += m1 * DisplaceVec / m12;
			box2.DisPlace -= m2 * DisplaceVec / m12;
			var d2 = m2 * DisplaceVec / m12;

		}
	};
}
