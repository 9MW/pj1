

namespace Collsion
{
    
     struct CollisionData
    {
         EntityData e1, e2;
        // PhysicsContext _physicsContext;
    }
    
     struct InitJob 
    {
         NativeArray<EntityData> _enetities;
        [ReadOnly]
         EntityData reference;
         unsafe void Execute(int index)
        {
            _enetities[index] = reference;
            UnsafeUtilityEx.ArrayElementAsRef<EntityData>(_enetities.GetUnsafePtr(), index).id = index;
        }
    }
    
     struct DataType : IDisposable
    {
         int _TypeId, _Size;
         NativeList<mtransfrom> _transforms;
         NativeList<BlobAssetReference<Collid>> Colliders;
         NativeList<int4> _positionBound;
         NativeList<EntityData> _enetities;
         UnsafeList _data;
         NativeQueue<uint> _pool;
         FunctionPointer<OnCollisoinEve> OnCollisoinEve;
         NativeList<InstancedDrawerMtrans.InstanceProPerty> _ProPertys;
        Allocator _allocator;
         unsafe static void* CreatDataType<T>(FunctionPointer<OnCollisoinEve> OnCollisoinEve, NativeList<mtransfrom> transforms,
            NativeList<BlobAssetReference<Collid>> Colliders, out JobHandle initjobHandle,
            EntityData reference,
            UnsafeList data, Allocator allocator = Allocator.Persistent) where T : struct
        {

            var capacity = transforms.Length;
            var positionBound = new NativeList<int4>(capacity, allocator);
            var enetities = new NativeList<EntityData>(data.Length, allocator);
            NativeList<InstancedDrawerMtrans.InstanceProPerty> ProPertys = new NativeList<InstancedDrawerMtrans.InstanceProPerty>(capacity, allocator);
            enetities.ResizeUninitialized(data.Length);
            positionBound.ResizeUninitialized(data.Length);
            ProPertys.ResizeUninitialized(data.Length);
            initjobHandle = new InitJob(){ _enetities = enetities, reference = reference }.Schedule(transforms.Length, transforms.Length / 100);

            var p = UnsafeUtility.Malloc(UnsafeUtility.SizeOf<DataType>(), UnsafeUtility.AlignOf<DataType>(), allocator);
            var lw = new DataType()
            {
                _ProPertys = ProPertys,
                _data = data,
                _transforms = transforms,
                Colliders = Colliders,
                _enetities = enetities,
                _positionBound = positionBound,
                _pool = new NativeQueue<uint>(allocator),
                _Size = UnsafeUtility.SizeOf<T>(),
                OnCollisoinEve = OnCollisoinEve,
               _allocator = allocator
            };
            UnsafeUtility.CopyStructureToPtr(ref lw, p);
            //lw.Dispose();

            return p;
        }
         void Dispose()
        {
            _positionBound.Dispose();
            _enetities.Dispose();
            _transforms.Dispose();
            Colliders.Dispose();
            _data.Dispose();
            _pool.Dispose();
            _ProPertys.Dispose();
        }

    }
     unsafe struct PtrList
    {
         UnsafePtrList UnsafePtrList;
         NativeList<int> Length;
         ref T ReadRef<T>(ref EntityData entityData)where T:struct
        {
            return ref UnsafeUtilityEx.ArrayElementAsRef<T>(UnsafePtrList.Ptr[entityData.groupid], entityData.id);
        }
    }
     struct PhysicsContext : IDisposable
    {
         UnsafePtrList _Types;
         NativeMultiHashMap<uint, EntityData> _buckets;
         NativeMultiHashMap<uint, EntityData> _StaticBuckets;
         NativeHashMap<int, byte> ExistCollider;
         float2 _CellSize;
         NativeHashMap<int, int> Type2id;
         void Dispose()
        {
            _buckets.Dispose();
            ExistCollider.Dispose();
            Type2id.Dispose(); _StaticBuckets.Dispose();
            _Types.Dispose();
        }
         PhysicsContext(int typecapacity, int capacity, int StaticCapacity, float2 CellSize, Allocator allocator = Allocator.Persistent)
        {
            Type2id = new NativeHashMap<int, int>(typecapacity, allocator);
            this._CellSize = CellSize;
            _buckets = new NativeMultiHashMap<uint, EntityData>(capacity * 33, allocator);
            _StaticBuckets = new NativeMultiHashMap<uint, EntityData>(StaticCapacity, allocator);
            _Types = new  UnsafePtrList(typecapacity, allocator);
            ExistCollider = new NativeHashMap<int, byte>(_buckets.Capacity, allocator);
        }
         unsafe ref DataType Creat<T>(FunctionPointer<OnCollisoinEve> OnCollisoinEve,
            NativeList<T> data, NativeList<mtransfrom> transfroms, EntityData ed,
            NativeList<BlobAssetReference<Collid>> cod) where T : struct
        {
            var t = typeof(T);
            int i = 0;
            unsafe
            {
                var typehash = t.GetHashCode();
                if (Type2id.TryGetValue(typehash, out var group))
                {
                    throw new Exception("type already exist");
                    //var tp =group;
                    //int id;
                    //var InstanceData = tp._data;

                    //if (tp._pool.TryGetFirstValue(typehash, out var et, out var iterator)){
                    //    do
                    //    {
                    //        if (et.Active)
                    //            throw new Exception();
                    //        et.Active = true;
                    //        Colliders[et.id] = cod[i];
                    //        UnsafeUtilityEx.ArrayElementAsRef<T>(InstanceData.Ptr, et.id) = data[i++];
                    //        _transforms[et.id] = transfroms[i++];
                    //    }
                    //    while (_pool.TryGetNextValue(out et, ref iterator) && i < transfroms.Length);
                    //    }
                    //var slice = new NativeSlice<mtransfrom>(transfroms, i);
                    //var SliceColider = new NativeSlice<BlobAssetReference<Collid>>(cod, i);
                    //var slicedata = new NativeSlice<T>(data, i);
                    //var groupid = InstanceData.Length;
                    //id=_transforms.Length;
                    //_transforms.AddRange(slice.GetUnsafePtr(),slice.Length);
                    //Colliders.AddRange(SliceColider.GetUnsafePtr(), SliceColider.Length);
                    //InstanceData.AddRange<T>(slicedata.GetUnsafePtr(), slicedata.Length);
                    //while (i<data.Length)
                    //{
                    //    var newentity = new EntityData() { Active = true, group = (InstanceEnum)group, groupid = groupid++, id = id++};
                    //    _enetities.Add(newentity);
                    //    i++;
                    //}
                }
                else
                {
                    ed.groupid = _Types.Length;
                    var InstanceData = new UnsafeList(UnsafeUtility.SizeOf<T>(), UnsafeUtility.AlignOf<T>(), data.Length, Allocator.Persistent);
                    InstanceData.AddRange<T>(data.GetUnsafePtr(), data.Length);
                    Type2id.TryAdd(t.GetHashCode(), _Types.Length);
                    _Types.Add(DataType.CreatDataType<T>(OnCollisoinEve, transfroms, cod, out var handle, ed, InstanceData));
                    handle.Complete();
                    data.Dispose();
                    //  var slice =transfroms;
                    //  var SliceColider =cod;
                    //  var slicedata = data;
                    //  var groupid = 0;
                    //int  id = _transforms.Length;
                    //  _transforms.AddRange(slice.GetUnsafePtr(), slice.Length);
                    //  Colliders.AddRange(SliceColider.GetUnsafePtr(), SliceColider.Length);
                    //  while (i < data.Length)
                    //  {
                    //      var newentity = new EntityData() { Active = true, group = (InstanceEnum)group, groupid = groupid++, id = id++ };
                    //      _enetities.Add(newentity);
                    //      i++;
                    //  }
                }
            }

            return ref UnsafeUtilityEx.AsRef<DataType>(_Types.Ptr[_Types.Length - 1]);
        }
    }
    
     struct RemoveEntity : IJobParallelForDefer
    {
         PhysicsContext Context;

         NativeArray<EntityData> indx;
         void Execute(int index)
        {
            var ed = indx[index];

            var etend = indx[indx.Length - 1];
            int group = (int)ed.group, id = ed.id;
            if (etend.groupid == group)
                etend.groupid = group;

            //Context._data[group].RemoveRangeSwapBack(type.Size, index, index + 1);
            //var spaces = Context._prevBound[ed.id]; 
            //int2 start = spaces.head(2), end = spaces.zw;

            //int2 hashPosition = int2.zero;
            //for (int x = start.x; x < end.x; ++x)
            //{
            //    hashPosition.x = x;

            //    for (int y = start.y; y < end.y; ++y)
            //    {
            //        hashPosition.y = y;
            //        var hash = math.hash(hashPosition);
            //        Context._buckets.Remove(hash, index);
            //    }
            //}
            //Context._enetities.RemoveAtSwapBack(id);
            //Context._transforms.RemoveAtSwapBack(id);
        }
    }
     interface PhyEve { FunctionPointer<OnCollisoinEve> onCollisoin{ get; } };

     delegate void OnCollisoinEve(ref CollisionData cd);
     struct EntityData
    {
         int groupid, id;
         InstanceEnum group;
        bool2 bool2;
         bool Equals(EntityData obj)
        {
            return obj.id == id && obj.groupid == groupid;
        }
         bool Active{ get = > bool2.x; set = > bool2.x = value; }
         bool Static{ get = > bool2.y; set = > bool2.y = value; }
    }
     class CollisonTest //:JobComponentSystem// UnityEngine.MonoBehaviour
    {
        static PhysicsContext context;
         static PrepareCollisionJob prepareCollision;
        static BoundedCollisionJob collisionJob;
        static  JobHandle handle;
         static int num = 2;
         int numm;
         void Start()
        {
            var all = Allocator.Persistent;
            context = new PhysicsContext(100, num, 0, new float2(2, 2));
            var data = new NativeList<Shell>(num, Allocator.Persistent);
            NativeList<mtransfrom> transfroms = new NativeList<mtransfrom>(num, all);
            NativeList<BlobAssetReference<Collid>> cod = new NativeList<BlobAssetReference<Collid>>(data.Length, all);
            data.ResizeUninitialized(num);
            transfroms.ResizeUninitialized(num);
            cod.ResizeUninitialized(data.Length);
            var squareLength = math.sqrt(transfroms.Length);
            Parallel.For(0, data.Length, (a) = >
            {
                transfroms[a] = (new mtransfrom()
                    {
                        pos = new float3() { x = (a % squareLength) * 4, y = ((int)(a / squareLength) * 0.5f) * 9 },
                        rotation = quaternion.identity
                    });
                cod[a] = (IBoxCollider.Creat(new float2(2, 4), 1, false));
            });
            var ed = new EntityData(){ Active = true, Static = false, group = InstanceEnum.O1 };
            var dp = context.Creat(Shell.onCollisoin, data, transfroms, ed, cod);
            prepareCollision = new PrepareCollisionJob(ref context, dp);
            handle = prepareCollision.Schedule(num, 100);
            handle.Complete();
            collisionJob = new BoundedCollisionJob(context, dp);
            drawer.mtransfrom = transfroms;
            drawer.ProPertys = dp._ProPertys;

        }
          void OnDestroy()
        {
            handle.Complete();
            context.Dispose();
        }
         static InstancedDrawerMtrans drawer;
        struct SetV 
        {
            [ReadOnly]
             NativeArray<mtransfrom> mtransfroms;
             NativeArray<BlobAssetReference<Collid>> blobs;
             void Execute(int index)
            {
                if (math.all(mtransfroms[index].pos.head(2) == float2.zero))
                    return;
                blobs[index].Value.Velocity = new float3(math.normalize(-mtransfroms[index].pos) * 0.3f);
            }
        }

         unsafe void Update()
        {
            handle.Complete();
            //var ExistCollider = collisionJob.ExistCollider.GetKeyValueArrays(Allocator.Temp);
            //for (int i = 0; ExistCollider.Keys.Length > 0 && i < 1000; i++)
            //{
            //    var vvv = ExistCollider.Values[i];
            //    Debug.Log($"k={ExistCollider.Keys[i]} V={vvv} posx={collisionJob._transforms[vvv.x]} y={collisionJob._transforms[vvv.y]} ");
            //}
            handle = new SetV(){ blobs = collisionJob._Colliders, mtransfroms = collisionJob._transforms }.Schedule(num, innerloop);

            collisionJob.ExistCollider.Clear();
            context._buckets.Clear();
            handle.Complete();
            //for (int i = 0; i < num; i++)
            //{
            //    prepareCollision.Execute(i);
            //}
            handle = prepareCollision.Schedule(num, innerloop, handle);
            drawer.Draw();
            for (int i = 0; i < num; i++)
            {
                PhysTst.Draw(*(IBoxCollider*)collisionJob._Colliders[i].GetUnsafePtr(), collisionJob._transforms[i], Color.white);
            }
            handle.Complete();
            for (int i = 0; i < num; i++)
            {
                collisionJob.Execute(i);
            }
            for (int i = 0; i < num; i++)
            {
                CollisionSys.IntegrateMotion(collisionJob, i);
            }
            //handle = collisionJob.Schedule(collisionJob._positionBound.Length, innerloop,handle);
            //handle = CollisionSys.IntegrateMotion(collisionJob, handle,innerloop);
        }
         void OnCreate()
        {
            //var al = Allocator.Persistent;
            //li = new NativeList<int>(10, al);
            //listWrapper = new ListWrapper() { li = li, Allocator = al };
            //var all = Allocator.Persistent;
            //context = new PhysicsContext(100, 1000, 0, new float2(2, 2));
            //int num = 1000;
            //var ed = new EntityData() { Active = true, Static = false, group = InstanceEnum.O1 };
            //var data = new NativeList<Shell>(num, Allocator.Persistent);
            //NativeList<mtransfrom> transfroms = new NativeList<mtransfrom>(num, all);
            //NativeList<BlobAssetReference<Collid>> cod = new NativeList<BlobAssetReference<Collid>>(data.Length, all);
            //context.Creat(Shell.onCollisoin, data, transfroms, ed, cod);
            Start();
        }
        //protected override JobHandle OnUpdate(JobHandle inputDeps)
        //{
        //    return inputDeps;
        //}
        const int innerloop = 300;
    }
     static class CollisionSys
    {

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
         static JobHandle IntegrateMotion(in BoundedCollisionJob boundedCollision, in JobHandle dep, int innerllopbatchcount = 100)
        {
            return new IntegrateMotionsJob(){ Collider = boundedCollision._Colliders, MotionDatas = boundedCollision._transforms, Timestep = boundedCollision._dt }
            .Schedule(boundedCollision._Colliders.Length, innerllopbatchcount, dep);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
         static void IntegrateMotion(in BoundedCollisionJob boundedCollision, int i)
        {
            IPhysics.IntegrateMotion(ref boundedCollision._transforms.refele(i), ref boundedCollision._Colliders.refele(i).Value, boundedCollision._dt);
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
         static void Add(BlobAssetReference<Collid> collid, NativeMultiHashMap<uint, int>.ParallelWriter _buckets,
            float2 CellSize, float3 pos, int index)
        {

            CollisionSys.CalculStartEndIterationInternal(CellSize, new Box2d(pos.head(2), collid.Value.ConvexR), out var start, out var end);
            int2 hashPosition = int2.zero;
            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;
                    var hash = math.hash(hashPosition);
                    _buckets.Add(hash, index);
                }
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
         static void Quantize(NativeList<uint> func, Box2d bound, ref NativeMultiHashMap<uint, int>.ParallelWriter bucket, int id, float2 CellSize)
        {
            CalculStartEndIterationInternal(CellSize, (bound), out var start, out var end);
            int2 hashPosition = int2.zero;
            for (int x = start.x; x < end.x; ++x)
            {
                hashPosition.x = x;

                for (int y = start.y; y < end.y; ++y)
                {
                    hashPosition.y = y;
                    var hash = math.hash(hashPosition);
                    func.Add(hash);
                    bucket.Add(hash, id);
                }
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
         static void CalculStartEndIterationInternal(float2 CellSize, Box2d bounds, out int2 start, out int2 end)
        {
            start = (bounds.Min / CellSize).FloorToInt();
            end = (bounds.Max / CellSize).CeilToInt();
        }
    }
}
