"""USD/PhysX authoring helpers (no Kit dependency)."""
from math import cos, degrees, sin, tau

from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade


def xform(stage, path, pos=(0, 0, 0), quat=None):
    x = UsdGeom.Xform.Define(stage, path)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat or Gf.Quatd(1, 0, 0, 0))
    return x


def rigid(stage, path, pos, quat, mass, kinematic=False, iters=(32, 4), ccd=False):
    prim = xform(stage, path, pos, quat).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(kinematic)
    if mass is not None:
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    px.CreateSolverPositionIterationCountAttr(iters[0])
    px.CreateSolverVelocityIterationCountAttr(iters[1])
    px.CreateEnableCCDAttr(ccd)
    return prim


def material(stage, path, static, dynamic, restitution=0.):
    m = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(restitution)
    px = PhysxSchema.PhysxMaterialAPI.Apply(m.GetPrim())
    px.CreateFrictionCombineModeAttr("min")
    px.CreateRestitutionCombineModeAttr("min")
    return m


def look(stage, path, rgb, rough=.5, metal=0.):
    m = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path+"/S")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return m


def bind(prim, phys=None, vis=None):
    if phys is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(phys, UsdShade.Tokens.weakerThanDescendants, "physics")
    if vis is not None:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(vis)


def prism(stage, path, a, b, collide=True, phys=None, vis=None, offset=None):
    """Convex solid between two point loops a and b (same count, same winding)."""
    n = len(a)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in list(a)+list(b)])
    faces, counts = [], []
    faces += list(range(n-1, -1, -1)); counts.append(n)
    faces += list(range(n, 2*n)); counts.append(n)
    for i in range(n):
        j = (i+1) % n
        faces += [i, j, n+j, n+i]; counts.append(4)
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateSubdivisionSchemeAttr("none")
    prim = mesh.GetPrim()
    if collide:
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        if offset is not None:
            PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateContactOffsetAttr(offset)
            PhysxSchema.PhysxCollisionAPI(prim).CreateRestOffsetAttr(0.)
    bind(prim, phys, vis)
    return prim


def extrude(stage, path, loop, z0, z1, **kw):
    """Prism of a 2-D loop (x, y) between z0 and z1 in the parent frame."""
    return prism(stage, path, [(x, y, z0) for x, y in loop], [(x, y, z1) for x, y in loop], **kw)


def circle(cx, cy, r, n=24):
    return [(cx+r*cos(i*tau/n), cy+r*sin(i*tau/n)) for i in range(n)]


def box(stage, path, center, size, collide=True, phys=None, vis=None):
    cx, cy, cz = center
    hx, hy, hz = (s/2 for s in size)
    loop = [(cx-hx, cy-hy), (cx+hx, cy-hy), (cx+hx, cy+hy), (cx-hx, cy+hy)]
    return extrude(stage, path, loop, cz-hz, cz+hz, collide=collide, phys=phys, vis=vis)


def annulus_sector(r0, r1, a0, a1, n=4):
    outer = [(r1*cos(a0+(a1-a0)*i/n), r1*sin(a0+(a1-a0)*i/n)) for i in range(n+1)]
    inner = [(r0*cos(a1-(a1-a0)*i/n), r0*sin(a1-(a1-a0)*i/n)) for i in range(n+1)]
    return outer+inner


def joint(stage, path, kind, b0, b1, pos0=(0, 0, 0), rot0=None, pos1=(0, 0, 0), rot1=None):
    cls = {"fixed": UsdPhysics.FixedJoint, "revolute": UsdPhysics.RevoluteJoint,
           "prismatic": UsdPhysics.PrismaticJoint, "d6": UsdPhysics.Joint}[kind]
    j = cls.Define(stage, path)
    if b0 is not None:
        j.CreateBody0Rel().SetTargets([b0])
    j.CreateBody1Rel().SetTargets([b1])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(Gf.Quatf(rot0 or Gf.Quatf(1, 0, 0, 0)))
    j.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    j.CreateLocalRot1Attr(Gf.Quatf(rot1 or Gf.Quatf(1, 0, 0, 0)))
    return j


def limit(j, dof, lo, hi):
    api = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
    api.CreateLowAttr(lo)
    api.CreateHighAttr(hi)


def lock(j, dofs):
    for d in dofs:
        limit(j, d, 1., -1.)


def drive(j, dof, target, k, c, fmax, vel=None, kind="force"):
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), dof)
    d.CreateTypeAttr(kind)
    d.CreateTargetPositionAttr(target)
    if vel is not None:
        d.CreateTargetVelocityAttr(vel)
    d.CreateStiffnessAttr(k)
    d.CreateDampingAttr(c)
    d.CreateMaxForceAttr(fmax)
    return d


def per_deg(k_rad):
    """USD angular drive gains are per degree."""
    return k_rad/degrees(1.)


def quat_axis(axis, deg):
    return Gf.Rotation(Gf.Vec3d(*axis), deg).GetQuat()


def cylinder(stage, path, xy, z0, z1, r, phys=None, vis=None, offset=None):
    """Native cylinder collider (PhysX custom geometry: exactly round, no polygon chords)."""
    c = UsdGeom.Cylinder.Define(stage, path)
    c.CreateAxisAttr("Z")
    c.CreateHeightAttr(z1-z0)
    c.CreateRadiusAttr(r)
    c.AddTranslateOp().Set(Gf.Vec3d(xy[0], xy[1], (z0+z1)/2))
    prim = c.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    if offset is not None:
        PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateContactOffsetAttr(offset)
        PhysxSchema.PhysxCollisionAPI(prim).CreateRestOffsetAttr(0.)
    bind(prim, phys, vis)
    return prim
