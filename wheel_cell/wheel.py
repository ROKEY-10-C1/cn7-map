"""Wheel mounting-face colliders: the CAD web triangles, optionally merged into convex pieces."""


from pxr import UsdGeom

from fitment import OFFSET
from cell_config import CONFIG


def _cross(o,a,b):
    return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])


def _convex(poly,sign):
    n=len(poly)
    return all(sign*_cross(poly[i],poly[(i+1)%n],poly[(i+2)%n])>=-1e-12 for i in range(n))


def merge_convex(triangles,max_vertices=24):
    """Hertel–Mehlhorn style merge: join polygons across shared edges while they stay convex.

    The union is exactly the input triangles, so the apertures keep their shape; PhysX just
    sees ~10x fewer convex pieces (each piece is one more broad/narrow-phase pair per contact).
    """
    key=lambda p:(round(p[0],7),round(p[1],7))
    polys=[[key(p) for p in tri] for tri in triangles]
    sign=1 if _cross(*polys[0])>0 else -1
    merged=True
    while merged:
        merged=False
        edges={}
        for index,poly in enumerate(polys):
            for i in range(len(poly)):
                edges[(poly[i],poly[(i+1)%len(poly)])]=index
        for index,poly in enumerate(polys):
            if poly is None:
                continue
            for i in range(len(poly)):
                a,b=poly[i],poly[(i+1)%len(poly)]
                other=edges.get((b,a))
                if other is None or other==index or polys[other] is None:
                    continue
                q=polys[other]
                j=q.index(b)
                # Walk this polygon from b round to a, then the neighbour from a round to b.
                k=poly.index(b)
                path=[poly[(k+m)%len(poly)] for m in range(len(poly))]          # b ... a
                qa=q.index(a)
                rest=[q[(qa+m)%len(q)] for m in range(1,len(q)-1)]              # after a, before b
                candidate=path+rest
                if len(candidate)<=max_vertices and _convex(candidate,sign):
                    polys[index]=candidate
                    polys[other]=None
                    merged=True
                    break
            if merged:
                break
        polys=[p for p in polys if p is not None]
    return polys


def collision_web(stage):
    """Use only mounting-face triangles, keeping the six apertures open on CPU."""
    web=UsdGeom.Mesh(stage.GetPrimAtPath("/AssetLibrary/Wheel_16/CenterWeb"))
    coords=web.GetPointsAttr().Get()
    cursor=0
    result=[]
    indices=web.GetFaceVertexIndicesAttr().Get()
    for n in web.GetFaceVertexCountsAttr().Get():
        face=[coords[j] for j in indices[cursor:cursor+n]]
        cursor+=n
        if n==3 and all(abs(p[2]-OFFSET)<1e-6 for p in face):
            result.append([(p[0],p[1]) for p in reversed(face)])
    if not result:
        raise ValueError("CN7 wheel mounting face is missing")
    if CONFIG["physics"]["merge_web_colliders"]:
        result=merge_convex(result)
    return result
