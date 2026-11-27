import sys
from pxr import Usd, UsdPhysics, UsdGeom
st = Usd.Stage.Open(sys.argv[1])
for p in ["/khi_rs080n/link6/rs080n2rg2", "/khi_rs080n/link5/joint6"]:
    j = UsdPhysics.Joint(st.GetPrimAtPath(p))
    print(p, j.GetLocalPos0Attr().Get(), j.GetLocalRot0Attr().Get(), j.GetLocalPos1Attr().Get(), j.GetLocalRot1Attr().Get())
xc = UsdGeom.XformCache()
for l in ["base_link","link1","link2","link3","link4","link5","link6","onrobot_rg2_base_link","gripper_center"]:
    m = xc.GetLocalToWorldTransform(st.GetPrimAtPath("/khi_rs080n/"+l))
    print(l, [round(v,4) for v in m.ExtractTranslation()], m.ExtractRotationQuat())
