import sys
from pxr import Usd
st = Usd.Stage.Open(sys.argv[1])
for p in st.Traverse():
    if p.GetPath().pathString.count("/") <= 3: print(p.GetPath(), p.GetTypeName())
