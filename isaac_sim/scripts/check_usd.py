from pxr import Usd, UsdGeom

# Path to the USD file
usd_file_path = "/usd/kimm_4ws/kimm_4ws.usd"

# Load the stage
stage = Usd.Stage.Open(usd_file_path)

# Traverse and list all prims
print("Listing all prims in the USD file:")
for prim in stage.Traverse():
    print(prim.GetPath())

