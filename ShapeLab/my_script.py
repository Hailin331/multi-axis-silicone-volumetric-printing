

from pickle import FALSE
import pymeshlab
import os

directory = '../model/isoSurface/TTA'
directory2 = '../model/isoSurface/TTA2/'

# Ensure the output directory exists
os.makedirs(directory2, exist_ok=True)

# Sort the filenames to ensure consistent indexing
obj_files = sorted([f for f in os.listdir(directory) if f.endswith('.obj')])

for index, filename in enumerate(obj_files):
    file_path = os.path.join(directory, filename)
    print(f'[{index}] Loading {file_path}')
    
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(file_path)

    # Apply smoothing
    ms.apply_coord_laplacian_smoothing(cotangentweight=0, stepsmoothnum=3)

    # Apply remeshing with different target lengths based on index
   
    ms.meshing_isotropic_explicit_remeshing(targetlen=pymeshlab.PureValue(1.5))

    # Save processed mesh
    ms.save_current_mesh(os.path.join(directory2, filename))