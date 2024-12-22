import yaml

# Define the data structure for the scene
scene_root = 
scene_name = "scans"
scene_data = {
    'scenes': [
        {
            'id': scene_name,
            'objects': [
                {
                    'is_static': True,
                    'motion': 'phys_anim/data/assets/scenes/train/SAMP/Tables/836157300c030c9a246b9f2ca347e8e3.npy',
                    'path': 'phys_anim/data/assets/scenes/train/SAMP/Tables/836157300c030c9a246b9f2ca347e8e3.urdf',
                    'object_options': {
                        'vhacd_enabled': True,
                        'vhacd_params': {
                            'resolution': 50000
                        },
                        'fix_base_link': True
                    }
                }
            ],
            'replications': 1
        }
    ]
}

# Write the data structure to a YAML file
with open('scene.yaml', 'w') as file:
    yaml.dump(scene_data, file, default_flow_style=False)

print("YAML file 'scene.yaml' has been created.")

# cmake \
# -D WITH_CUDA=ON \
# -D WITH_CUDNN=ON \
# -D OPENCV_DNN_CUDA=0N \
# -D CUDA_ARCH_BIN=8.9 \
# -D OPENCV_EXTRA_MODULES_PATH=/home/bowie/Desktop/scanAPP/3D_reconstructor/lib/opencv_contrib/modules/ ..

# -D PYTHON3_EXECUTABLE=/home/amos/anaconda3/envs/cv/bin/python \
# -D PYTHON_LIBRARIES=/home/amos/anaconda3/envs/cv/lib/python3.11/site-packages ..


# -D CMAKE_C_COMPILER=gcc-12 \
# -D CMAKE_CXX_COMPILER=g++-12 \