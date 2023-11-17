# 1 docker image
docker.plusai.co:5050/plusai/mmdetection3d:mars  
docker.plusai.co:5050/plusai/mmdetection3d:neuralsim(包含mars但不包含vio环境)
### docker image更新方法

1. gpu14启动docker: `nvidia-docker run  -it  --ipc=host --name container_name  --mount type=bind,source=/home,target=/home  --mount type=bind,source=/mnt,target=/mnt docker.plusai.co:5050/plusai/mmdetection3d:neuralsim /bin/bash`
2. container更新环境
3. `docker commit container_name docker.plusai.co:5050/plusai/mmdetection3d:neuralsim -m "commit message"`
4. `docker push docker.plusai.co:5050/plusai/mmdetection3d:neuralsim`(如遇权限问题，找penglinag.gao设置)

# 2 VIO
### compile (optional)
if you use mars container, it's already installed
```
cd plus_vio_process/
sh build.sh
```

### run vio
```
sh run_vio.sh $bag_data $save_dir
```
# 

# 3 data generate


## omidata

指定`--data_root`，会处理`data_root`下所有子文件夹，生成数据在`normal`和`depth`文件夹下。  
如果只生成某些子文件夹的数据，使用`--seq_list`，或者直接修改`extract_mono_cues.py`中的`select_scene_ids`变量。

```python
# normal 生成
CUDA_VISIBLE_DEVICES=0 /opt/conda/envs/nr3d/bin/python /mnt/intel/data/mrb/nerf/omnidata/omnidata_tools/torch/extract_mono_cues.py --data_root /mnt/intel/data/mrb/dataset/nerf/pdb_b2_benchmark/  --verbose --task normal --normals_dirname normal 

# depth 生成
CUDA_VISIBLE_DEVICES=0 /opt/conda/envs/nr3d/bin/python /mnt/intel/data/mrb/nerf/omnidata/omnidata_tools/torch/extract_mono_cues.py --data_root /mnt/intel/data/mrb/dataset/nerf/pdb_b2_benchmark/ --verbose --task depth --depth_dirname depth
```

## semantic mask

生成的数据在`front_left_mask`和`front_right_mask`两个文件夹

```shell
cd /mnt/intel/data/mrb/nerf/mmsegmentation/ && bash generate_semantic_mask.sh /mnt/intel/data/mrb/nerf/neuralsim/20221228T111336_pdb-l4e-b0002_20_1to21.db
```

## 双目depth
生成的deph在`depth_img`和`depth_npy`下
```
cd /home/rongbo.ma/nerf/CREStereo-Pytorch && \
/opt/conda/bin/python test_plus.py --input_folder 000210_20211111T153653_j7-00010_42_1to21.db
```

