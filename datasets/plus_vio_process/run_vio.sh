bag=$1      # file or dir
out_dir=$2

echo "From bad data: $bag"
echo "To out_dir: $out_dir"

if [ ! -e "$out_dir" ]; then
    # 如果不存在，创建目录
    mkdir -p "$out_dir"
    echo "Directory created: $out_dir"
fi

if [ -f "$bag" ]; then
    # 文件
    echo "Processing bag file: $bag"
    python -m visual_odom.main --bag_name "$bag"  --cache_pre "$out_dir" --nerf True

elif [ -d "$bag" ]; then
    # 目录
    echo "Processing bag dir: $bag"
    file_count=$(find "$bag" -maxdepth 1 -name "*.db" | wc -l)
    # 遍历里面的.db文件并显示进度条
    processed_files=0
    for file in "$bag"/*.db; do
        if [ -f "$file" ]; then
            ((processed_files++))
            progress=$((processed_files * 100 / file_count))
            echo -e "\n-----------------------------"
            echo -ne "Processing file: $file [$processed_files/$file_count] [$progress%]\r"

            python -m visual_odom.main --bag_name "$file" --cache_pre "$out_dir" --nerf True
            
            echo -e "------------------------------\n"
        fi
    done

else
    echo "File or directory not found: $bag"
fi


# tags to show vio result
# --vis_track True \
# --global_cloud True \


# /mnt/intel/jupyterhub/mingyao.li/nerf_data/plus/lon120.6019952_lat31.4720272_radius100_vehicle_patternpdb-l4e-b_start_date2023-11-01_end_date2023-11-15_start_hour1_end_hour8