
cd /home/jhuai/Documents/slam_src/3D-Registration-with-Maximal-Cliques/Linux/release
datadir="/media/jhuai/BackupPlus/jhuai/results/align_coloradar"
handaligneddir=/media/jhuai/BackupPlus/jhuai/results/regis_coloradar

align_one_group() {
for i in "${!queryseqs[@]}"; do
    base=${baseseqs[0]}
    query=${queryseqs[$i]}
    echo "Aligning $base with $query"
    ./MAC --fixed_pcd="$datadir/$base/mergedmap.pcd" \
        --moving_pcd="$datadir/$query/mergedmap.pcd" \
        --output_path="$datadir/$query/" --demo
done
}

copyseqs() {
    base=${baseseqs[0]}
    mkdir -p "$outputdir/$base"
    cp "$datadir/$base/mergedmap.pcd" "$outputdir/$base/mergedmap.pcd"
    for i in "${!queryseqs[@]}"; do
        query=${queryseqs[$i]}
        mkdir -p "$outputdir/$query"
        cp "$datadir/$query/mergedmap.pcd" "$outputdir/$query/mergedmap.pcd"
    done
}

copyposes() {
    for i in "${!queryseqs[@]}"; do
        query=${queryseqs[$i]}
        cp "$handaligneddir/$query/W0_T_Wi.txt" "$datadir/$query/W0_T_Wi.txt"
    done
}


baseseqs=(edgar_classroom_run0)
queryseqs=(edgar_classroom_run3)
# align_one_group
copyposes

baseseqs=(ec_hallways_run0)
queryseqs=(ec_hallways_run1 ec_hallways_run2 ec_hallways_run3)
# align_one_group
copyposes

baseseqs=(outdoors_run0)
queryseqs=(outdoors_run1 outdoors_run3 outdoors_run7 outdoors_run8)
# align_one_group
copyposes

baseseqs=(edgar_army_run0)
queryseqs=(edgar_army_run1 edgar_army_run3)
# align_one_group
copyposes

baseseqs=(longboard_run0)
queryseqs=(longboard_run1 longboard_run2 longboard_run3 longboard_run4 longboard_run5 longboard_run6 longboard_run7)
# align_one_group
copyposes